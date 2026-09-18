#include "br_perception/communication/serial_manager.hpp"

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace br_perception {
namespace communication {

namespace {

/// 单次 read() 的缓冲。一帧最大 272 字节 payload, 512 足够装下最常见的整块到达。
constexpr std::size_t kReadChunkBytes = 512;

/// 断线期间接收线程的轮询间隔 [ms]。发送线程在负责重连 (间隔 = reconnect_interval_ms),
/// 这边只是醒来看一眼 fd 回来没有 —— 所以取得比重连间隔小得多。
constexpr int kDisconnectedPollMs = 50;

/// 波特率 → termios 常量。不支持的取值返回 B0 (= 无效)。
speed_t to_speed(int baud) noexcept
{
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:     return B0;
  }
}

/// 写结果。**刻意返回枚举而不是让调用方事后查 errno** ——
/// errno 是线程局部的但会被任何函数调用改写 (std::mutex 竞争时底层 futex 就会
/// 把 errno 设成 EAGAIN), 等调用方加完锁再读 errno, 读到的可能已经不是 write 的了。
enum class WriteResult { kOk, kWouldBlock, kError };

WriteResult write_all(int fd, const std::uint8_t* data, std::size_t len,
                      std::size_t& written) noexcept
{
  written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd, data + written, len - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n == 0) {
      return WriteResult::kError;      // len>0 时 write 返回 0 不该发生, 视为错误
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return WriteResult::kWouldBlock;
    }
    return WriteResult::kError;
  }
  return WriteResult::kOk;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════════════════════════════════

SerialConfig SerialManager::sanitize(SerialConfig config)
{
  // 夹取而不是静默接受: send_rate_hz=0 会在 1000/0 处除零 (SIGFPE 直接崩),
  // 负值会让 wait_for 立刻返回变成忙等 (CPU 100%)。两种都不该静默发生。
  config.send_rate_hz = std::max(1, std::min(1000, config.send_rate_hz));
  config.reconnect_interval_ms = std::max(0, config.reconnect_interval_ms);
  config.crc_alarm_threshold = std::max(1, config.crc_alarm_threshold);
  config.receive_poll_timeout_ms = std::max(1, config.receive_poll_timeout_ms);
  if (config.data_bits < 5 || config.data_bits > 8) {
    config.data_bits = 8;
  }
  if (config.stop_bits != 1 && config.stop_bits != 2) {
    config.stop_bits = 1;
  }
  // parity 统一转大写: 下面的 termios 分支只认 'E'/'O', 用户写 "e" 会**静默**
  // 落到 default 分支当成 'N' (无校验) —— 两端校验位不一致时收到的都是坏帧。
  config.parity = static_cast<char>(std::toupper(static_cast<unsigned char>(config.parity)));
  return config;
}

SerialManager::SerialManager(const SerialConfig& config)
  : config_(sanitize(config))
  , decoder_(config_.protocol)      // ⚠ 必须用同一份协议参数, 否则接收链路会静默全失效
{
}

SerialManager::~SerialManager()
{
  stop();
}

// ═══════════════════════════════════════════════════════════════════════════
// 生命周期
// ═══════════════════════════════════════════════════════════════════════════

bool SerialManager::start()
{
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);

  assert(!ever_started_.load() &&
         "SerialManager 不可重启: stop() 之后请构造新实例, 不要再次 start()");
  if (ever_started_.load()) {
    return false;
  }

  if (!open_device()) {
    // ⚠ ever_started_ **不能在这里置位** —— 否则"修好设备再 start()"就做不到了
    //   (第二次调用会被当成非法重启直接拒掉)。只有真正起来了才算用过。
    return false;
  }
  ever_started_.store(true);

  {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    start_time_ = std::chrono::steady_clock::now();
  }

  running_.store(true);
  send_thread_ = std::thread(&SerialManager::send_loop, this);
  receive_thread_ = std::thread(&SerialManager::receive_loop, this);
  return true;
}

void SerialManager::stop()
{
  // 串行化: 两个线程同时 stop() 时, 只有第一个做实事, 其余进来时已 join 过。
  // 没有这把锁, 两边都可能在 joinable() 为真时调 join() —— 那是 UB。
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);

  running_.store(false);
  queue_cv_.notify_all();           // 催醒可能正阻塞在 wait_for 上的发送线程

  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  close_device();
}

// ═══════════════════════════════════════════════════════════════════════════
// 串口
// ═══════════════════════════════════════════════════════════════════════════

bool SerialManager::open_device()
{
  std::lock_guard<std::mutex> lk(fd_mutex_);
  if (fd_.load() >= 0) {
    return true;
  }

  // O_NONBLOCK: 读写都不阻塞调用线程 (任务书 §5.3 明确要求)。
  // 读侧另有 poll() 等待, 所以不会变成忙轮询。
  const int fd = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return false;                   // 只试一次, 不重试、不睡眠 (重连见 reconnect())
  }

  termios tty{};
  if (::tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    return false;
  }

  const speed_t speed = to_speed(config_.baud_rate);
  if (speed == B0) {
    ::close(fd);
    return false;                   // 不支持的波特率 —— 宁可失败也不要按默认速率乱发
  }

  // ── 原始模式 (等价于 cfmakeraw, 但不依赖该函数可不可见) ──
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_cflag &= ~(CSIZE | PARENB);

  // ── 数据位 ──
  switch (config_.data_bits) {
    case 5: tty.c_cflag |= CS5; break;
    case 6: tty.c_cflag |= CS6; break;
    case 7: tty.c_cflag |= CS7; break;
    default: tty.c_cflag |= CS8; break;
  }

  // ── 校验 ──
  switch (config_.parity) {
    case 'E': tty.c_cflag |= PARENB; tty.c_cflag &= ~PARODD; break;
    case 'O': tty.c_cflag |= PARENB; tty.c_cflag |= PARODD;  break;
    default:  tty.c_cflag &= ~PARENB; break;   // 'N' 无校验
  }

  // ── 停止位 ──
  if (config_.stop_bits == 2) {
    tty.c_cflag |= CSTOPB;
  } else {
    tty.c_cflag &= ~CSTOPB;
  }

  // ── 流控 ──
  if (config_.flow_control) {
    tty.c_cflag |= CRTSCTS;
  } else {
    tty.c_cflag &= ~CRTSCTS;
  }

  tty.c_cflag |= (CLOCAL | CREAD);   // 忽略调制解调器控制线, 使能接收

  // VMIN=0/VTIME=0 配合 O_NONBLOCK: read() 立刻返回可读的字节数, 没数据返回 0。
  // ⚠ 因此 **read()==0 不能当成断线** —— 空转的串口本来就会返回 0。
  //   断线要靠 poll() 的 POLLHUP/POLLERR 判断 (见 receive_loop)。
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0 ||
      ::tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    return false;
  }
  ::tcflush(fd, TCIOFLUSH);

  fd_.store(fd);
  connected_.store(true);
  return true;
}

void SerialManager::close_device()
{
  std::lock_guard<std::mutex> lk(fd_mutex_);
  const int fd = fd_.exchange(-1);
  if (fd >= 0) {
    ::close(fd);
  }
  connected_.store(false);
}

bool SerialManager::reconnect()
{
  {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    ++stats_.reconnect_attempts;      // 尝试就计数, 成功与否都算
  }

  close_device();

  const auto wait = std::chrono::milliseconds(config_.reconnect_interval_ms);
  // 分片睡眠, 免得 stop() 要等满一个重连间隔
  const auto deadline = std::chrono::steady_clock::now() + wait;
  while (running_.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!running_.load()) {
    return false;
  }

  if (open_device()) {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    ++stats_.reconnects_succeeded;
    return true;
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// 发送侧
// ═══════════════════════════════════════════════════════════════════════════

void SerialManager::post_frame(std::vector<std::uint8_t> bytes)
{
  bool dropped = false;
  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    if (queue_.size() >= kMaxQueuedFrames) {
      queue_.pop_front();           // 丢最旧的: 陈旧的感知帧没有价值
      dropped = true;
    }
    queue_.push_back(std::move(bytes));
  }
  if (dropped) {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    ++stats_.frames_dropped_on_post;
  }
  // 唤醒发送线程 —— 投递到发出的延迟因此不受 send_rate_hz 轮询周期约束
  queue_cv_.notify_one();
}

void SerialManager::set_heartbeat_provider(HeartbeatProvider provider)
{
  std::lock_guard<std::mutex> lk(callback_mutex_);
  heartbeat_provider_ = std::move(provider);
}

void SerialManager::set_frame_callback(FrameCallback callback)
{
  std::lock_guard<std::mutex> lk(callback_mutex_);
  frame_callback_ = std::move(callback);
}

void SerialManager::send_loop()
{
  const auto tick = std::chrono::milliseconds(1000 / config_.send_rate_hz);
  const auto heartbeat_period = std::chrono::milliseconds(
      config_.heartbeat_interval_ms > 0 ? config_.heartbeat_interval_ms : 0);

  // ⚠ 等待时长取 min(tick, 心跳周期): 只看 tick 的话, send_rate_hz=1 (tick=1000ms)
  //   会让 1s 心跳抖到 100% —— 心跳本来就不该被轮询周期决定。
  const auto wait_span = (heartbeat_period.count() > 0 && heartbeat_period < tick)
                             ? heartbeat_period
                             : tick;

  auto last_heartbeat = std::chrono::steady_clock::now();

  // 上一帧没发完的剩余部分。半帧留在线上会被对端当坏帧并触发重同步,
  // 比"晚一点再发"糟得多, 所以这里保证一帧要么整帧发完, 要么一个字节都不发出去。
  std::vector<std::uint8_t> tx;
  std::size_t tx_offset = 0;
  bool tx_is_heartbeat = false;

  while (running_.load()) {
    // ── 1) 先把未发完的帧续完 ──
    if (tx_offset < tx.size()) {
      const int fd = fd_.load();
      if (fd >= 0) {
        std::size_t n = 0;
        const WriteResult result =
            write_all(fd, tx.data() + tx_offset, tx.size() - tx_offset, n);
        tx_offset += n;
        {
          std::lock_guard<std::mutex> lk(stats_mutex_);
          stats_.bytes_sent += n;
        }
        if (result == WriteResult::kError) {
          std::lock_guard<std::mutex> lk(stats_mutex_);
          ++stats_.write_errors;
          connected_.store(false);
          tx.clear();
          tx_offset = 0;
          continue;
        }
        if (result == WriteResult::kWouldBlock) {
          std::this_thread::sleep_for(tick);   // 发送缓冲满, 下一 tick 接着发
          continue;
        }
      }
      // 整帧发完才计数 —— "发出去了"应当是"完整发出去了"
      if (tx_offset >= tx.size()) {
        std::lock_guard<std::mutex> lk(stats_mutex_);
        if (tx_is_heartbeat) {
          ++stats_.heartbeats_sent;
        } else {
          ++stats_.frames_sent;
        }
        tx.clear();
        tx_offset = 0;
        tx_is_heartbeat = false;
      }
    }

    // ── 2) 心跳优先: 到点了就插, **不等队列空** ──
    // 原来的写法是"只在没帧可发时才发心跳", 于是感知帧持续积压时心跳被永久饿死 ——
    // 而心跳是保活, 被常规数据挤掉等于对端会判定链路断开。
    if (config_.heartbeat_interval_ms > 0 && tx_offset >= tx.size()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_heartbeat >= heartbeat_period) {
        HeartbeatProvider provider;
        {
          std::lock_guard<std::mutex> lk(callback_mutex_);
          provider = heartbeat_provider_;   // 取一份拷贝再调, 免得持锁执行用户代码
        }
        if (provider) {
          std::vector<std::uint8_t> hb;
          bool threw = false;
          try {
            hb = provider();
          } catch (...) {
            threw = true;
          }
          if (threw) {
            std::lock_guard<std::mutex> lk(stats_mutex_);
            ++stats_.callback_errors;
          } else if (!hb.empty()) {
            tx = std::move(hb);
            tx_offset = 0;
            tx_is_heartbeat = true;
            last_heartbeat = now;
            continue;                       // 本 tick 先把它发出去
          }
        }
        last_heartbeat = now;               // 没有 provider 或它返回空也算"看过一次"
      }
    }

    // ── 3) 取下一帧: **只取最新**, 中间的直接丢 ──
    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      queue_cv_.wait_for(lk, wait_span, [this] {
        return !queue_.empty() || !running_.load();
      });
      if (!running_.load()) {
        break;
      }
      if (!queue_.empty()) {
        const std::size_t skipped = queue_.size() - 1;
        tx = std::move(queue_.back());     // 最新的那帧
        queue_.clear();
        tx_offset = 0;
        tx_is_heartbeat = false;
        if (skipped > 0) {
          lk.unlock();
          std::lock_guard<std::mutex> slk(stats_mutex_);
          stats_.frames_dropped_on_send += skipped;
        }
      }
    }

    // ── 4) 断线重连 (只在发送线程做, 避免两条线程抢 fd) ──
    if (!connected_.load() && running_.load()) {
      reconnect();
      last_heartbeat = std::chrono::steady_clock::now();   // 重连后别立刻补发心跳
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 接收侧
// ═══════════════════════════════════════════════════════════════════════════

void SerialManager::receive_loop()
{
  std::uint8_t chunk[kReadChunkBytes];

  // 只在本线程内使用的增量状态 (不必是成员)
  std::uint64_t seen_frames = 0;
  std::uint64_t seen_crcs = 0;
  std::uint64_t crc_run = 0;                 // 自上一好帧以来的 CRC 错误数

  while (running_.load()) {
    // ── 拿一份 fd 的**副本**再去做 I/O ──
    // 直接 fd_.load() 然后 poll/read 是不安全的: 发送线程重连时会 close 旧 fd,
    // 那个 fd 号**立刻进入复用池**, 可能被任何 open/dup 拿到 (本进程别的串口、
    // 日志文件……)。于是这一轮的 poll 操作的可能根本不是串口。
    // dup() 出来的副本在内核层面独立指向同一个文件描述, 原 fd 被关也不受影响。
    // (写侧不需要这么做: fd 的关闭/重开全由发送线程自己做, 不存在跨线程竞态。)
    int local_fd = -1;
    {
      std::lock_guard<std::mutex> lk(fd_mutex_);
      const int fd = fd_.load();
      if (fd >= 0) {
        local_fd = ::dup(fd);
      }
    }
    if (local_fd < 0) {
      // 断线期间: 发送线程在负责重连, 这边安静等
      std::this_thread::sleep_for(std::chrono::milliseconds(kDisconnectedPollMs));
      continue;
    }

    pollfd pfd{};
    pfd.fd = local_fd;
    pfd.events = POLLIN;
    const int pr = ::poll(&pfd, 1, config_.receive_poll_timeout_ms);
    if (pr < 0) {
      ::close(local_fd);
      if (errno == EINTR) {
        continue;
      }
      connected_.store(false);
      continue;
    }
    if (pr == 0) {
      ::close(local_fd);
      continue;                              // 静默链路上的超时是正常的
    }

    // ⚠ 断线靠这里判断, **不是**靠 read()==0 —— VMIN=0 时空转串口本来就返回 0,
    //   把 0 当断线会让空闲链路被误判成掉线。
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      ::close(local_fd);
      connected_.store(false);
      continue;
    }
    if (!(pfd.revents & POLLIN)) {
      ::close(local_fd);
      continue;
    }

    const ssize_t n = ::read(local_fd, chunk, sizeof(chunk));
    ::close(local_fd);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      connected_.store(false);
      continue;
    }
    if (n == 0) {
      continue;                              // 无数据可读, 不是断线 (见上)
    }

    const auto frames = decoder_.feed(chunk, static_cast<std::size_t>(n));

    // ── 把 decoder 的累计量同步进 stats_ ──
    // ⚠ 必须由**接收线程**(decoder 的唯一访问者) 来做。stats() 可能在任意线程被调,
    //   直接去读 decoder_ 的非原子计数器就是数据竞争 —— 那是 UB, 不是"读到旧值"。
    const std::uint64_t total_frames = decoder_.frames_ok();
    const std::uint64_t total_crcs = decoder_.crc_errors();

    // ── CRC 连续错误告警 (任务书 §5.3: 连续 10 次 → 疑硬件干扰) ──
    // ⚠ 必须是 else if: 一批里同时出现好帧和坏帧时, 顺序信息在"两个计数差"里
    //   已经丢失了。此时保守地清零零连续计数 —— 告警宁可晚触发, 不该误触发。
    if (total_frames > seen_frames) {
      crc_run = 0;
    } else if (total_crcs > seen_crcs) {
      crc_run += total_crcs - seen_crcs;
      if (crc_run >= static_cast<std::uint64_t>(config_.crc_alarm_threshold)) {
        crc_alarm_.store(true);              // 置位后保持, 由调用方 reset
      }
    }
    seen_frames = total_frames;
    seen_crcs = total_crcs;

    {
      std::lock_guard<std::mutex> lk(stats_mutex_);
      stats_.bytes_received += static_cast<std::uint64_t>(n);
      stats_.frames_received = total_frames;
      stats_.crc_errors = total_crcs;
      stats_.decoder_pending_bytes = decoder_.pending_bytes();
    }

    if (frames.empty()) {
      continue;
    }

    FrameCallback callback;
    {
      std::lock_guard<std::mutex> lk(callback_mutex_);
      callback = frame_callback_;            // 取拷贝再调, 免得持锁执行用户代码
    }
    if (!callback) {
      continue;
    }

    for (const auto& frame : frames) {
      try {
        callback(frame);
      } catch (...) {
        // ⚠ 不接住的话接收线程会直接死掉 —— 而 connected() 仍为 true、
        //   bytes_received 照涨, 症状是"一切正常但永远收不到新帧"。
        std::lock_guard<std::mutex> lk(stats_mutex_);
        ++stats_.callback_errors;
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 诊断
// ═══════════════════════════════════════════════════════════════════════════

SerialStats SerialManager::stats() const
{
  // stats_ 由各线程在自己那边更新好, 这里只是拷一份 —— **不碰 decoder_**
  // (它的计数器由接收线程独占写, 从别的线程读是数据竞争)
  std::lock_guard<std::mutex> lk(stats_mutex_);
  SerialStats s = stats_;

  if (start_time_ != std::chrono::steady_clock::time_point{}) {
    s.uptime_ms = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_).count());
  }
  return s;
}

std::size_t SerialManager::pending_frames() const
{
  std::lock_guard<std::mutex> lk(queue_mutex_);
  return queue_.size();
}

}  // namespace communication
}  // namespace br_perception
