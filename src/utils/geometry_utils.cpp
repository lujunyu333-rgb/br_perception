#include "br_perception/utils/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

// JacobiSVD 不在 Eigen/Core、Eigen/Geometry 里 —— 少了这行 fit_sphere 编不过
#include <Eigen/SVD>

namespace br_perception {
namespace utils {

namespace {

/// 判"零向量"的平方范数门限 = (1e-12)²。
/// 比精确判零宽一点, 挡掉数值噪声级的"几乎零"轴向量 (归一化它会放大成随机方向)。
constexpr double kTinySquaredNorm = 1e-24;

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-1 点到平面距离
// ═══════════════════════════════════════════════════════════════════════════

double point_plane_distance(const Point3& p, const Plane& plane)
{
  const double n_norm = plane.normal.norm();

  // !(x > 0) 同时挡住 0 与 NaN (法向量含 NaN 时 norm 也是 NaN)。
  // 注意: 只有**恰好为零**才算退化 —— 平面方程对任意非零倍数都成立,
  // 法向量再小 (如 1e-9) 也是合法表示, 除完仍得正确距离, 不该判退化。
  if (!(n_norm > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::abs(plane.normal.dot(p) + plane.d) / n_norm;
}

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-2 / §7.1-6 AABB 运算与 3D IoU
// ═══════════════════════════════════════════════════════════════════════════

bool aabb_is_valid(const Aabb& box)
{
  return (box.min.array() <= box.max.array()).all();
}

bool aabb_intersects(const Aabb& a, const Aabb& b)
{
  if (!aabb_is_valid(a) || !aabb_is_valid(b)) {
    return false;
  }
  // <= : **恰好相切也算相交** (接触面 / 接触边 / 接触角)。
  // 与 iou_3d() 的语义不同 —— 见头文件"AABB 两个语义别混"
  return ((a.min.array() <= b.max.array()) &&
          (b.min.array() <= a.max.array())).all();
}

bool aabb_contains(const Aabb& box, const Point3& p)
{
  // 非法盒子 (min > max) 天然不可能包含任何点, 无需额外校验
  return ((p.array() >= box.min.array()) && (p.array() <= box.max.array())).all();
}

double aabb_volume(const Aabb& box)
{
  if (!aabb_is_valid(box)) {
    return 0.0;
  }
  const Point3 s = box.size();
  return s.x() * s.y() * s.z();     // 零体积合法盒子 (某轴边长 0) 自然得 0
}

bool aabb_intersection(const Aabb& a, const Aabb& b, Aabb& out)
{
  if (!aabb_intersects(a, b)) {
    return false;
  }
  out.min = a.min.cwiseMax(b.min);
  // ⚠ 必须是 a.**max** —— 写成 a.min 会让交集盒子恒非法 (min > max),
  //    进而 aabb_volume() 恒为 0, iou_3d() 对任何相交盒子都静默返回 0。
  //    这个错误不崩溃、不报错、不出 NaN。
  out.max = a.max.cwiseMin(b.max);
  return true;
}

double iou_3d(const Aabb& a, const Aabb& b)
{
  Aabb inter;
  if (!aabb_intersection(a, b, inter)) {
    return 0.0;                      // 不相交或盒子非法
  }

  // 相切时 inter 是合法的退化盒子 (某轴边长 0) → vi = 0 → 下面算得 IoU = 0
  const double vi = aabb_volume(inter);
  const double union_v = aabb_volume(a) + aabb_volume(b) - vi;

  // !(x > 0) 同时挡掉 union_v <= 0 与 union_v = NaN —— 后者会经 0/0 污染下游
  if (!(union_v > 0.0)) {
    return 0.0;
  }
  return vi / union_v;
}

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-3 点到线段最短距离
// ═══════════════════════════════════════════════════════════════════════════

double point_segment_distance(const Point3& p, const Point3& a, const Point3& b)
{
  const Point3 ab = b - a;
  const double len2 = ab.squaredNorm();

  // 退化成点 (a == b); !(x > 0) 同时挡住 NaN
  if (!(len2 > 0.0)) {
    return (p - a).norm();
  }

  double t = (p - a).dot(ab) / len2;
  // ⚠ 端点 clamp 是本函数的全部意义: 投影落在端点外时取端点距离,
  //    而不是到**无限直线**的距离。漏掉这行的实现会在端点外侧偏小。
  t = std::clamp(t, 0.0, 1.0);

  return (p - (a + t * ab)).norm();
}

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-4 球体拟合 (代数最小二乘)
// ═══════════════════════════════════════════════════════════════════════════

bool fit_sphere(const std::vector<Point3>& points, Sphere& out, double* rms_error)
{
  const std::size_t n = points.size();
  if (n < 4) {
    return false;
  }

  // 脏输入先挡掉。不清洗就交给 SVD 是不可靠的: Jacobi 迭代的收敛判据全是比较,
  // 与 NaN 比较恒为假, 循环会提前退出、奇异值成为 NaN —— 其最终行为不属于
  // Eigen 的规范保证。与其依赖库的未定义表现, 不如在这里明确拒绝。
  for (const Point3& p : points) {
    if (!p.allFinite()) {
      return false;
    }
  }

  // ── 要点 2: 先平移到质心再列方程 ──
  // 雷达点常在 10~20 m 外, 直接解时 |p|² ≈ 400, 条件数很差。
  // 在质心坐标系里解出 c', 最后 c = 质心 + c'。数学模型不变, 只是换个原点。
  Point3 centroid = Point3::Zero();
  for (const Point3& p : points) {
    centroid += p;
  }
  centroid /= static_cast<double>(n);

  // 每个点一行: [2qᵀ, 1] · [c', k]ᵀ = ‖q‖²,  其中 k = r² - ‖c'‖²
  const Eigen::Index rows = static_cast<Eigen::Index>(n);
  Eigen::MatrixXd A(rows, 4);
  Eigen::VectorXd rhs(rows);
  for (Eigen::Index i = 0; i < rows; ++i) {
    const Point3 q = points[static_cast<std::size_t>(i)] - centroid;
    A(i, 0) = 2.0 * q.x();
    A(i, 1) = 2.0 * q.y();
    A(i, 2) = 2.0 * q.z();
    A(i, 3) = 1.0;
    rhs(i) = q.squaredNorm();
  }

  // ── 要点 1: 先查秩, 再 solve ──
  // JacobiSVD::solve() 在秩亏时**不报错**, 静默返回最小范数解 ——
  // 顺序颠倒的话, 共面点会解出一个"看起来像球"的错误结果且全程无报错。
  // (ComputeThinU | ComputeThinV 是 solve() 的前置要求)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.rank() < 4) {
    return false;                    // 共线 / 共面 → 法方程秩亏
  }

  const Eigen::VectorXd x = svd.solve(rhs);
  const Point3 center_local = x.head<3>();
  const double k = x(3);

  // ── 要点 3: !(r2 > 0.0) 而非 r2 <= 0.0 ──
  // NaN 输入会经 SVD 传播到这里; !(NaN > 0) 为真 → 一并按失败处理,
  // 与"脏数据不崩管线"同策略。
  const double r2 = center_local.squaredNorm() + k;
  if (!(r2 > 0.0)) {
    return false;
  }

  out.center = centroid + center_local;
  out.radius = std::sqrt(r2);

  if (rms_error != nullptr) {
    double acc = 0.0;
    for (const Point3& p : points) {
      const double d = (p - out.center).norm() - out.radius;
      acc += d * d;
    }
    *rms_error = std::sqrt(acc / static_cast<double>(n));
  }
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-7 / §7.1-9 坐标系快速变换与批量旋转平移
// ═══════════════════════════════════════════════════════════════════════════

Point3 transform_point(const Eigen::Isometry3d& T, const Point3& p)
{
  return T * p;
}

void transform_points(const Eigen::Isometry3d& T, std::vector<Point3>& points)
{
  for (Point3& p : points) {
    p = T * p;
  }
}

bool rotate_points_about_axis(std::vector<Point3>& points,
                              const Point3& origin,
                              const Point3& axis,
                              double angle_rad)
{
  const double n2 = axis.squaredNorm();
  // 零轴 (含数值噪声级) → 拒绝且**不改动**输入;
  // !(x > tiny) 同时挡住 NaN 轴
  if (!(n2 > kTinySquaredNorm)) {
    return false;
  }

  // 内部归一化 —— 调用方不必先归一化 (见头文件契约)
  const Eigen::Matrix3d R = Eigen::AngleAxisd(angle_rad, axis.normalized()).toRotationMatrix();
  for (Point3& p : points) {
    p = origin + R * (p - origin);   // 绕**过 origin 的轴**转, 不是绕世界原点
  }
  return true;
}

}  // namespace utils
}  // namespace br_perception
