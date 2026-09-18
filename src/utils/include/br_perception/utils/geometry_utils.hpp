#pragma once

/// ═══════════════════════════════════════════════════════════════════════════
/// geometry_utils.hpp — ROBOCON 2027 3D 几何工具 (任务书 §7.1)
///
/// 依赖: **Eigen3** (仅头文件库) — 无 ROS / PCL / OpenCV。
///       选 Eigen 而非 PCL 的理由: 本模块要的是线性代数 (最小二乘/变换), 不是点云容器;
///       且仓库现有代码 (cluster_extractor.cpp:352-358) 已在用 Eigen。PCL 在 Ubuntu 上
///       本就把 Eigen 带进来, 故不增加实际依赖。
///
/// 职责 (§7.1 的 9 条中,**实际纳入 7 条**):
///   1. 点到平面距离              → point_plane_distance()
///   2. AABB 相交检测             → aabb_intersects() / aabb_contains() / aabb_volume()
///   3. 点到线段最短距离          → point_segment_distance()
///   4. 球体拟合 (球心 + 半径)     → fit_sphere()
///   6. 3D IoU                    → iou_3d()
///   7. 坐标系快速变换 (不用 TF)   → transform_point() / transform_points()
///   9. 批量旋转/平移             → transform_points() (Eigen::Isometry3d 同时含旋转与平移)
///
/// ⚠ 两条**刻意排除**(避免制造"第三份实现" —— 本仓库已因重复实现吃过亏,
///   见 doc/容器化重构设计_20260914.md §1.1):
///   - §7.1 第 5 条「圆柱体拟合 (RANSAC 圆柱模型)」: `src/lidar/cylinder_detector.cpp:396`
///     已有完整实现 (`pcl::SACMODEL_CYLINDER` + 法向量)。要抽公共实现应走容器化重构
///     任务 6 (搬迁 CylinderDetector), 不是在 utils 里再写一份。
///   - §7.1 第 8 条「PCA 主方向计算」: 已存在两份 —— `cluster_extractor.cpp:330-380` 与
///     `lidar_perception_node.cpp:947-1010`。统一它们属于容器化重构任务 5/8 的范围。
///     本模块**只定义 Aabb 类型与盒子间运算**, 不提供"从点集求 AABB"。
///
/// 线程安全: 无全局可变状态, 全部函数可并发调用 (纯函数)。
///
/// 数值约定:
///   - 距离/长度单位与输入一致 (本工程为 m)
///   - 退化输入 (共线/共面/零向量) 不抛异常: 返回 false 或 quiet_NaN,
///     调用方用 isfinite() / 返回值判断 —— 与 timer_utils 的"脏数据不崩管线"同策略
///   - ⚠ **消费方必须遵守的团队约定**: 所有消费 distance / IoU 结果的地方,
///     入口先做 isfinite() 检查。quiet_NaN 自身也会**静默传播** ——
///     这与本工程要防的"NaN 一路污染分类结果"是同一性质的问题,
///     只能靠约定兜住, 类型系统帮不上忙。
/// ═══════════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace br_perception {
namespace utils {

// ═══════════════════════════════════════════════════════════════════════════
// 基础类型
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 点 / 向量统一用 Eigen::Vector3d (见文件头依赖说明)
using Point3 = Eigen::Vector3d;

/// @brief 轴对齐包围盒。⚠ 构造后需保证 min ≤ max (逐分量), 否则各运算结果无意义
struct Aabb {
  Point3 min{Point3::Zero()};
  Point3 max{Point3::Zero()};

  /// 各轴边长 (max - min)
  Point3 size() const { return max - min; }
};

/// @brief 平面: normal · p + d = 0。
///
/// ⚠ **全库契约 (新增任何消费 Plane 的函数都必须遵守, 不只是 point_plane_distance)**:
///   `normal` **允许是非单位向量**。平面方程对任意非零倍数都成立, 而 RANSAC / PCL
///   直接吐出来的系数本来就不是单位向量 —— 强迫调用方先归一化是白给的负担。
///   代价是: **每一个**消费 Plane 的函数都要自己按 `|normal·p + d| / ‖normal‖` 处理,
///   不得假设 ‖normal‖ == 1。将来加 signed_distance / project_point 时同样如此。
///   normal 恰好为零向量 = 退化平面 → 消费函数应返回 quiet_NaN。
struct Plane {
  Point3 normal{Point3::UnitZ()};
  double d{0.0};
};

/// @brief 球 (球心 + 半径)
struct Sphere {
  Point3 center{Point3::Zero()};
  double radius{0.0};
};

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-1 点到平面距离
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 点 p 到平面的**绝对**距离 (非有符号)。
///
/// 内部按 |normal·p + d| / ‖normal‖ 计算 —— 故 normal 无需预先单位化。
/// normal 为零向量时返回 quiet_NaN (退化平面无距离可言)。
double point_plane_distance(const Point3& p, const Plane& plane);

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-2 / §7.1-6 AABB 运算与 3D IoU
//
// ⚠ **两个语义别混** (写死在这里, 免得调用方踩):
//     "有没有接触"     → aabb_intersects()   语义含 <=, **相切也算接触**
//     "重叠程度多大"   → iou_3d()            相切时交集体积为 0 → **IoU = 0**
//   于是相切时会得到看似矛盾的 (true, 0.0) —— 这是**有意**的: 有接触就必须处理,
//   但重叠度确实是 0。若用 iou > 0 当接触判据, 会漏掉全部相切情形。
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 盒子是否合法 (逐分量 min ≤ max)。
///
/// 其余 AABB 函数对非法盒子一律**不崩也不算**: 返回 false / 0.0。
/// 需要区分"盒子非法"与"合法但不相交"时, 调用方先自行调用本函数。
bool aabb_is_valid(const Aabb& box);

/// @brief 两 AABB 是否相交 (含**恰好相切**: 接触面/接触边也算相交)。
/// 任一盒子非法 (min > max) → 返回 false
bool aabb_intersects(const Aabb& a, const Aabb& b);

/// @brief 点是否落在盒内 (**含边界**)
bool aabb_contains(const Aabb& box, const Point3& p);

/// @brief 盒子体积; 非法盒子 (min > max) → 0.0
double aabb_volume(const Aabb& box);

/// @brief 3D IoU = 交体积 / 并体积。
/// 不相交 → 0.0; 任一盒子退化 (体积为 0) → 0.0 (避免 0/0 产生 NaN)
double iou_3d(const Aabb& a, const Aabb& b);

/// @brief 两 AABB 的交集盒子。
///
/// ⚠ 相切时返回的交集是**合法但退化**的盒子 (某轴边长恰为 0, 体积 0) —— 这不是错误。
///   实现时注意 max 侧取 `a.max.cwiseMin(b.max)` (**不是 a.min**) ——
///   写错会让交集恒为非法盒子, 从而 iou_3d() 恒返回 0, 且不报任何错。
/// @return false = 不相交或任一盒子非法 (out 不被修改)
bool aabb_intersection(const Aabb& a, const Aabb& b, Aabb& out);

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-3 点到线段最短距离
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 点 p 到线段 [a, b] 的最短距离。
///
/// p 在端点外侧时返回端点距离 (不是到无限直线的距离 —— 这是本函数与
/// "点到直线距离"的区别, 也是聚类/避障里真正要的量)。
/// a == b (退化成点) → 返回 ‖p - a‖
double point_segment_distance(const Point3& p, const Point3& a, const Point3& b);

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-4 球体拟合
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 从点集拟合球 (球心 + 半径), **代数最小二乘** (非 RANSAC)。
///
/// 展开 |p - c|² = r² 得线性形式: |p|² = 2c·p + (r² - |c|²),
/// 解 4 未知量 (cx, cy, cz, r²-|c|²) 的超定方程组。
///
/// ⚠ 这是**最小二乘**拟合, 对离群点不鲁棒 —— 输入应先用聚类/滤波剔干净
///   (雷达侧的点云聚类已经做过这件事)。
///
/// @param[in]  points    输入点集 (至少 4 个)
/// @param[out] out       拟合结果 —— 失败时**不被修改**
/// @param[out] rms_error 可选: 各点到球面的距离的均方根 (即 |‖p-c‖ - r| 的 RMS);
///                       不需要时传 nullptr
/// @return false = 无法拟合。四条触发条件:
///         点数 < 4 / 点共线或共面导致秩亏 / 解出的 r² ≤ 0 / 输入含 NaN 或 Inf。
///         失败时 out 不被修改
///
/// ⚠ **实现要点 (写 .cpp 时必须照做, 否则"共面 → false"这条保证根本不成立)**:
///   1. **先查秩, 再 solve**。Eigen 的 `JacobiSVD::solve()` 在秩亏时**不报错**,
///      而是静默返回最小范数解 —— 漏掉 `svd.rank() < 4` 这一步, 共面点会返回一个
///      "看起来像球"的错误结果, 且全程无任何报错。
///   2. **先平移到质心再解**。雷达点常在 10~20 m 外, |p|² ≈ 400 时条件数很差;
///      在质心坐标系里解出 c', 最后 c = 质心 + c'。数学模型不变, 只是换原点。
///   3. **r² 判定写 `!(r2 > 0.0)` 而不是 `r2 <= 0.0`** —— 前者顺带捕获 NaN
///      (脏点穿透聚类层进来时 SVD 会传播 NaN), 与"脏数据不崩管线"自洽。
///   4. 需要 `#include <Eigen/SVD>` —— `JacobiSVD` **不在** Core/Geometry 里。
///
/// ⚠ **小弧外推陷阱**: 若只扫到球顶一小块 (< 半球), 秩仍是 4, 本函数照样返回 true,
///   但球心是外推出来的, 会发散。调用方应对 rms_error 设阈值兜底 ——
///   穆斯蒂卡 Ø200 只看到顶部一小片时, rms 会明显高于噪声水平。
bool fit_sphere(const std::vector<Point3>& points, Sphere& out, double* rms_error = nullptr);

// ═══════════════════════════════════════════════════════════════════════════
// §7.1-7 / §7.1-9 坐标系快速变换与批量旋转平移
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 单点仿射变换 p' = T * p。
///
/// "不使用 TF 的简化版" —— 给算法类内部用 (算法类不碰 ROS, 见设计文档 §4),
/// 外参由薄壳从 config/launch 读好后以 Eigen::Isometry3d 传进来。
Point3 transform_point(const Eigen::Isometry3d& T, const Point3& p);

/// @brief 批量变换 (原地), p'_i = T * p_i。
///
/// 同时覆盖 §7.1 第 9 条"点云旋转/平移的批量操作" ——
/// 只含旋转或只含平移的 T 由调用方自行构造 (Eigen::Isometry3d 的线性部分或平移部分)。
/// 这是本模块唯一的 O(n) 接口, 但仍不分配额外内存。
void transform_points(const Eigen::Isometry3d& T, std::vector<Point3>& points);

/// @brief 批量绕**指定轴**旋转 (原地) —— 旋转轴过 origin 点。
///
/// 与 transform_points 的区别: 这个不需要构造 Isometry3d, 用于"绕竖直轴转 yaw"
/// 这类最常见的场景 (例如把点云按机器人朝向转回世界系)。
/// @param axis   旋转轴。**非单位向量会被内部归一化** —— 调用方不必先归一化
///               (少一个坑, 代价是一次 sqrt); 零向量 (‖axis‖ < 1e-12) →
///               不修改并返回 false
/// @param angle_rad 旋转角 (弧度, 右手定则)
/// @return false = axis 非法, points 保持不变
bool rotate_points_about_axis(std::vector<Point3>& points,
                              const Point3& origin,
                              const Point3& axis,
                              double angle_rad);

}  // namespace utils
}  // namespace br_perception
