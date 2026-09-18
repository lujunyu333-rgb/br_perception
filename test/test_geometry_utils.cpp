/// ═══════════════════════════════════════════════════════════════════════════
/// test_geometry_utils.cpp — geometry_utils 单元测试 (任务书 §11.1)
///
/// ⚠ 本测试**不起 ROS** —— geometry_utils 只依赖 Eigen (设计文档 §4 硬性规则第 1 条)。
///
/// ⚠ 全部用例**确定性**: 点集由参数化网格生成 (非随机), 无 sleep、无时间依赖。
///
/// ⚠ TEST 名用中文 (贴合任务书, 与 test_coordinate_transformer.cpp 一致), 但
///   **测试组名 (第一个参数) 全是 ASCII** —— 过滤请用组名:
///   `--gtest_filter=FitSphereTest.*`, 别用中文 pattern (会撞 locale)。
///   gtest 也**没有**按组重新编译的粒度: 组只是名字前缀, 全部用例在同一次编译里。
///
/// 本文件重点钉住三处**容易写错且静默出错**的地方 (各自有专门用例):
///   1. aabb_intersection 的 max 侧必须用 a.**max** —— 写错会让 iou_3d 恒返回 0
///   2. point_segment_distance 的端点 clamp —— 漏掉会算成"到无限直线距离"
///   3. fit_sphere 的秩检查必须在 solve **之前** —— 共面点否则解出假球
/// ═══════════════════════════════════════════════════════════════════════════

#include "br_perception/utils/geometry_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace br_perception {
namespace utils {
namespace {

constexpr double kPi = 3.14159265358979323846;

/// 构造轴对齐盒子
Aabb make_box(double x0, double y0, double z0, double x1, double y1, double z1)
{
  Aabb b;
  b.min = Point3(x0, y0, z0);
  b.max = Point3(x1, y1, z1);
  return b;
}

/// 在球面上按经纬网格生成点 (避开极点, 无随机)
/// @param theta_max 极角上界: kPi = 整球, kPi/2 = 上半球 (中纬度以上)
std::vector<Point3> make_sphere_points(const Point3& center,
                                       double radius,
                                       int n_theta = 9,
                                       int n_phi = 18,
                                       double theta_max = kPi)
{
  std::vector<Point3> pts;
  for (int i = 0; i < n_theta; ++i) {
    // 从 0.5 步起步、不到 theta_max —— 避开退化极点
    const double theta = theta_max * (static_cast<double>(i) + 0.5) /
                         static_cast<double>(n_theta);
    for (int j = 0; j < n_phi; ++j) {
      const double phi = 2.0 * kPi * static_cast<double>(j) / static_cast<double>(n_phi);
      pts.push_back(center + radius * Point3(std::sin(theta) * std::cos(phi),
                                             std::sin(theta) * std::sin(phi),
                                             std::cos(theta)));
    }
  }
  return pts;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// point_plane_distance
// ═══════════════════════════════════════════════════════════════════════════

TEST(PointPlaneDistanceTest, 单位法向量的基本距离)
{
  Plane plane;
  plane.normal = Point3(0.0, 0.0, 1.0);
  plane.d = -4.0;                       // 平面 z = 4
  EXPECT_DOUBLE_EQ(1.0, point_plane_distance(Point3(1.0, 2.0, 3.0), plane));
  EXPECT_DOUBLE_EQ(0.0, point_plane_distance(Point3(7.0, -5.0, 4.0), plane));
}

TEST(PointPlaneDistanceTest, 非单位法向量结果一致)
{
  // 同一张平面 z=4 的两种等价写法: (0,0,1,-4) 与 (0,0,2,-8)
  Plane unit;
  unit.normal = Point3(0.0, 0.0, 1.0);
  unit.d = -4.0;

  Plane scaled;
  scaled.normal = Point3(0.0, 0.0, 2.0);
  scaled.d = -8.0;

  const Point3 p(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(1.0, point_plane_distance(p, unit));
  EXPECT_DOUBLE_EQ(point_plane_distance(p, unit), point_plane_distance(p, scaled));
}

TEST(PointPlaneDistanceTest, 零法向量返回NaN)
{
  Plane degenerate;
  degenerate.normal = Point3::Zero();
  degenerate.d = 1.0;
  EXPECT_TRUE(std::isnan(point_plane_distance(Point3(1.0, 1.0, 1.0), degenerate)));
}

TEST(PointPlaneDistanceTest, NaN法向量返回NaN)
{
  Plane bad;
  bad.normal = Point3(std::nan(""), 0.0, 1.0);
  bad.d = 0.0;
  EXPECT_TRUE(std::isnan(point_plane_distance(Point3(1.0, 1.0, 1.0), bad)));
}

// ═══════════════════════════════════════════════════════════════════════════
// aabb_is_valid / aabb_intersects / aabb_contains / aabb_volume
// ═══════════════════════════════════════════════════════════════════════════

TEST(AabbValidTest, 合法性判定)
{
  EXPECT_TRUE(aabb_is_valid(make_box(0, 0, 0, 1, 1, 1)));
  EXPECT_TRUE(aabb_is_valid(make_box(0, 0, 0, 0, 0, 0)));       // 退化成点仍合法
  EXPECT_TRUE(aabb_is_valid(make_box(0, 0, 0, 0, 1, 1)));       // 某轴边长为 0 仍合法
  EXPECT_FALSE(aabb_is_valid(make_box(0, 0, 0, -1, 1, 1)));     // x 轴 min > max
  EXPECT_FALSE(aabb_is_valid(make_box(0, 0, 0, 1, 1, -1)));
}

TEST(AabbIntersectsTest, 重叠与相离)
{
  const Aabb a = make_box(0, 0, 0, 2, 2, 2);
  EXPECT_TRUE(aabb_intersects(a, make_box(1, 1, 1, 3, 3, 3)));
  EXPECT_TRUE(aabb_intersects(a, make_box(-1, -1, -1, 1, 1, 1)));
  EXPECT_TRUE(aabb_intersects(a, a));
  EXPECT_FALSE(aabb_intersects(a, make_box(2.001, 0, 0, 3, 1, 1)));
  EXPECT_FALSE(aabb_intersects(a, make_box(0, 0, 5, 1, 1, 6)));
}

TEST(AabbIntersectsTest, 三种相切都算相交)
{
  // 语义决定: <= 而非 < —— 面接触 / 棱接触 / 角接触 全算相交
  const Aabb a = make_box(0, 0, 0, 2, 2, 2);

  EXPECT_TRUE(aabb_intersects(a, make_box(2, 1, 1, 4, 3, 3)));   // 面接触
  EXPECT_TRUE(aabb_intersects(a, make_box(2, 2, 1, 4, 4, 3)));   // 棱接触
  EXPECT_TRUE(aabb_intersects(a, make_box(2, 2, 2, 4, 4, 4)));   // 角接触
}

TEST(AabbIntersectsTest, 非法盒子一律不相交)
{
  const Aabb valid = make_box(0, 0, 0, 2, 2, 2);
  const Aabb invalid = make_box(0, 0, 0, -1, 2, 2);
  EXPECT_FALSE(aabb_intersects(invalid, valid));
  EXPECT_FALSE(aabb_intersects(valid, invalid));
  EXPECT_FALSE(aabb_intersects(invalid, invalid));
}

TEST(AabbContainsTest, 含边界)
{
  const Aabb box = make_box(0, 0, 0, 2, 2, 2);
  EXPECT_TRUE(aabb_contains(box, Point3(1.0, 1.0, 1.0)));
  EXPECT_TRUE(aabb_contains(box, Point3(0.0, 0.0, 0.0)));       // 角
  EXPECT_TRUE(aabb_contains(box, Point3(2.0, 1.0, 0.0)));       // 棱
  EXPECT_TRUE(aabb_contains(box, Point3(2.0, 2.0, 2.0)));       // 角
  EXPECT_FALSE(aabb_contains(box, Point3(2.001, 1.0, 1.0)));
  EXPECT_FALSE(aabb_contains(box, Point3(-0.001, 1.0, 1.0)));
}

TEST(AabbVolumeTest, 正常退化与非法)
{
  EXPECT_DOUBLE_EQ(8.0, aabb_volume(make_box(0, 0, 0, 2, 2, 2)));
  EXPECT_DOUBLE_EQ(0.0, aabb_volume(make_box(0, 0, 0, 2, 2, 0)));    // 退化: 某轴为 0
  EXPECT_DOUBLE_EQ(0.0, aabb_volume(make_box(0, 0, 0, -2, 2, 2)));   // 非法
}

// ═══════════════════════════════════════════════════════════════════════════
// aabb_intersection —— ⚠ 专门钉死 "max 侧写错" 这个静默 bug
// ═══════════════════════════════════════════════════════════════════════════

TEST(AabbIntersectionTest, 正常重叠的交集)
{
  const Aabb a = make_box(0, 0, 0, 2, 2, 2);
  const Aabb b = make_box(1, 1, 1, 3, 3, 3);

  Aabb out;
  ASSERT_TRUE(aabb_intersection(a, b, out));
  EXPECT_DOUBLE_EQ(1.0, out.min.x());
  EXPECT_DOUBLE_EQ(1.0, out.min.y());
  EXPECT_DOUBLE_EQ(1.0, out.min.z());
  EXPECT_DOUBLE_EQ(2.0, out.max.x());
  EXPECT_DOUBLE_EQ(2.0, out.max.y());
  EXPECT_DOUBLE_EQ(2.0, out.max.z());
}

TEST(AabbIntersectionTest, 交集必须合法且体积为正)
{
  // 这条是"max 侧写成 a.min"那个 bug 的专用哨兵:
  // 写错时 out 会恒为 min > max 的非法盒子 → 体积 0 → iou_3d() 恒返回 0,
  // 且不崩溃、不报错、不出 NaN。
  const Aabb a = make_box(0, 0, 0, 2, 2, 2);
  const Aabb b = make_box(1, 1, 1, 3, 3, 3);

  Aabb out;
  ASSERT_TRUE(aabb_intersection(a, b, out));
  EXPECT_TRUE(aabb_is_valid(out));
  EXPECT_GT(aabb_volume(out), 0.0);
  EXPECT_DOUBLE_EQ(1.0, aabb_volume(out));      // 1×1×1
}

TEST(AabbIntersectionTest, 相切返回合法退化盒子)
{
  const Aabb a = make_box(0, 0, 0, 2, 2, 2);
  const Aabb b = make_box(2, 1, 1, 4, 3, 3);    // 面接触

  Aabb out;
  ASSERT_TRUE(aabb_intersection(a, b, out));    // 相切 = 相交
  EXPECT_TRUE(aabb_is_valid(out));              // 但仍是合法盒子
  EXPECT_DOUBLE_EQ(0.0, aabb_volume(out));      // 体积为 0, 不是错误
  EXPECT_DOUBLE_EQ(0.0, out.size().x());        // x 轴边长为 0
}

TEST(AabbIntersectionTest, 不相交时返回false且不改out)
{
  const Aabb a = make_box(0, 0, 0, 1, 1, 1);
  const Aabb b = make_box(5, 5, 5, 6, 6, 6);

  Aabb out = make_box(-9, -9, -9, -8, -8, -8);  // 哨兵
  EXPECT_FALSE(aabb_intersection(a, b, out));
  EXPECT_DOUBLE_EQ(-9.0, out.min.x());          // out 未被修改
  EXPECT_DOUBLE_EQ(-8.0, out.max.x());
}

// ═══════════════════════════════════════════════════════════════════════════
// iou_3d
// ═══════════════════════════════════════════════════════════════════════════

TEST(Iou3dTest, 完全相同的盒子为一)
{
  const Aabb a = make_box(0, 0, 0, 2, 2, 2);
  EXPECT_DOUBLE_EQ(1.0, iou_3d(a, a));
}

TEST(Iou3dTest, 部分重叠的具体数值)
{
  // a = [0,2]³ 体积 8; b = [1,3]×[0,2]×[0,2] 体积 8
  // 交集 = [1,2]×[0,2]×[0,2] 体积 4; 并 = 8+8-4 = 12 → IoU = 4/12 = 1/3
  const Aabb a = make_box(0, 0, 0, 2, 2, 2);
  const Aabb b = make_box(1, 0, 0, 3, 2, 2);
  EXPECT_DOUBLE_EQ(1.0 / 3.0, iou_3d(a, b));
}

TEST(Iou3dTest, 一维包含)
{
  // 小盒完全在大盒内: 交集 = 小盒体积 8, 并 = 大盒 27
  const Aabb big = make_box(0, 0, 0, 3, 3, 3);
  const Aabb small = make_box(1, 1, 1, 3, 3, 3);   // 2×2×2 = 8
  EXPECT_DOUBLE_EQ(8.0 / 27.0, iou_3d(big, small));
}

TEST(Iou3dTest, 相离与相切都为零且无NaN)
{
  const Aabb a = make_box(0, 0, 0, 2, 2, 2);

  const double disjoint = iou_3d(a, make_box(5, 5, 5, 6, 6, 6));
  EXPECT_TRUE(std::isfinite(disjoint));
  EXPECT_DOUBLE_EQ(0.0, disjoint);

  // 相切: intersects 为 true, 但交集体积为 0 → IoU = 0 (看似矛盾, 实为有意)
  const double tangent = iou_3d(a, make_box(2, 0, 0, 4, 2, 2));
  EXPECT_TRUE(std::isfinite(tangent));
  EXPECT_DOUBLE_EQ(0.0, tangent);
}

TEST(Iou3dTest, 零体积盒子不出NaN)
{
  const Aabb flat = make_box(0, 0, 0, 2, 2, 0);      // 体积 0
  const Aabb solid = make_box(0, 0, 0, 2, 2, 2);

  EXPECT_TRUE(std::isfinite(iou_3d(flat, solid)));
  EXPECT_DOUBLE_EQ(0.0, iou_3d(flat, solid));
  EXPECT_TRUE(std::isfinite(iou_3d(flat, flat)));
  EXPECT_DOUBLE_EQ(0.0, iou_3d(flat, flat));         // 并体积 0 → 不得 0/0
}

TEST(Iou3dTest, 两个零体积盒子不相交)
{
  // 走的是"交集不存在 → 直接早退"这条分支, 与上一条的"并体积为 0"分支不同。
  // 两条路径都必须不出 NaN, 所以分开测。
  const Aabb flat1 = make_box(0, 0, 0, 2, 2, 0);
  const Aabb flat2 = make_box(5, 5, 0, 7, 7, 0);

  EXPECT_FALSE(aabb_intersects(flat1, flat2));
  EXPECT_TRUE(std::isfinite(iou_3d(flat1, flat2)));
  EXPECT_DOUBLE_EQ(0.0, iou_3d(flat1, flat2));
}

TEST(Iou3dTest, 非法盒子不出NaN)
{
  const Aabb invalid = make_box(0, 0, 0, -2, 2, 2);
  const Aabb valid = make_box(0, 0, 0, 2, 2, 2);
  EXPECT_TRUE(std::isfinite(iou_3d(invalid, valid)));
  EXPECT_DOUBLE_EQ(0.0, iou_3d(invalid, valid));
}

// ═══════════════════════════════════════════════════════════════════════════
// point_segment_distance —— ⚠ 端点 clamp 是重点
// ═══════════════════════════════════════════════════════════════════════════

TEST(PointSegmentDistanceTest, 投影落在线段内)
{
  const Point3 a(0.0, 0.0, 0.0);
  const Point3 b(4.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(3.0, point_segment_distance(Point3(1.0, 3.0, 0.0), a, b));
  EXPECT_DOUBLE_EQ(0.0, point_segment_distance(Point3(2.0, 0.0, 0.0), a, b));
}

TEST(PointSegmentDistanceTest, 端点外取端点距离而非直线距离)
{
  // ⚠ clamp 反例: p 在 b 之外, 垂足落在 (5,0,0)
  //    到**线段** = ‖p - b‖ = ‖(1,3,0)‖ = √10 ≈ 3.1623
  //    到**无限直线** = 3
  //    任何漏掉 clamp 的实现都会给出 3, 当场被抓。
  const Point3 a(0.0, 0.0, 0.0);
  const Point3 b(4.0, 0.0, 0.0);
  const Point3 p(5.0, 3.0, 0.0);

  const double d = point_segment_distance(p, a, b);
  EXPECT_NEAR(std::sqrt(10.0), d, 1e-12);
  EXPECT_GT(d, 3.0);                          // 必须比直线距离大
  EXPECT_NEAR(3.0, std::abs(p.y()), 1e-12);   // 直线距离确实是 3 (反证参照)
}

TEST(PointSegmentDistanceTest, a端外同样取端点距离)
{
  const Point3 a(0.0, 0.0, 0.0);
  const Point3 b(4.0, 0.0, 0.0);
  EXPECT_NEAR(std::sqrt(10.0), point_segment_distance(Point3(-1.0, 3.0, 0.0), a, b), 1e-12);
}

TEST(PointSegmentDistanceTest, 三维斜线段)
{
  const Point3 a(0.0, 0.0, 0.0);
  const Point3 b(1.0, 1.0, 1.0);
  // 线段上一点
  EXPECT_NEAR(0.0, point_segment_distance(Point3(0.5, 0.5, 0.5), a, b), 1e-12);
  // 取中点 (0.5,0.5,0.5), 垂距 = ‖(0.5,-0.5,0)‖ 投影到法平面 →
  // p - 投影点 = (0.5,-0.5,0) 中垂直于线段方向 (1,1,1)/√3 的分量
  const Point3 p(1.0, 0.0, 0.0);
  const Point3 proj(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
  EXPECT_NEAR((p - proj).norm(), point_segment_distance(p, a, b), 1e-12);
}

TEST(PointSegmentDistanceTest, 三维斜线段端点外)
{
  // clamp 在**非轴对齐**线段上同样成立 —— 上面那条反例只在 X 轴上测过。
  // p 沿 (1,1,1) 方向越过 b: 到线段的距离 = ‖p-b‖ = √3 ≈ 1.732,
  // 而 p 恰在延长线上 → 到**无限直线**的距离是 0。两者差得不含糊。
  const Point3 a(0.0, 0.0, 0.0);
  const Point3 b(1.0, 1.0, 1.0);
  const Point3 p(2.0, 2.0, 2.0);

  const double d = point_segment_distance(p, a, b);
  EXPECT_NEAR(std::sqrt(3.0), d, 1e-12);
  EXPECT_NEAR((p - b).norm(), d, 1e-12);
  EXPECT_GT(d, 0.0);                       // 漏 clamp 会得到 0
}

TEST(PointSegmentDistanceTest, 退化线段返回点距)
{
  const Point3 a(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(5.0, point_segment_distance(Point3(4.0, 6.0, 3.0), a, a));
}

// ═══════════════════════════════════════════════════════════════════════════
// fit_sphere
// ═══════════════════════════════════════════════════════════════════════════

TEST(FitSphereTest, 干净球面精确恢复)
{
  const Point3 center(1.0, 2.0, 3.0);
  const double radius = 2.0;
  const std::vector<Point3> pts = make_sphere_points(center, radius);

  Sphere s;
  ASSERT_TRUE(fit_sphere(pts, s));
  EXPECT_NEAR(center.x(), s.center.x(), 1e-9);
  EXPECT_NEAR(center.y(), s.center.y(), 1e-9);
  EXPECT_NEAR(center.z(), s.center.z(), 1e-9);
  EXPECT_NEAR(radius, s.radius, 1e-9);
}

TEST(FitSphereTest, 大坐标偏移仍能恢复)
{
  // ⚠ 容差是**故意收紧**的, 别放宽 —— 这条同时是"精度足够"的验收 和
  //    "质心中心化没被删掉"的回归哨兵。
  //    球心 (15,15,0)、半径 0.1 = 场地上最远的现实工况 (穆斯蒂卡量级)。
  //    做了中心化 → 误差 ~1e-15; 去掉中心化后 r² = |c'|² + k 里 |c'|²≈450 与
  //    k≈-450 相消, 丢 4~5 位有效数字, 半径误差劣化到 ~1e-8 → 会被 1e-9 抓住。
  const Point3 center(15.0, 15.0, 0.0);
  const double radius = 0.1;
  const std::vector<Point3> pts = make_sphere_points(center, radius);

  Sphere s;
  ASSERT_TRUE(fit_sphere(pts, s));
  EXPECT_NEAR(center.x(), s.center.x(), 1e-9);
  EXPECT_NEAR(center.y(), s.center.y(), 1e-9);
  EXPECT_NEAR(center.z(), s.center.z(), 1e-9);
  EXPECT_NEAR(radius, s.radius, 1e-9);
}

TEST(FitSphereTest, 半径随点云半径变而球心不变)
{
  // 球面上的点 → 拟合半径必须**恰为**该半径, 球心纹丝不动。
  // 跑三个差距很大的半径才真能验证"同步": 只测一个 +0.01 看不出半径与球心的关联。
  const Point3 center(-2.0, 0.5, 1.0);
  const double radii[] = {1.51, 2.0, 0.2};

  for (const double radius : radii) {
    const std::vector<Point3> pts = make_sphere_points(center, radius);
    Sphere s;
    ASSERT_TRUE(fit_sphere(pts, s)) << "半径 " << radius << " 拟合失败";
    EXPECT_NEAR(radius, s.radius, 1e-9) << "半径 " << radius;
    EXPECT_NEAR(center.x(), s.center.x(), 1e-9) << "半径 " << radius;
    EXPECT_NEAR(center.y(), s.center.y(), 1e-9) << "半径 " << radius;
    EXPECT_NEAR(center.z(), s.center.z(), 1e-9) << "半径 " << radius;
  }
}

TEST(FitSphereTest, 半球可以拟合)
{
  const Point3 center(0.0, 0.0, 0.0);
  const std::vector<Point3> pts = make_sphere_points(center, 1.0, 9, 18, kPi / 2.0);

  Sphere s;
  ASSERT_TRUE(fit_sphere(pts, s));
  EXPECT_NEAR(1.0, s.radius, 1e-6);
  EXPECT_NEAR(0.0, s.center.norm(), 1e-6);
}

TEST(FitSphereTest, 共面点判秩亏返回false)
{
  // z 恒为 0 → 法方程第 3 列全零 → 秩 3
  // ⚠ 这条钉死"先查秩再 solve": JacobiSVD::solve() 在秩亏时不报错,
  //    漏掉 rank 检查会解出一个"看起来像球"的错误结果。
  std::vector<Point3> pts;
  for (int i = 0; i < 24; ++i) {
    const double a = 2.0 * kPi * static_cast<double>(i) / 24.0;
    pts.push_back(Point3(2.0 * std::cos(a), 2.0 * std::sin(a), 0.0));
  }

  Sphere s;
  EXPECT_FALSE(fit_sphere(pts, s));
}

TEST(FitSphereTest, 共线点判秩亏返回false)
{
  std::vector<Point3> pts;
  for (int i = 0; i < 10; ++i) {
    pts.push_back(Point3(static_cast<double>(i), 0.0, 0.0));
  }

  Sphere s;
  EXPECT_FALSE(fit_sphere(pts, s));
}

TEST(FitSphereTest, 少于四点返回false)
{
  const std::vector<Point3> pts = {Point3(0.0, 0.0, 0.0),
                                   Point3(1.0, 0.0, 0.0),
                                   Point3(0.0, 1.0, 0.0)};
  Sphere s;
  EXPECT_FALSE(fit_sphere(pts, s));
}

TEST(FitSphereTest, 含NaN的点返回false)
{
  // 落在 fit_sphere 入口的有限性检查上 (不把 NaN 喂给 SVD —— 见 .cpp 注释)。
  // 若那道检查被删掉, NaN 会经 SVD 传播到 r2, 由 !(r2 > 0.0) 兜底 ——
  // 两道防线都指向 false, 所以本条只断言结果, 不绑定是哪条路径挡下的。
  std::vector<Point3> pts = make_sphere_points(Point3(1.0, 2.0, 3.0), 2.0);
  pts[5] = Point3(std::nan(""), 0.0, 0.0);

  Sphere s;
  EXPECT_FALSE(fit_sphere(pts, s));
}

TEST(FitSphereTest, 含Inf的点返回false)
{
  std::vector<Point3> pts = make_sphere_points(Point3(1.0, 2.0, 3.0), 2.0);
  pts[3] = Point3(std::numeric_limits<double>::infinity(), 0.0, 0.0);

  Sphere s;
  EXPECT_FALSE(fit_sphere(pts, s));
}

TEST(FitSphereTest, 失败时不修改输出)
{
  std::vector<Point3> pts = {Point3(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0)};
  Sphere s;
  s.center = Point3(9.0, 9.0, 9.0);
  s.radius = 99.0;

  EXPECT_FALSE(fit_sphere(pts, s));
  EXPECT_DOUBLE_EQ(9.0, s.center.x());
  EXPECT_DOUBLE_EQ(99.0, s.radius);
}

TEST(FitSphereTest, rms误差输出)
{
  const Point3 center(1.0, 2.0, 3.0);
  const std::vector<Point3> pts = make_sphere_points(center, 2.0);

  Sphere s;
  double rms = -1.0;
  ASSERT_TRUE(fit_sphere(pts, s, &rms));
  EXPECT_NEAR(0.0, rms, 1e-9);          // 干净球面 → 残差为零

  // 不传 rms_error 也必须能正常工作
  Sphere s2;
  EXPECT_TRUE(fit_sphere(pts, s2, nullptr));
  EXPECT_NEAR(s.radius, s2.radius, 1e-12);
}

// ═══════════════════════════════════════════════════════════════════════════
// transform_point / transform_points
// ═══════════════════════════════════════════════════════════════════════════

TEST(TransformTest, 单点旋转加平移)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(kPi / 2.0, Point3::UnitZ()).toRotationMatrix();
  T.translation() = Point3(1.0, 2.0, 3.0);

  // R·(1,0,0) = (0,1,0); 再 + (1,2,3) → (1,3,3)
  const Point3 out = transform_point(T, Point3(1.0, 0.0, 0.0));
  EXPECT_NEAR(1.0, out.x(), 1e-12);
  EXPECT_NEAR(3.0, out.y(), 1e-12);
  EXPECT_NEAR(3.0, out.z(), 1e-12);
}

TEST(TransformTest, 单位变换不改点)
{
  const Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Point3 p(0.3, -1.2, 4.5);
  const Point3 out = transform_point(T, p);
  EXPECT_NEAR(p.x(), out.x(), 1e-15);
  EXPECT_NEAR(p.y(), out.y(), 1e-15);
  EXPECT_NEAR(p.z(), out.z(), 1e-15);
}

TEST(TransformTest, 批量与单点结果一致)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(0.7, Point3(0.0, 0.0, 1.0)).toRotationMatrix();
  T.translation() = Point3(-1.0, 0.5, 2.0);

  std::vector<Point3> pts = {Point3(1.0, 0.0, 0.0), Point3(0.0, 1.0, 0.0),
                             Point3(0.0, 0.0, 1.0), Point3(-1.0, -2.0, -3.0)};
  std::vector<Point3> expected;
  for (const Point3& p : pts) {
    expected.push_back(transform_point(T, p));
  }

  transform_points(T, pts);
  ASSERT_EQ(expected.size(), pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    EXPECT_NEAR(expected[i].x(), pts[i].x(), 1e-12);
    EXPECT_NEAR(expected[i].y(), pts[i].y(), 1e-12);
    EXPECT_NEAR(expected[i].z(), pts[i].z(), 1e-12);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// rotate_points_about_axis
// ═══════════════════════════════════════════════════════════════════════════

TEST(RotatePointsTest, 绕z轴九十度)
{
  std::vector<Point3> pts = {Point3(1.0, 0.0, 0.0)};
  ASSERT_TRUE(rotate_points_about_axis(pts, Point3::Zero(), Point3::UnitZ(), kPi / 2.0));
  EXPECT_NEAR(0.0, pts[0].x(), 1e-12);
  EXPECT_NEAR(1.0, pts[0].y(), 1e-12);
  EXPECT_NEAR(0.0, pts[0].z(), 1e-12);
}

TEST(RotatePointsTest, 绕指定原点转轴)
{
  // p=(2,0,0), origin=(1,0,0), 绕 z 转 90°
  // p-origin=(1,0,0) → 转后 (0,1,0) → +origin → (1,1,0)
  std::vector<Point3> pts = {Point3(2.0, 0.0, 0.0)};
  ASSERT_TRUE(rotate_points_about_axis(pts, Point3(1.0, 0.0, 0.0), Point3::UnitZ(), kPi / 2.0));
  EXPECT_NEAR(1.0, pts[0].x(), 1e-12);
  EXPECT_NEAR(1.0, pts[0].y(), 1e-12);
  EXPECT_NEAR(0.0, pts[0].z(), 1e-12);
}

TEST(RotatePointsTest, 非单位轴会被内部归一化)
{
  std::vector<Point3> a = {Point3(1.0, 0.0, 0.0)};
  std::vector<Point3> b = {Point3(1.0, 0.0, 0.0)};

  ASSERT_TRUE(rotate_points_about_axis(a, Point3::Zero(), Point3::UnitZ(), kPi / 2.0));
  ASSERT_TRUE(rotate_points_about_axis(b, Point3::Zero(), Point3(0.0, 0.0, 5.0), kPi / 2.0));

  // ⚠ 用 EXPECT_DOUBLE_EQ 而非容差: (0,0,5).normalized() 在 IEEE 下**精确**得到
  //    (0,0,1) (norm=√25=5.0 精确, 5/5=1.0 精确), 两条路径的旋转矩阵应逐位相同。
  //    放宽容差会掩盖"归一化实现写错"这类错误 —— 那正是本用例要抓的东西。
  EXPECT_DOUBLE_EQ(a[0].x(), b[0].x());
  EXPECT_DOUBLE_EQ(a[0].y(), b[0].y());
  EXPECT_DOUBLE_EQ(a[0].z(), b[0].z());
}

TEST(RotatePointsTest, 零轴返回false且不改动输入)
{
  std::vector<Point3> pts = {Point3(1.0, 2.0, 3.0)};
  EXPECT_FALSE(rotate_points_about_axis(pts, Point3::Zero(), Point3::Zero(), 1.0));
  EXPECT_DOUBLE_EQ(1.0, pts[0].x());          // 输入保持不变
  EXPECT_DOUBLE_EQ(2.0, pts[0].y());
  EXPECT_DOUBLE_EQ(3.0, pts[0].z());
}

TEST(RotatePointsTest, NaN轴返回false)
{
  std::vector<Point3> pts = {Point3(1.0, 2.0, 3.0)};
  EXPECT_FALSE(rotate_points_about_axis(
      pts, Point3::Zero(), Point3(std::nan(""), 0.0, 1.0), 1.0));
  EXPECT_DOUBLE_EQ(1.0, pts[0].x());
}

TEST(RotatePointsTest, 转一整圈回到原位)
{
  std::vector<Point3> pts = {Point3(3.0, -1.0, 2.0)};
  ASSERT_TRUE(rotate_points_about_axis(pts, Point3(1.0, 1.0, 1.0),
                                       Point3(1.0, 1.0, 1.0), 2.0 * kPi));
  EXPECT_NEAR(3.0, pts[0].x(), 1e-9);
  EXPECT_NEAR(-1.0, pts[0].y(), 1e-9);
  EXPECT_NEAR(2.0, pts[0].z(), 1e-9);
}

}  // namespace utils
}  // namespace br_perception
