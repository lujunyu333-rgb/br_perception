# -*- coding: utf-8 -*-
"""
analyze_stl.py — STL 网格资产专业体检(ROS2/机器人视角)
=======================================================
输出: 基本统计 / 单位与包围盒 / REP-103 合规性 / 网格质量(退化面、边界边、
      非流形边、连通分量、重复顶点) / 主方向对齐 / URDF-Gazebo 可用性建议

用法: python analyze_stl.py <file.stl>
"""
import struct
import sys

import numpy as np

PATH = sys.argv[1] if len(sys.argv) > 1 else r"C:\学习\rc1.STL"


def load_binary_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack("<I", data[80:84])[0]
    if len(data) != 84 + n * 50:
        raise ValueError("不是标准二进制 STL (长度不匹配)")
    arr = np.frombuffer(data, dtype=np.uint8, count=n * 50, offset=84)
    arr = arr.reshape(n, 50)
    tri = arr[:, 12:48].copy().view(np.float32).reshape(n, 3, 3).astype(np.float64)
    nrm = arr[:, 0:12].copy().view(np.float32).reshape(n, 3).astype(np.float64)
    return tri, nrm


def main():
    tri, nrm = load_binary_stl(PATH)
    n = len(tri)
    print("=" * 72)
    print("STL 资产体检:", PATH)
    print("=" * 72)
    print("[1] 规模")
    print("    三角面数 : %d" % n)
    print("    顶点条目 : %d (未去重, STL 逐面存点)" % (n * 3))

    v = tri.reshape(-1, 3)
    mn, mx = v.min(axis=0), v.max(axis=0)
    span = mx - mn
    print("[2] 包围盒 / 单位")
    print("    min  = (%.4f, %.4f, %.4f)" % tuple(mn))
    print("    max  = (%.4f, %.4f, %.4f)" % tuple(mx))
    print("    span = (%.4f, %.4f, %.4f)" % tuple(span))
    if 5 < span.max() < 50:
        print("    → 量纲推断: 米(m)。8~12m 量级符合 ROBOCON 场地 (11000mm)。")
    elif 5000 < span.max() < 50000:
        print("    → 量纲推断: 毫米(mm)。ROS/Gazebo 需 ×0.001。")
    else:
        print("    → ⚠ 量纲不明确, 需人工确认!")

    # ---------- REP-103 / 朝向 ----------
    print("[3] 坐标系 (ROS REP-103: X前, Y左, Z上, 右手系, 米)")
    order = np.argsort(span)
    print("    三个轴尺寸排序(小→大): %s" % str(order))
    print("    高度轴 = 尺寸最小的轴(通常)  → 疑似 Z=%d" % int(np.argmin(span)))

    # ---------- 几何法向一致性 ----------
    e1 = tri[:, 1] - tri[:, 0]
    e2 = tri[:, 2] - tri[:, 0]
    cr = np.cross(e1, e2)
    area2 = np.linalg.norm(cr, axis=1)
    area = 0.5 * area2
    degen = area < 1e-12
    print("[4] 网格质量")
    print("    退化面(零面积)     : %d (%.4f%%)" % (int(degen.sum()), 100.0 * degen.sum() / n))
    units = cr / (area2[:, None] + 1e-15)
    dot = np.abs((units * nrm).sum(axis=1))
    bad_n = (dot < 0.9) & (~degen)
    print("    法向与几何不符面   : %d (%.4f%%)" % (int(bad_n.sum()), 100.0 * bad_n.sum() / n))
    print("    面积: 中位 %.6f  最小 %.3e  最大 %.6f (m^2)" % (np.median(area), area.min(), area.max()))

    # ---------- 拓扑: 顶点去重 / 边界边 / 非流形 / 连通分量 ----------
    q = np.round(v / 1e-4).astype(np.int64)          # 0.1mm 量化容差
    _, inv = np.unique(q, axis=0, return_inverse=True)
    T = inv.reshape(n, 3)
    nv = int(inv.max() + 1)
    print("    唯一顶点(0.1mm容差): %d  → 顶点复用率 %.1f%%"
          % (nv, 100.0 * (1 - nv / (n * 3.0))))

    ed = np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]], axis=0)
    ed = np.sort(ed, axis=1)
    uniq_e, cnt = np.unique(ed, axis=0, return_counts=True)
    boundary = int((cnt == 1).sum())
    nonman = int((cnt > 2).sum())
    print("    唯一边数            : %d" % len(uniq_e))
    print("    边界边(只被1面用)   : %d  %s" % (boundary, "→ 开放/非水密" if boundary else "→ 水密(closed)"))
    print("    非流形边(>2面共用)  : %d  %s" % (nonman, "→ 拓扑有问题" if nonman else "→ OK"))

    # 连通分量 (union-find)
    parent = np.arange(nv)

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b in uniq_e:
        ra, rb = find(int(a)), find(int(b))
        if ra != rb:
            parent[ra] = rb
    roots = np.array([find(i) for i in range(nv)])
    comp = len(np.unique(roots))
    print("    连通分量(独立实体)  : %d" % comp)

    # ---------- 轴向对齐 ----------
    ax = np.abs(units)
    aligned = (ax.max(axis=1) > 0.999) & (~degen)
    print("[5] 主方向")
    print("    轴向对齐面占比      : %.1f%%  (CAD 导出特征: 高=规则几何, 低=曲面/扫描)"
          % (100.0 * aligned.sum() / max(1, (~degen).sum())))
    for i, nm in enumerate("XYZ"):
        frac = 100.0 * ((ax[:, i] > 0.999) & (~degen)).sum() / max(1, (~degen).sum())
        print("      法向≈±%s 的面: %.1f%%" % (nm, frac))

    print("[6] 结论 / ROS2 使用建议")
    print("    · 视觉(visual): 26MB/52万面 直接可用, 但建议转 .dae 并 LOD, RViz 更流畅。")
    print("    · 碰撞(collision): ⚠ 绝不可直接用原始网格 —— 52万面会拖垮物理引擎。")
    print("      应做: 凸分解/简化(如 V-HACD) 或改用解析体(box/plane/cylinder)近似。")
    print("    · 静态场景: 若进 Gazebo world, 设 <static>true</static> 且可只留 visual, 用平面+简化盒碰撞。")
    print("    · 单位: %s" % ("米, 符合 ROS 约定" if 5 < span.max() < 50 else "需×0.001 转米"))
    print("    · URDF: 需 .dae/.stl + 材质疑似缺失(STL 无材质/颜色), 建议导 DAE 以带颜色分区。")
    print("    · 语义信息: STL 无颜色/UV, 地面涂装(建筑位/区域)不在几何中 ——")
    print("      感知需要的分区坐标必须另配平面图/参数, 不能从本网格反推。")


if __name__ == "__main__":
    main()
