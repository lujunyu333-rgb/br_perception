"""临时探针 (只读): 把 `1区栅栏` 的**每个实体**的原始顶点分开打出来。

    python .tmp_pdf/probe_ring_entities.py

动机: `--dump` 只给每个实体的**包围盒**, 而包围盒会把藏在里面的东西盖住
      (上次中轴隔墙就是这么漏的)。这次那个 80 点的"外圈围栏"实体包围盒
      是整个场地, 里面到底有几段墙看不出来 —— 只能按点拆。

复用 parse_field_assembly 的 build() 逻辑 (原样抄过来, 只为拿每个 root 自己的点集)。
"""
import os
import re
import sys
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import parse_field_assembly as P  # noqa: E402

STEP = os.path.join(os.path.dirname(__file__), '..', 'doc', 'field_assembly',
                    '2027场地总装配.STEP')
TARGET = '1区栅栏'

stmts, refs = P.load(STEP)
REF_RE = P.REF_RE

# ── 抄 build(): 找实例的 rep 与变换 T ──
geom_rep_of = {}
for i, b in stmts.items():
    if b.startswith('SHAPE_REPRESENTATION_RELATIONSHIP'):
        r = REF_RE.findall(b)
        if len(r) >= 2:
            geom_rep_of[int(r[0])] = int(r[1])

insts = []
for i, b in stmts.items():
    if not b.startswith('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION'):
        continue
    r = REF_RE.findall(b)
    if len(r) < 2:
        continue
    rb = stmts.get(int(r[0]), '')
    m1 = re.search(r"REPRESENTATION_RELATIONSHIP\s*\(\s*'[^']*'\s*,\s*'[^']*'\s*,"
                   r"\s*#(\d+)\s*,\s*#(\d+)\s*\)", rb)
    m2 = re.search(r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)\s*\)", rb)
    if not (m1 and m2):
        continue
    rep2 = int(m1.group(2))
    ir = REF_RE.findall(stmts.get(int(m2.group(1)), ''))
    if len(ir) < 2:
        continue
    a1, a2 = P.axis_of(stmts, int(ir[0])), P.axis_of(stmts, int(ir[1]))
    if a1 is None or a2 is None:
        continue
    T = P.mat_mul(P.mat_of_axis(a1), P.mat_inv_rigid(P.mat_of_axis(a2)))
    nm = re.search(r"SHAPE_REPRESENTATION\s*\(\s*'([^']*)'", stmts.get(rep2, ''))
    insts.append({'name': P.dec_name(nm.group(1)) if nm else '', 'T': T,
                  'rep': geom_rep_of.get(rep2, rep2)})

# ── 抄 build(): 引用图 BFS 取点 (AXIS2 必须跳过) ──
def rep_points(rep):
    seen, pts = set(), []
    stk = [rep]
    while stk:
        i = stk.pop()
        if i in seen:
            continue
        seen.add(i)
        b = stmts.get(i, '')
        if b.startswith('CARTESIAN_POINT'):
            p = P.point_of(stmts, i)
            if p:
                pts.append(p)
            continue
        if b.startswith('AXIS2_PLACEMENT_3D'):
            continue
        for r in refs.get(i, []):
            if r not in seen:
                stk.append(r)
    return pts


def solid_roots(rep):
    b = stmts.get(rep, '')
    items = re.search(r'\(\s*((?:#\d+[^()]*)+)\)', b)
    if not items:
        return [rep]
    roots = [int(r) for r in REF_RE.findall(items.group(1))
             if not stmts.get(int(r), '').startswith('AXIS2_PLACEMENT_3D')]
    return roots or [rep]


hit = [x for x in insts if x['name'] == TARGET]
if not hit:
    sys.exit('没找到 ' + TARGET)
inst = hit[0]

print('零件 %s: 实体 %d 个' % (TARGET, len(solid_roots(inst['rep']))))
print()

axis_pts = []      # 贴中轴的点 (X 在 ±25 内) —— 收集后按 Z 聚类
for k, root in enumerate(solid_roots(inst['rep'])):
    rp = rep_points(root)
    wp = [P.apply(inst['T'], q) for q in rp]
    xs = [q[0] for q in wp]
    zs = [q[2] for q in wp]
    ys = [q[1] for q in wp]
    print('实体 #%d: %d 点   X[%.1f,%.1f] 高度Y[%.1f,%.1f] Z[%.1f,%.1f]'
          % (k, len(rp), min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))
    near = sorted({(round(q[0], 1), round(q[2], 1)) for q in wp if abs(q[0]) <= 100})
    if near:
        print('      ⚠ 该实体里贴中轴 (|X|<=100) 的顶点 %d 个:' % len(near))
        for a, c in near:
            print('         X=%8.1f Z=%8.1f  ->  场地 (%.4f, %.4f)'
                  % (a, c, 5.5 + a / 1000, 5.5 - c / 1000))
        axis_pts.extend(near)
    print()

# 按 Z 分组, 还原"墙段" (同一段墙的点共享一组 Z 值)
by_z = defaultdict(list)
for a, c in axis_pts:
    by_z[round(c, 1)].append(round(a, 1))
print('=== 贴中轴的顶点按 Z 汇总 (场地 y = 5.5 - Z/1000) ===')
for c in sorted(by_z, key=lambda v: -v):
    print('  场地 y=%7.4f  (Z=%9.1f)  X 取值: %s'
          % (5.5 - c / 1000, c, sorted(set(by_z[c]))))
