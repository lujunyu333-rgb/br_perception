# -*- coding: utf-8 -*-
"""render_map_from_sdf.py — 把生成出来的 Gazebo 场地模型画成俯视图 (2026-09-11)

为什么不用 render_map.py:
  那个读 config/field_geometry.yaml 自己重画一遍, 于是和真正生成的 SDF 会脱节
  (它没有 L1 周界屏障, 中轴隔墙也是没扣洞的整条)。本脚本**只读
  scripts/out/gazebo/models/br_field/model.sdf**, 按每个 link 的实际位姿/尺寸画,
  所以图上看到的一定等于 Gazebo 里的。

用法:  python scripts/render_map_from_sdf.py
输出:  scripts/out/map_gazebo.png      全场俯视
       scripts/out/map_gazebo_zoom.png 中轴 + 南端 细节
"""
import io
import os
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, 'out', 'gazebo', 'models', 'br_field')
SDF = os.path.join(MODEL, 'model.sdf')
OUT = os.path.join(HERE, 'out')

for f in ('Microsoft YaHei', 'SimHei', 'SimSun', 'DengXian'):
    matplotlib.rcParams['font.sans-serif'] = [f]
    matplotlib.rcParams['axes.unicode_minus'] = False
    break


def obj_bbox(path):
    xs, ys = [], []
    with open(path, encoding='utf-8') as fd:
        for ln in fd:
            if ln.startswith('v '):
                _, x, y, z = ln.split()[:4]
                xs.append(float(x)); ys.append(float(y))
    return (min(xs), max(xs), min(ys), max(ys)) if xs else None


def color_of(link):
    d = link.find('.//diffuse')
    if d is None or not d.text:
        return (0.6, 0.6, 0.6, 1.0)
    v = [float(t) for t in d.text.split()]
    return (v[0], v[1], v[2], v[3] if len(v) > 3 else 1.0)


def collect():
    items = []
    for lk in ET.parse(SDF).getroot().iter('link'):
        nm = lk.get('name')
        pose = lk.find('pose')
        p = [float(v) for v in pose.text.split()] if pose is not None else [0] * 6
        cx, cy, cz = p[0], p[1], p[2]
        geo = lk.find('.//visual/geometry') or lk.find('.//collision/geometry')
        if geo is None:
            continue
        col = color_of(lk)
        box = geo.find('box/size'); cyl = geo.find('cylinder'); mesh = geo.find('mesh/uri')
        if box is not None:
            sx, sy, sz = [float(v) for v in box.text.split()]
            items.append(dict(name=nm, kind='box', x0=cx-sx/2, x1=cx+sx/2, y0=cy-sy/2, y1=cy+sy/2,
                              z0=cz-sz/2, z1=cz+sz/2, col=col))
        elif cyl is not None:
            r = float(cyl.find('radius').text); h = float(cyl.find('height').text)
            items.append(dict(name=nm, kind='cyl', cx=cx, cy=cy, r=r,
                              z0=cz-h/2, z1=cz+h/2, col=col))
        elif mesh is not None:
            fn = os.path.join(MODEL, 'meshes', os.path.basename(mesh.text))
            bb = obj_bbox(fn) if os.path.exists(fn) else None
            if bb:
                items.append(dict(name=nm, kind='box', x0=cx+bb[0], x1=cx+bb[1],
                                  y0=cy+bb[2], y1=cy+bb[3], z0=cz, z1=cz + 0.6, col=col))
    return items


def draw(ax, items, xlim, ylim, title):
    for it in sorted(items, key=lambda d: (d['z0'], d['z1'])):
        if it['kind'] == 'cyl':
            ax.add_patch(Circle((it['cx'], it['cy']), it['r'], facecolor=it['col'][:3],
                                edgecolor='#222', lw=0.8, zorder=10 + it['z1']))
        else:
            w, h = it['x1']-it['x0'], it['y1']-it['y0']
            thin = min(w, h) < 0.08          # 20mm 的墙/围栏在这个比例下几乎看不见, 加描边
            # 非薄板也给一圈淡描边: §14 里 "Level 1 Area(Red)" 与 "L2 Stairs(Red)" 同色(235-180-160),
            # 不加边就看不出台阶在哪
            ax.add_patch(Rectangle((it['x0'], it['y0']), w, h,
                                   facecolor=it['col'][:3], alpha=it['col'][3],
                                   edgecolor='#b00000' if thin else '#00000033',
                                   lw=1.6 if thin else 0.6,
                                   zorder=1 + it['z1'] * 8 + (4 if thin else 0)))
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect('equal')
    ax.grid(True, lw=0.3, alpha=0.3)
    ax.set_title(title, fontsize=10)


def save(fig, path):
    """文件被看图程序占用时(用户在图上批注过), 改存 _v2 而不是覆盖他的批注"""
    try:
        fig.savefig(path, dpi=150)
    except (PermissionError, OSError):
        alt = os.path.splitext(path)[0] + '_v2.png'
        fig.savefig(alt, dpi=150)
        print('  (%s 被占用, 改存 %s)' % (os.path.basename(path), os.path.basename(alt)))
        return alt
    return path


def main():
    items = collect()
    print('从 model.sdf 读到 %d 个 link' % len(items))

    fig, ax = plt.subplots(figsize=(11, 11))
    draw(ax, items, (-0.6, 11.6), (-0.6, 11.6),
         'ROBOCON 2027 BR 场地 — 由 scripts/out/gazebo/models/br_field/model.sdf 生成\n'
         '(所见即 Gazebo 里所见)')
    ax.set_xlabel('X 东 (m)  ·  红=西半场(我方)  蓝=东半场')
    ax.set_ylabel('Y 北 (m)')
    fig.tight_layout()
    p1 = os.path.join(OUT, 'map_gazebo.png')
    print('->', save(fig, p1))

    fig2, axs = plt.subplots(1, 3, figsize=(16, 6.2),
                             gridspec_kw={'width_ratios': [1.0, 1.0, 0.3]})
    draw(axs[0], items, (4.2, 6.8), (0.3, 2.2), '南端 · 天空棋盘（隔墙断开）')
    draw(axs[1], items, (3.3, 7.7), (3.3, 7.7), 'L2 + L1→L2 双侧阶梯 + L2 侧墙')
    draw(axs[2], items, (5.2, 5.8), (-0.4, 11.4), '中轴全段')
    fig2.tight_layout()
    p2 = os.path.join(OUT, 'map_gazebo_zoom.png')
    print('->', save(fig2, p2))

    print()
    print('中轴 x=5.5 上每一段的状态 (z 高度区分地面/L1):')
    ax_items = [i for i in items if i['kind'] == 'box'
                and i['x0'] < 5.5 < i['x1'] and (i['x1']-i['x0']) < 0.05]
    for i in sorted(ax_items, key=lambda d: d['y0']):
        print('   y[%6.2f,%6.2f] (长 %.2f)  z[%.2f,%.2f]  %s' %
              (i['y0'], i['y1'], i['y1']-i['y0'], i['z0'], i['z1'], i['name']))


if __name__ == '__main__':
    main()
