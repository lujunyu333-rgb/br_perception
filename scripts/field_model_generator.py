# -*- coding: utf-8 -*-
"""
field_model_generator.py — 参数化场地模型生成器
================================================

用法:
    python field_model_generator.py            # 预览: 生成 yaml 到 scripts/out/ + SVG + 校验报告
    python field_model_generator.py --apply    # 生效: 覆盖 config/field_geometry.yaml (先自动备份)

输入 : scripts/field_model_spec.py   (唯一事实源)
输出 :
    config/field_geometry.yaml        (ROS2 参数文件, --apply 时覆盖)
    scripts/out/field_geometry.generated.yaml
    scripts/out/field_layout_preview.svg    (俯视图, 与官方图纸人工核对用)
    scripts/out/field_model_report.txt      (校验结果 + 待补数值清单)

坐标系: 原点西南角, X 东 Y 北 Z 上, 单位 m; 红(我方)在西半场, 沿 x=5.5 镜像出蓝方。
⚠ 位置类数值多为 DRAFT 假设, 必须与官方图纸核对后替换(见 spec 内 TODO / 报告清单)。
"""

import os
import sys
import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import field_model_spec as S  # noqa: E402

OUT_DIR = os.path.join(HERE, "out")
CONFIG_PATH = os.path.join(ROOT, "config", "field_geometry.yaml")
FIELD = S.FIELD_SIZE
MID = FIELD / 2.0


# ─────────────────────────────────────────────────────────────────────────────
# 基础工具
# ─────────────────────────────────────────────────────────────────────────────
def mirror_x(x):
    return round(FIELD - x, 3)


def mirror_rect(r):
    """rect = (x1, y1, x2, y2) 沿 x=MID 镜像"""
    x1, y1, x2, y2 = r
    return (round(FIELD - x2, 3), y1, round(FIELD - x1, 3), y2)


def center_rect(cx, cy, w, h):
    return (round(cx - w / 2, 3), round(cy - h / 2, 3),
            round(cx + w / 2, 3), round(cy + h / 2, 3))


def fmt(x):
    s = ("%.3f" % x).rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def arr(vals):
    return "[" + ", ".join(fmt(v) for v in vals) + "]"


# ─────────────────────────────────────────────────────────────────────────────
# 布局装配 (红队 + 沿中轴镜像的蓝队)
# ─────────────────────────────────────────────────────────────────────────────
def build():
    lay = {}
    fs = S.FIELD_SIZE

    # -- 基础 --
    lay["ground_z"] = S.GROUND_Z
    lay["l1"] = dict(z=S.L1_HEIGHT, rect=center_rect(*S.L1_CENTER, S.L1_SIZE, S.L1_SIZE))
    lay["l2"] = dict(z=S.L2_HEIGHT, rect=center_rect(*S.L2_CENTER, S.L2_SIZE, S.L2_SIZE))

    # -- 地面共享区 / 柱子 --
    gs = S.GROUND_SHARED_SIZE
    lay["ground_shared_rect"] = center_rect(*S.GROUND_SHARED_CENTER, gs, gs)
    lay["mustika_pillar"] = dict(pos=list(S.MUSTIKA_PILLAR_POS), z=S.GROUND_Z,
                                 height=S.MUSTIKA_PILLAR_H, radius=S.MUSTIKA_PILLAR_D / 2)
    lay["core_pillar"] = dict(pos=list(S.CORE_PILLAR_POS), z=S.L2_HEIGHT,
                              height=S.CORE_PILLAR_H, radius=S.CORE_PILLAR_D / 2)

    # -- 红队区域 (尺寸 size 或 (w,h)) --
    def zone(cx, cy, size):
        if isinstance(size, (tuple, list)):
            w, h = size
        else:
            w = h = size
        return center_rect(cx, cy, w, h)

    red = {}
    red["storage"] = zone(*S.RED_STORAGE, S.STORAGE_SIZE)
    red["start_tr"] = zone(*S.RED_START_TR, S.START_BOX_SIZE)
    red["start_br"] = zone(*S.RED_START_BR, S.START_BOX_SIZE)
    red["transfer"] = zone(*S.RED_TRANSFER, S.TRANSFER_SIZE)
    red["retry_l1"] = zone(*S.RED_RETRY_L1, S.RETRY_L1_SIZE)
    red["ramp"] = (S.RED_RAMP["from"], S.RED_RAMP["to"])
    red["stairs_l1"] = S.RED_STAIRS_L1
    red["stairs_l2"] = S.RED_STAIRS_L2

    blue = {k: (mirror_rect(v) if isinstance(v, tuple) and len(v) == 4 else v)
            for k, v in red.items()}
    blue["ramp"] = ((mirror_x(red["ramp"][0][0]), red["ramp"][0][1]),
                    (mirror_x(red["ramp"][1][0]), red["ramp"][1][1]))
    blue["stairs_l1"] = dict(foot=(mirror_x(red["stairs_l1"]["foot"][0]),
                                   red["stairs_l1"]["foot"][1]),
                             steps=red["stairs_l1"]["steps"])
    blue["stairs_l2"] = dict(foot=(mirror_x(red["stairs_l2"]["foot"][0]),
                                   red["stairs_l2"]["foot"][1]),
                             steps=red["stairs_l2"]["steps"])
    lay["red"], lay["blue"] = red, blue

    # -- 建筑位 --
    l1_ours = [(x, y, "own") for x, y in S.SPOTS_L1_OURS] + \
              [(x, y, "shared") for x, y in S.SPOTS_L1_SHARED]
    l1_theirs = [(round(FIELD - x, 3), y, "shared" if z == "shared" else "own")
                 for x, y, z in l1_ours]
    lay["spots_l1_ours"] = l1_ours
    lay["spots_l1_theirs"] = l1_theirs
    lay["spots_l2"] = [list(p) + ["global"] for p in S.SPOTS_L2_GLOBAL]

    # -- 天空方块棋盘 --
    sky = []
    for r, c, col in S.sky_grid_positions():
        ox, oy = S.SKY_GRID_ORIGIN
        sky.append([round(ox + c * S.SKY_GRID_STEP, 3),
                    round(oy + r * S.SKY_GRID_STEP, 3), col])
    lay["sky"] = sky

    return lay


# ─────────────────────────────────────────────────────────────────────────────
# YAML 输出 (兼容现有 ROS2 参数 schema)
# ─────────────────────────────────────────────────────────────────────────────
def rect_corners(r):
    x1, y1, x2, y2 = r
    return [x1, y1, x2, y1, x2, y2, x1, y2]  # 西南起逆时针, 拍平


def gen_yaml(lay):
    red, blue = lay["red"], lay["blue"]
    ours_spots = lay["spots_l1_ours"]
    theirs_spots = lay["spots_l1_theirs"]
    l2_spots = lay["spots_l2"]
    all_l1 = ours_spots + theirs_spots

    def spot_rect(s, level):
        r = center_rect(s[0], s[1], S.SPOT_SIZE, S.SPOT_SIZE)
        if level == 2:
            r = (r[0], r[1] + S.L1_HEIGHT, r[2], r[3] + S.L1_HEIGHT)  # z 提示, 仅注释
        return r

    L = []
    A = L.append

    A("# ═══════════════════════════════════════════════════════════════════")
    A("# 场地几何配置 — ROBOCON 2027  (由 field_model_generator.py 自动生成)")
    A("# 生成时间: %s" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    A("# 唯一事实源: scripts/field_model_spec.py  — 不要手工编辑本文件!")
    A("#")
    A("# ⚠⚠ 警告: 位置类数值为 DRAFT 假设布局, 尚未与官方图纸核对, 严禁直接用于比赛!")
    A("#   OFFICIAL = 规则书 v1.1 文字规格; DRAFT = 团队假设; TODO = 待官方图册/实测替换")
    A("#   依据: Robocon_2027_Rulebook_v1-1.pdf (robocon.org.cn 2026-08-28)")
    A("# 坐标系: 原点西南角, X 东 Y 北 Z 上, 单位 m; 红(我方)在西半场")
    A("# ═══════════════════════════════════════════════════════════════════")
    A("/**:")
    A("  ros__parameters:")
    A("")
    A("    # ── 1. 场地基础 ──")
    A("    field:")
    A("      origin: [0.0, 0.0]")
    A("      size:   [%s, %s]     # %d×%d mm (OFFICIAL)" % (fmt(FIELD), fmt(FIELD), FIELD * 1000, FIELD * 1000))
    A("")
    A("    ground_level:")
    A("      z: %s" % fmt(lay["ground_z"]))
    A("")
    A("    # ── 2. 平台 (中心跨中轴为 DRAFT 假设) ──")
    A("    l1_platform:")
    A("      height: %s        # 600mm (OFFICIAL)" % fmt(lay["l1"]["z"]))
    r = lay["l1"]["rect"]
    A("      center: %s" % arr([(r[0] + r[2]) / 2, (r[1] + r[3]) / 2]))
    A("      half_size: %s" % arr([S.L1_SIZE / 2, S.L1_SIZE / 2]))
    A("      x_range: [%s, %s]   # DRAFT" % (fmt(r[0]), fmt(r[2])))
    A("      y_range: [%s, %s]   # DRAFT" % (fmt(r[1]), fmt(r[3])))
    A("")
    A("    l2_platform:")
    A("      height: %s        # 900mm (OFFICIAL)" % fmt(lay["l2"]["z"]))
    r = lay["l2"]["rect"]
    A("      center: %s" % arr([(r[0] + r[2]) / 2, (r[1] + r[3]) / 2]))
    A("      half_size: %s" % arr([S.L2_SIZE / 2, S.L2_SIZE / 2]))
    A("      x_range: [%s, %s]   # DRAFT" % (fmt(r[0]), fmt(r[2])))
    A("      y_range: [%s, %s]   # DRAFT" % (fmt(r[1]), fmt(r[3])))
    A("")
    A("    # ── 3. 转运/存储/共享区 (corner 拍平, 西南起逆时针) ──")
    A("    transfer_zone:")
    A("      ours:")
    A("        center: %s        # DRAFT" % arr([(red['transfer'][0] + red['transfer'][2]) / 2,
                                                  (red['transfer'][1] + red['transfer'][3]) / 2]))
    A("        corners: %s" % arr(rect_corners(red["transfer"])))
    A("      theirs:")
    A("        center: %s        # DRAFT" % arr([(blue['transfer'][0] + blue['transfer'][2]) / 2,
                                                  (blue['transfer'][1] + blue['transfer'][3]) / 2]))
    A("        corners: %s" % arr(rect_corners(blue["transfer"])))
    A("")
    A("    storage_zone:")
    for side, nm in (("ours", red), ("theirs", blue)):
        A("      %s:" % side)
        A("        center: %s    # DRAFT (1000×2000mm OFFICIAL)" % arr([(nm['storage'][0] + nm['storage'][2]) / 2,
                                                                         (nm['storage'][1] + nm['storage'][3]) / 2]))
        A("        corners: %s" % arr(rect_corners(nm["storage"])))
    A("")
    A("    ground_shared_zone:")
    A("      center: %s     # DRAFT (1200×1200mm OFFICIAL)" % arr(list(S.GROUND_SHARED_CENTER)))
    A("      half_size: [%s, %s]" % (fmt(S.GROUND_SHARED_SIZE / 2), fmt(S.GROUND_SHARED_SIZE / 2)))
    A("      mustika_surround:")
    A("        half_size: [0.5, 0.5]  # 1000×1000mm (OFFICIAL)")
    A("")
    A("    l1_shared_zone:")
    A("      description: \"L1 中轴两侧共享带 (红/蓝各 %s m 宽, DRAFT)\"" % fmt(S.L1_SHARED_BAND_W))
    A("      ours_band_x: [%s, %s]   # DRAFT" % (fmt(MID - S.L1_SHARED_BAND_W), fmt(MID)))
    A("      theirs_band_x: [%s, %s] # DRAFT" % (fmt(MID), fmt(MID + S.L1_SHARED_BAND_W)))
    A("")
    A("    l2_shared_zone:")
    A("      description: \"整个 L2 平台为共享区 (OFFICIAL)\"")
    A("      center: [5.5, 5.5]")
    A("      half_size: [1.5, 1.5]")
    A("")
    A("    # ── 4. 柱子 ──")
    A("    mustika_pillar:")
    A("      position: %s   # DRAFT (地面)" % arr(lay["mustika_pillar"]["pos"]))
    A("      height: %s      # 500mm (OFFICIAL)" % fmt(lay["mustika_pillar"]["height"]))
    A("      diameter: %s    # 270mm (OFFICIAL)" % fmt(lay["mustika_pillar"]["height"] * 0.54))
    A("      radius: %s" % fmt(lay["mustika_pillar"]["radius"]))
    A("")
    A("    core_pillar:")
    A("      position: %s   # DRAFT (L2 中央)" % arr(lay["core_pillar"]["pos"]))
    A("      height: %s      # 800mm (OFFICIAL)" % fmt(lay["core_pillar"]["height"]))
    A("      diameter: %s" % fmt(lay["core_pillar"]["height"] * 0.3375))
    A("      radius: %s" % fmt(lay["core_pillar"]["radius"]))
    A("")
    A("    # ── 5. 建筑位 (500×500mm; 坐标 DRAFT, 官方数量/排列见图纸!) ──")
    A("    building_spots:")
    A("      spot_size: %s" % fmt(S.SPOT_SIZE))
    A("      spot_half_size: %s" % fmt(S.SPOT_SIZE / 2))
    l1_all = []
    for x, y, z in all_l1:
        l1_all += [x, y]
    l2_all = []
    for x, y, z in l2_spots:
        l2_all += [x, y]
    A("      # 拍平 [x1,y1, x2,y2, ...]  — L1 我方(6)+共享带(4)+对方镜像(10)")
    A("      l1: %s" % arr(l1_all))
    A("      # L2 全局共享 3×3 缺中心(8 个), 不按队伍复制")
    A("      l2: %s" % arr(l2_all))
    A("")
    A("    # ── 6. 围栏/中心分隔 (DRAFT) ──")
    A("    fences:")
    A("      description: \"场地外圈\"")
    A("      segments: [0.0, 0.0, %s, 0.0,  %s, 0.0, %s, %s,  %s, %s, 0.0, %s,  0.0, %s, 0.0, 0.0]"
      % (fmt(FIELD), fmt(FIELD), fmt(FIELD), fmt(FIELD), fmt(FIELD), fmt(FIELD), fmt(FIELD), fmt(FIELD)))
    A("    center_divider:")
    A("      description: \"中轴隔墙, 中央留开口 y∈[%s,%s] (DRAFT)\"" % (fmt(S.DIVIDER_OPEN_Y[0]), fmt(S.DIVIDER_OPEN_Y[1])))
    A("      segments: [5.5, 0.0, 5.5, %s,  5.5, %s, 5.5, %s]"
      % (fmt(S.DIVIDER_OPEN_Y[0]), fmt(S.DIVIDER_OPEN_Y[1]), fmt(FIELD)))
    A("")
    A("    # ── 7. 坡道/楼梯 (位置 DRAFT) ──")
    A("    ramps:")
    for side, nm in (("ours", red), ("theirs", blue)):
        A("      %s: [%s, %s, %s, %s, 0.0, %s]   # from_xy→to_xy 坡道 (DRAFT)"
          % (side, fmt(nm["ramp"][0][0]), fmt(nm["ramp"][0][1]),
             fmt(nm["ramp"][1][0]), fmt(nm["ramp"][1][1]), fmt(S.L1_HEIGHT)))
    A("    stairs:")
    for level, key in (("l1", "stairs_l1"), ("l2", "stairs_l2")):
        for side, nm in (("ours", red), ("theirs", blue)):
            A("      %s.%s.foot: %s   # 台阶脚 (DRAFT), 级数=%d"
              % (level, side, arr(list(nm[key]["foot"])), nm[key]["steps"]))
    A("")
    A("    # ── 8. v1.6: 启动区/重试区 (位置 DRAFT) ──")
    A("    start_zones:")
    A("      box_size: %s       # 700×700mm (OFFICIAL)" % fmt(S.START_BOX_SIZE))
    for side, nm in (("ours", red), ("theirs", blue)):
        A("      %s:" % side)
        for robot, k in (("tr", "start_tr"), ("br", "start_br")):
            A("        %s: %s" % (robot, arr([(nm[k][0] + nm[k][2]) / 2, (nm[k][1] + nm[k][3]) / 2])))
    A("    retry_zones:")
    A("      description: \"TR 回地面启动区; BR 可回地面启动区或 L1 重试区 (规则 5.2.1)\"")
    A("      l1:")
    for side, nm in (("ours", red), ("theirs", blue)):
        A("        %s: %s" % (side, arr([(nm['retry_l1'][0] + nm['retry_l1'][2]) / 2,
                                         (nm['retry_l1'][1] + nm['retry_l1'][3]) / 2])))
    A("")
    A("    # ── 9. v1.6: 天空方块初始棋盘 (规则 4.1.4; 原点 DRAFT) ──")
    A("    sky_block_initial:")
    A("      center: [5.5, 1.25]        # CONFIRMED-SW 棋盘中心 (= 内框中心)")
    A("      outer_size: [1.2, 1.2]     # 外框 1200 (草图52/34)")
    A("      outer_x_range: [4.9, 6.1]")
    A("      outer_y_range: [0.65, 1.85]")
    A("      inner_size: [1.0, 1.0]     # 内框 1000 = 25 格所在区 (草图29)")
    A("      inner_x_range: [5.0, 6.0]")
    A("      inner_y_range: [0.75, 1.75]")
    A("      border: 0.10               # 外框与内框之间的边宽")
    A("      grid_origin: %s    # 5×5 西南格中心 = 内框西南角 + 半格" % arr(list(S.SKY_GRID_ORIGIN)))
    A("      grid_step: %s               # 单格 200mm = 内框 1000 / 5" % fmt(S.SKY_GRID_STEP))
    A("      count: 12           # (OFFICIAL)")
    sky_flat = []
    for x, y, col in lay["sky"]:
        sky_flat += [x, y, col]
    A("      # 拍平 [x,y,color×12], color: 1=红朝上 2=蓝朝上; 中心格空置")
    A("      positions: %s" % arr(sky_flat))
    A("")
    for ln in S.ZONE_COLOR_HEADER:
        A(ln)
    A("    zone_colors:")
    for key, (r, g, b), cmt in S.ZONE_COLORS:
        A("      %-28s [%3d, %3d, %3d]   # %s" % (key + ":", r, g, b, cmt))
    A("")
    A("    # ── 11. 兼容层: 扁平数组 (供现有节点; L1 全部 20 位, Z 基准 = L1) ──")
    A("    #     注意: 旧协议每帧最多 16 建筑位; 若总数超限需按优先级取舍")
    bx = [p[0] for p in all_l1]
    by = [p[1] for p in all_l1]
    A("    building_spot_x: %s" % arr(bx))
    A("    building_spot_y: %s" % arr(by))
    A("    platform_z: %s        # 建筑位高度基准 = L1 表面 (L2 需层级化支持)" % fmt(S.L1_HEIGHT))
    A("    spot_half_size: %s" % fmt(S.SPOT_SIZE / 2))
    A("    column_z_min: 0.0")
    A("    column_z_max: 1.5")
    A("    min_points_threshold: 5")
    A("    top_surface_tolerance: 0.03")
    A("    top_surface_min_points: 3")
    A("    empty_max_height: 0.05")
    A("    one_earth_min: 0.20")
    A("    one_earth_max: 0.45")
    A("    two_earth_min: 0.55")
    A("    two_earth_max: 0.80")
    A("    complete_tower_min: 0.80")
    A("    complete_tower_max: 1.10")
    A("")
    return "\n".join(L)


# ─────────────────────────────────────────────────────────────────────────────
# 校验 + 报告
# ─────────────────────────────────────────────────────────────────────────────
def validate(lay):
    notes, warns = [], []

    def inside(rect, x, y):
        return rect[0] - 1e-6 <= x <= rect[2] + 1e-6 and rect[1] - 1e-6 <= y <= rect[3] + 1e-6

    l1r, l2r = lay["l1"]["rect"], lay["l2"]["rect"]
    # 平台包含关系
    if not (inside((0, 0, FIELD, FIELD), l1r[0], l1r[1]) and
            inside((0, 0, FIELD, FIELD), l1r[2], l1r[3])):
        warns.append("L1 平台超出场地边界!")
    if not (inside(l1r, l2r[0], l2r[1]) and inside(l1r, l2r[2], l2r[3])):
        warns.append("L2 平台未完全落在 L1 上!")
    else:
        notes.append("L1/L2 平台包含关系 OK")
    # 建筑位归属
    n_own = n_shared = n_own_b = n_shared_b = 0
    for x, y, z in lay["spots_l1_ours"]:
        ok = inside(l1r, x, y)
        (notes if ok else warns).append("L1 我方位 (%.2f,%.2f)%s" % (x, y, "" if ok else " 不在 L1 内!"))
        if z == "shared":
            n_shared += 1
        else:
            n_own += 1
    for x, y, z in lay["spots_l1_theirs"]:
        ok = inside(l1r, x, y)
        if not ok:
            warns.append("L1 对方位 (%.2f,%.2f) 不在 L1 内!" % (x, y))
        if z == "shared":
            n_shared_b += 1
        else:
            n_own_b += 1
    for x, y, z in lay["spots_l2"]:
        ok = inside(l2r, x, y)
        (notes if ok else warns).append("L2 位 (%.2f,%.2f)%s" % (x, y, "" if ok else " 不在 L2 内!"))
    notes.append("建筑位: L1 我方 %d + 共享 %d, 对方镜像 %d+%d; L2 全局 %d"
                 % (n_own, n_shared, n_own_b, n_shared_b, len(lay["spots_l2"])))
    # 柱子与 L2 位不重叠
    px, py = lay["core_pillar"]["pos"]
    for x, y, z in lay["spots_l2"]:
        if abs(x - px) < S.SPOT_SIZE / 2 + 0.05 and abs(y - py) < S.SPOT_SIZE / 2 + 0.05:
            warns.append("L2 位 (%.2f,%.2f) 与核心支柱冲突!" % (x, y))
    # 天空棋盘 12 块
    if len(lay["sky"]) != 12:
        warns.append("天空方块数量应为 12, 实得 %d" % len(lay["sky"]))
    else:
        notes.append("天空棋盘 12 块, 中心空置, 颜色沿中心列镜像 — 符合规则 4.1.4 逻辑")
    # 区域内互不重叠粗检
    zones = list(red_rects(lay).values())
    for i in range(len(zones)):
        for j in range(i + 1, len(zones)):
            a, b = zones[i], zones[j]
            if a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]:
                warns.append("红方地面区域重叠: %s ↔ %s" % (a, b))
    return notes, warns


def red_rects(lay):
    red = lay["red"]
    return {
        "存储区": red["storage"],
        "启动箱TR": red["start_tr"],
        "启动箱BR": red["start_br"],
        "转运区": red["transfer"],
        "L1重试": red["retry_l1"],
    }


def write_report(lay, notes, warns, apply):
    lines = []
    A = lines.append
    A("ROBOCON 2027 场地模型 — 校验报告 (%s)" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    A("=" * 70)
    A("")
    A("[校验结果]")
    for n in notes:
        A("  · %s" % n)
    A("")
    if warns:
        A("[警告 %d]" % len(warns))
        for w in warns:
            A("  ⚠ %s" % w)
    else:
        A("[警告] 无")
    A("")
    A("=" * 70)
    A("[待补数值清单 — 与官方图纸(规则书 v1.1 Figure 1/2)核对后修改 spec]")
    A("  D1 中轴隔墙开口区间 y = %s..%s" % (fmt(S.DIVIDER_OPEN_Y[0]), fmt(S.DIVIDER_OPEN_Y[1])))
    A("  D2 L1 平台位置(当前: 中心跨中轴 %s×%s)" % (fmt(S.L1_SIZE), fmt(S.L1_SIZE)))
    A("  D3 L2 平台位置(当前: 与 L1 同心 3×3)")
    A("  D4 每队启动箱数量与位置 (当前假设每队 2 个, 红方西南 apron)")
    A("  D5 存储区位置/朝向 (当前 红方 %s, 1000×2000mm)" % (S.RED_STORAGE,))
    A("  D6 转运区位置 (当前 红方 L1 南缘上方, 1000×1000mm)")
    A("  D7 L1 重试区位置 (红方 L1 北侧)")
    A("  D8 L1/L2 共享区带宽度 (当前 L1 共享带 %s m)" % fmt(S.L1_SHARED_BAND_W))
    A("  D9 建筑位数量与排列 — L1 我方 %d/共享 %d, L2 %d 全为 DRAFT!" % (
        len(S.SPOTS_L1_OURS), len(S.SPOTS_L1_SHARED), len(S.SPOTS_L2_GLOBAL)))
    A("  D10 穆斯蒂卡柱/核心支柱精确位置 (当前都居中)")
    A("  D11 天空棋盘原点 (当前 %s, 步长 %s; 与图 3/4 核对)" % (S.SKY_GRID_ORIGIN, S.SKY_GRID_STEP))
    A("  D12 红方坡道(3.5m)与 L1/L2 楼梯的位置/级数")
    A("  D13 穆斯蒂卡实物核对: 规则书=金属金 futsal 球(RGB 218-165-32), 与 v1.6 排球假设冲突!")
    A("")
    A("修改方式: 编辑 scripts/field_model_spec.py 对应常量 → 重新运行生成器。")
    txt = "\n".join(lines)
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "field_model_report.txt"), "w", encoding="utf-8") as f:
        f.write(txt)
    return txt


# ─────────────────────────────────────────────────────────────────────────────
# SVG 俯视图
# ─────────────────────────────────────────────────────────────────────────────
def rgb(c):
    return "rgb(%d,%d,%d)" % c


def gen_svg(lay):
    SCL = 60                      # px per meter
    PAD = 50
    W = FIELD * SCL + PAD * 2 + 240
    H = FIELD * SCL + PAD * 2 + 40

    def X(x):
        return PAD + x * SCL

    def Y(y):
        return H - PAD - y * SCL      # 翻转使北在上

    def rect_svg(r, fill, op=1.0, stroke=None, sw=1, dash=None, label=None, lc=None, ldx=0.05, ldy=-0.08):
        x1, y1, x2, y2 = r
        s = '<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s" fill-opacity="%s"' % (
            X(x1), Y(y2), (x2 - x1) * SCL, (y2 - y1) * SCL, fill, op)
        if stroke:
            s += ' stroke="%s" stroke-width="%s"' % (stroke, sw)
            if dash:
                s += ' stroke-dasharray="%s"' % dash
        s += '/>'
        if label:
            lx = X(x1 + (x2 - x1) * (0.5 + ldx))
            ly = Y(y1 + (y2 - y1) * (0.5 + ldy)) - 4
            s += '<text x="%.1f" y="%.1f" font-size="11" fill="%s" text-anchor="middle">%s</text>' % (
                lx, ly, lc or "#222", label)
        return s

    def circle_svg(cx, cy, rad, fill, label=None):
        s = '<circle cx="%.1f" cy="%.1f" r="%.1f" fill="%s" stroke="#000" stroke-width="1"/>' % (
            X(cx), Y(cy), rad * SCL, fill)
        if label:
            s += '<text x="%.1f" y="%.1f" font-size="10" text-anchor="middle" fill="#000">%s</text>' % (
                X(cx), Y(cy) + 3, label)
        return s

    P = []
    A = P.append
    A('<?xml version="1.0" encoding="UTF-8"?>')
    A('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">' % (W, H, W, H))
    A('<rect width="%d" height="%d" fill="#fbfbf8"/>' % (W, H))
    C = S.COLORS
    red, blue = lay["red"], lay["blue"]

    # 地面
    A(rect_svg((0, 0, MID, FIELD), rgb(C["ground_red"]), label="红方地面", lc="#a00", ldy=0.06))
    A(rect_svg((MID, 0, FIELD, FIELD), rgb(C["ground_blue"]), label="蓝方地面", lc="#05a", ldy=0.06))
    # 地面共享区(奶油色) + 棋盘
    A(rect_svg(lay["ground_shared_rect"], rgb(C["shared_cream"]), op=0.85,
               stroke="#888", sw=1.2, label="地面共享区", ldy=0.4))
    for x, y, col in lay["sky"]:
        hw = 0.14
        A(rect_svg((x - hw, y - hw, x + hw, y + hw),
                   rgb(C["sky_red"] if col == 1 else C["sky_blue"]),
                   label="R" if col == 1 else "B", lc="#fff"))
    # 平台
    A(rect_svg(lay["l1"]["rect"], rgb(C["l1_red"]), op=0.55, stroke="#b86",
               label="L1 (600mm)", ldy=0.05, lc="#a52"))
    A(rect_svg(lay["l2"]["rect"], rgb(C["l2_gray"]), op=0.9, stroke="#555",
               label="L2 全共享 (900mm)", ldy=0.05))
    # L1 共享带
    for sgn, fill in (("ours", rgb(C["shared_cream"])), ("theirs", rgb(C["shared_cream"]))):
        x0 = MID - S.L1_SHARED_BAND_W if sgn == "ours" else MID
        A(rect_svg((x0, lay["l1"]["rect"][1], x0 + S.L1_SHARED_BAND_W, lay["l1"]["rect"][3]),
                   fill, op=0.4, stroke="#666", sw=1, dash="4,3",
                   label="L1共享" if sgn == "ours" else None, ldy=0.06))

    # 建筑位
    for x, y, z in lay["spots_l1_ours"]:
        A(rect_svg(center_rect(x, y, S.SPOT_SIZE, S.SPOT_SIZE), rgb(C["spot_green"]),
                   stroke="#040", label="位", lc="#fff"))
    for x, y, z in lay["spots_l1_theirs"]:
        A(rect_svg(center_rect(x, y, S.SPOT_SIZE, S.SPOT_SIZE), rgb(C["spot_green"]),
                   stroke="#040", op=0.8))
    for x, y, z in lay["spots_l2"]:
        A(rect_svg(center_rect(x, y, S.SPOT_SIZE, S.SPOT_SIZE), rgb(C["spot_green"]),
                   stroke="#040", label="位", lc="#fff"))

    # 红队区域
    A(rect_svg(red["storage"], rgb(C["ground_red"]), stroke="#b00", label="存储", lc="#a00"))
    A(rect_svg(red["start_tr"], rgb(C["start_red"]), stroke="#000", label="启动A(TR)", lc="#fff"))
    A(rect_svg(red["start_br"], rgb(C["start_red"]), stroke="#000", label="启动B(BR)", lc="#fff"))
    A(rect_svg(red["transfer"], rgb(C["transfer_red"]), stroke="#000", label="转运", lc="#000"))
    A(rect_svg(red["retry_l1"], rgb(C["start_red"]), stroke="#000", label="L1重试", lc="#fff"))
    # 蓝队区域
    A(rect_svg(blue["storage"], rgb(C["ground_blue"]), stroke="#00b", label="存储", lc="#05a"))
    A(rect_svg(blue["start_tr"], rgb(C["start_blue"]), stroke="#000", label="启动A(TR)", lc="#fff"))
    A(rect_svg(blue["start_br"], rgb(C["start_blue"]), stroke="#000", label="启动B(BR)", lc="#fff"))
    A(rect_svg(blue["transfer"], rgb(C["transfer_blue"]), stroke="#000", label="转运", lc="#000"))
    A(rect_svg(blue["retry_l1"], rgb(C["start_blue"]), stroke="#000", label="L1重试", lc="#fff"))

    # 坡道/楼梯(示意)
    for side, nm in (("R", red), ("B", blue)):
        (x0, y0), (x1, y1) = nm["ramp"]
        A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#a60" stroke-width="14" stroke-opacity="0.45"/>'
          % (X(x0), Y(y0), X(x1), Y(y1)))
        A('<text x="%.1f" y="%.1f" font-size="10" fill="#a60">坡道</text>' % (X(x0 + 0.12), Y((y0 + y1) / 2)))
    # 柱子
    A(circle_svg(*S.MUSTIKA_PILLAR_POS, S.MUSTIKA_PILLAR_D / 2, rgb(C["pillar_brown"]), label="穆柱"))
    A(circle_svg(*S.CORE_PILLAR_POS, S.CORE_PILLAR_D / 2, rgb(C["pillar_brown"]), label="核心柱"))

    # 分隔墙
    y0, y1 = S.DIVIDER_OPEN_Y
    for (ya, yb) in ((0, y0), (y1, FIELD)):
        A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#555" stroke-width="6"/>'
          % (X(MID), Y(ya), X(MID), Y(yb)))
    # 外圈
    A('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="none" stroke="#444" stroke-width="5"/>'
      % (X(0), Y(FIELD), FIELD * SCL, FIELD * SCL))

    # 图例
    lx = X(FIELD) + 30
    ly = Y(FIELD) + 30
    A('<text x="%.1f" y="%.1f" font-size="14" font-weight="bold">图例 / 说明</text>' % (lx, ly))
    items = [
        ("红色填充", rgb(C["l1_red"]), "红方区域(L1 淡色)"),
        ("蓝色填充", rgb(C["l1_blue"]), "蓝方区域(L1 淡色)"),
        (rgb(C["spot_green"]), rgb(C["spot_green"]), "建筑位 500×500 (DRAFT 排列)"),
        ("#a60", "#a60", "坡道(3.5m, DRAFT 位置)"),
        ("#666", "#666", "L1 共享带(DRAFT)"),
    ]
    yy = ly + 25
    for swatch, fill, txt in items:
        A('<rect x="%.1f" y="%.1f" width="16" height="10" fill="%s" stroke="#333"/>' % (lx, yy, swatch))
        A('<text x="%.1f" y="%.1f" font-size="11" fill="#000">%s</text>' % (lx + 22, yy + 9, txt))
        yy += 18
    A('<text x="%.1f" y="%.1f" font-size="11" fill="#c00" font-weight="bold">⚠ 本图为 DRAFT 假设布局 — 未与官方图纸核对</text>'
      % (lx, yy + 12))
    A('<text x="%.1f" y="%.1f" font-size="11" fill="#555">用 python field_model_generator.py --apply 重新生成</text>'
      % (lx, yy + 30))
    A("</svg>")
    svg = "\n".join(P)
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "field_layout_preview.svg"), "w", encoding="utf-8") as f:
        f.write(svg)
    return os.path.join(OUT_DIR, "field_layout_preview.svg")


# ─────────────────────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    apply = "--apply" in sys.argv
    lay = build()
    notes, warns = validate(lay)
    yaml_txt = gen_yaml(lay)

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "field_geometry.generated.yaml"), "w", encoding="utf-8") as f:
        f.write(yaml_txt + "\n")

    svg_path = gen_svg(lay)
    report = write_report(lay, notes, warns, apply)

    if apply:
        if os.path.exists(CONFIG_PATH):
            bak = os.path.join(OUT_DIR, "field_geometry.orig.yaml")
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                orig = f.read()
            with open(bak, "w", encoding="utf-8") as f:
                f.write(orig)
            print("备份原配置 -> %s" % bak)
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            f.write(yaml_txt + "\n")
        print("已写入 config/field_geometry.yaml")

    print(report)
    print("-" * 70)
    print("YAML 预览 : %s" % os.path.join(OUT_DIR, "field_geometry.generated.yaml"))
    print("SVG 预览  : %s" % svg_path)
    print("报告      : %s" % os.path.join(OUT_DIR, "field_model_report.txt"))
    print("使用 --apply 才会覆盖 config/field_geometry.yaml")
    return 0 if not warns else 1


if __name__ == "__main__":
    sys.exit(main())
