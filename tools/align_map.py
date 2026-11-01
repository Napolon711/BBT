#!/usr/bin/env python3
"""把"导航建图坐标系"对齐到"点表场地坐标系"——BBT 版。

用法：
  # 1) 写一个对应表（**洞中心**：隧道名 → 建图系里实测的坐标），至少 2 个，
  #    建议 3 个以上分散的洞（建图后在地图上量洞中心在哪）
  cat > pairs.csv <<'EOF'
  id,map_x,map_y
  own_road_tunnel,3.42,1.05
  enemy_road_tunnel,-2.10,4.77
  enemy_near_tunnel,1.20,-3.31
  EOF

  # 2) 解算（场地系坐标自动从 decision.yaml 的 tunnels[].center 取）
  python3 tools/align_map.py pairs.csv

  # 3) 结果直接写进 decision.yaml 的 field_to_map 段
  python3 tools/align_map.py pairs.csv --write

说明：
- 只解 **旋转 + 平移**（dx, dy, dyaw_deg），不解缩放：两边都是米制，建图不该有
  尺度差；若确实有尺度漂移，加 --scale 会同时解出 s（写进 yaml 前需要扩展字段）。
- 2 个点 → 唯一解；≥3 个点 → 最小二乘，并给出每个点的残差（超 5 cm 就查哪个点标错）。
- 对齐后 BBT 加载配置时会把 points / tunnels 的全部坐标一起搬到建图系，
  发给导航的 /goal_pose 与收到的 /odom 就在同一系里了。
- 仿真必须提供与点表一致的场地坐标；实车标定后必须 enabled: true。
"""
import argparse, csv, io, math, sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
DEFAULT_CONFIG = TOOLS.parent / "src/bbt_decide/config/decision.yaml"


def load_field(path):
    """用**洞中心**当对齐基准：场地系坐标取自 decision.yaml 的 tunnels[].center。"""
    import yaml
    doc = yaml.safe_load(io.open(path, encoding="utf-8"))
    out = {}
    for t in doc.get("tunnels", []) or []:
        c = t.get("center")
        if c and len(c) == 2:
            out[t.get("name", "?")] = (float(c[0]), float(c[1]))
    return out


def solve(pairs, use_scale=False):
    """最小二乘解 刚体变换（θ, s, tx, ty），把场地系点搬到建图系。"""
    n = len(pairs)
    px = sum(p[0] for p, _ in pairs) / n
    py = sum(p[1] for p, _ in pairs) / n
    qx = sum(q[0] for _, q in pairs) / n
    qy = sum(q[1] for _, q in pairs) / n
    num = den = 0.0
    sxx = 0.0
    for (p, q) in pairs:
        ax, ay = p[0] - px, p[1] - py
        bx, by = q[0] - qx, q[1] - qy
        num += ax * by - ay * bx          # 叉积：解 θ
        den += ax * bx + ay * by          # 点积
        sxx += ax * ax + ay * ay
    theta = math.atan2(num, den)
    s = 1.0
    if use_scale:
        s = math.hypot(num, den) / sxx if sxx > 0 else 1.0
    c, sn = math.cos(theta), math.sin(theta)
    tx = qx - s * (c * px - sn * py)
    ty = qy - s * (sn * px + c * py)
    return theta, s, tx, ty


def residuals(pairs, theta, s, tx, ty):
    """每对点的对齐残差（m），超 5 cm 大概率是标错了。"""
    c, sn = math.cos(theta), math.sin(theta)
    out = []
    for (p, q) in pairs:
        mx = s * (c * p[0] - sn * p[1]) + tx
        my = s * (sn * p[0] + c * p[1]) + ty
        out.append(math.hypot(mx - q[0], my - q[1]))
    return out


def write_yaml(cfg_path, dx, dy, dyaw):
    """把 field_to_map 写进 decision.yaml（只改那一小块，不碰别的）。"""
    import re
    s = io.open(cfg_path, encoding="utf-8").read()
    block = (f"field_to_map:\n  enabled: true\n  dx: {dx:.4f}\n"
             f"  dy: {dy:.4f}\n  dyaw_deg: {dyaw:.4f}\n")
    new, n = re.subn(r"field_to_map:\n(?:  .*\n)+", block, s, count=1)
    if n != 1:
        sys.exit("没找到 field_to_map 块（yaml 里没有这一段的话手工加上再跑）")
    io.open(cfg_path, "w", encoding="utf-8").write(new)
    print(f"已写入 {cfg_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pairs", help="对应表 csv：id,map_x,map_y（id = 洞名）")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG), help="decision.yaml 路径")
    ap.add_argument("--write", action="store_true", help="把解算结果写进 yaml")
    ap.add_argument("--scale", action="store_true", help="同时解缩放（诊断用，不写进 yaml）")
    args = ap.parse_args()

    field = load_field(args.config)
    pairs = []
    with io.open(args.pairs, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            name = (row.get("id") or "").strip()
            if name not in field:
                sys.exit(f"洞 {name} 不在 {args.config} 的 tunnels[].center 里（没标 center？）")
            pairs.append((field[name], (float(row["map_x"]), float(row["map_y"]))))
    if len(pairs) < 2:
        sys.exit("至少要 2 对点才能解旋转 + 平移")

    theta, s, tx, ty = solve(pairs, use_scale=args.scale)
    dyaw = math.degrees(theta)
    print(f"dx={tx:.4f} m  dy={ty:.4f} m  dyaw={dyaw:.4f}°")
    if args.scale:
        print(f"scale={s:.5f}（≠1 说明建图有尺度漂移，先查建图再谈对齐）")

    res = residuals(pairs, theta, s, tx, ty)
    names = [(row.get("id") or "?").strip() for row in csv.DictReader(io.open(args.pairs, encoding="utf-8"))]
    for name, r in zip(names, res):
        flag = "  ← 超 5 cm，查这个点" if r > 0.05 else ""
        print(f"  残差 {name}: {r * 100:.1f} cm{flag}")

    if args.write:
        if args.scale and abs(s - 1.0) > 0.005:
            sys.exit("带缩放的结果不写 yaml（field_to_map 不支持 scale 字段）")
        write_yaml(args.config, tx, ty, dyaw)
    else:
        print("(加 --write 写进 yaml)")


if __name__ == "__main__":
    main()
