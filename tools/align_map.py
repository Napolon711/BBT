#!/usr/bin/env python3
"""把"导航建图坐标系"对齐到"点表场地坐标系"——BBT 版。

用法：
  # 1) 写对应表：洞名从 tunnels[].center 取场地坐标；也可填 field_x,field_y。
  #    至少 2 个分散锚点；已知 map 朝向时可用单点并给 --dyaw-deg。
  cat > pairs.csv <<'EOF'
  id,field_x,field_y,map_x,map_y
  own_road_tunnel,,,3.42,1.05
  enemy_road_tunnel,,,-2.10,4.77
  enemy_near_tunnel,,,1.20,-3.31
  EOF

  # 2) 解算（场地系坐标自动从 decision.yaml 的 tunnels[].center 取）
  python3 tools/align_map.py pairs.csv

  # 3) 红方写入 field_to_map；蓝方加 --team blue 写入 blue_field_to_map
  python3 tools/align_map.py pairs.csv --write

说明：
- 只解 **旋转 + 平移**（dx, dy, dyaw_deg），不解缩放：两边都是米制，建图不该有
  尺度差；若确实有尺度漂移，加 --scale 会同时解出 s（写进 yaml 前需要扩展字段）。
- 2 个分散点 → 唯一解；≥3 个点 → 最小二乘；单点只有在已知 map 朝向时可解平移。
- 正式场地 --team blue 会先将场地点旋转为 (28-x, 15-y)，再解蓝方 map 变换；
  自定义地图 geometry_reference: none 不换边，与决策运行时一致。
- 对齐后 BBT 加载配置时会把 points / tunnels 的全部坐标一起搬到建图系，
  发给导航的 /goal_pose 与收到的 /odom 就在同一系里了。
- 仿真必须提供与点表一致的场地坐标；实车标定后必须 enabled: true。
"""
import argparse, csv, io, math, sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
DEFAULT_CONFIG = TOOLS.parent / "src/bbt_decide/config/decision.yaml"


def team_field_point(point, team, reference="red"):
    """把配置参考阵营的场地坐标换成指定阵营的场地坐标。"""
    if reference == "none" or team == reference:
        return point
    return (28.0 - point[0], 15.0 - point[1])


def load_field(path):
    """用**洞中心**当对齐基准：场地系坐标取自 decision.yaml 的 tunnels[].center。"""
    import yaml
    with io.open(path, encoding="utf-8") as stream:
        doc = yaml.safe_load(stream)
    out = {}
    for t in doc.get("tunnels", []) or []:
        c = t.get("center")
        if c and len(c) == 2:
            out[t.get("name", "?")] = (float(c[0]), float(c[1]))
    return out


def solve(pairs, use_scale=False, fixed_yaw_deg=None):
    """最小二乘解 刚体变换（θ, s, tx, ty），把场地系点搬到建图系。"""
    n = len(pairs)
    if n < 2 and fixed_yaw_deg is None:
        raise ValueError("单个标定点必须同时提供已知 map 朝向 --dyaw-deg")
    if fixed_yaw_deg is not None and not math.isfinite(fixed_yaw_deg):
        raise ValueError("--dyaw-deg 必须是有限数字")
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
    if n >= 2 and fixed_yaw_deg is None and sxx <= 1e-12:
        raise ValueError("标定点必须分布在不同位置，才能解旋转")
    theta = (math.radians(fixed_yaw_deg) if fixed_yaw_deg is not None
             else math.atan2(num, den))
    s = 1.0
    if use_scale:
        if sxx <= 1e-12:
            raise ValueError("标定点没有足够的空间分布，无法诊断尺度")
        if fixed_yaw_deg is None:
            s = math.hypot(num, den) / sxx
        else:
            c, sn = math.cos(theta), math.sin(theta)
            dot = sum((p[0] - px) * (c * (q[0] - qx) + sn * (q[1] - qy)) +
                      (p[1] - py) * (-sn * (q[0] - qx) + c * (q[1] - qy))
                      for p, q in pairs)
            s = dot / sxx
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


def write_yaml(cfg_path, dx, dy, dyaw, key="field_to_map"):
    """把指定阵营的场地→map 变换写进 decision.yaml。"""
    import yaml
    with io.open(cfg_path, encoding="utf-8") as stream:
        s = stream.read()
    block = (f"{key}:\n  enabled: true\n  dx: {dx:.4f}\n"
             f"  dy: {dy:.4f}\n  dyaw_deg: {dyaw:.4f}\n")
    root = yaml.compose(s)
    if not isinstance(root, yaml.MappingNode):
        sys.exit("配置必须是 YAML mapping")
    entries = [(k, v) for k, v in root.value if k.value == key]
    if len(entries) > 1:
        sys.exit(f"配置包含重复的 {key} 块，请先合并")
    if entries:
        k, v = entries[0]
        if v.start_mark.index < k.end_mark.index:
            sys.exit(f"{key} 使用 YAML alias，请先展开为独立标定块")
        end = v.end_mark.index
        if isinstance(v, yaml.MappingNode) and not v.flow_style and v.value:
            # block mapping 的 end_mark 包含随后注释；标定块只替换到最后一个值。
            end = v.value[-1][1].end_mark.index
        new = s[:k.start_mark.index] + block + s[end:]
    elif key == "blue_field_to_map":
        # 插在 root mapping 末尾、文档结束标记 (...) 之前。
        end = root.end_mark.index
        new = s[:end].rstrip("\n") + "\n" + block + s[end:]
    else:
        sys.exit("没找到 field_to_map 块（yaml 里没有这一段的话手工加上再跑）")
    if not new.endswith("\n"):
        new += "\n"
    # 例如被其他块引用的 anchor 不能因替换而留下悬空 alias；失败时保留原文件。
    yaml.safe_load(new)
    with io.open(cfg_path, "w", encoding="utf-8") as stream:
        stream.write(new)
    print(f"已写入 {cfg_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pairs", help="对应表 csv：id,map_x,map_y（id = 洞名）")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG), help="decision.yaml 路径")
    ap.add_argument("--write", action="store_true", help="把解算结果写进 yaml")
    ap.add_argument("--scale", action="store_true", help="同时解缩放（诊断用，不写进 yaml）")
    ap.add_argument("--team", choices=("red", "blue"), default="red",
                    help="标定哪一方的 map；blue 会先将场地点按 (28-x,15-y) 换边")
    ap.add_argument("--dyaw-deg", type=float,
                    help="已知 map 朝向（度）；允许只用一个原点标定平移")
    args = ap.parse_args()

    field = load_field(args.config)
    import yaml
    with io.open(args.config, encoding="utf-8") as stream:
        reference = (yaml.safe_load(stream) or {}).get("geometry_reference", "none")
    pairs = []
    names = []
    with io.open(args.pairs, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            name = (row.get("id") or "").strip()
            if row.get("field_x", "").strip() and row.get("field_y", "").strip():
                p = (float(row["field_x"]), float(row["field_y"]))
            elif name in field:
                p = field[name]
            else:
                sys.exit(f"标定点 {name or '(无 id)'} 不在洞表中；非洞锚点请填写 field_x,field_y")
            q = (float(row["map_x"]), float(row["map_y"]))
            if not all(math.isfinite(v) for v in (*p, *q)):
                sys.exit(f"标定点 {name or '(无 id)'} 坐标必须是有限数字")
            p = team_field_point(p, args.team, reference)
            pairs.append((p, q))
            names.append(name or f"anchor_{len(names)+1}")
    if len(pairs) < 1:
        sys.exit("至少要 1 对标定点")

    try:
        theta, s, tx, ty = solve(pairs, use_scale=args.scale,
                                 fixed_yaw_deg=args.dyaw_deg)
    except ValueError as e:
        sys.exit(str(e))
    dyaw = math.degrees(theta)
    print(f"dx={tx:.4f} m  dy={ty:.4f} m  dyaw={dyaw:.4f}°")
    if args.scale:
        print(f"scale={s:.5f}（≠1 说明建图有尺度漂移，先查建图再谈对齐）")

    res = residuals(pairs, theta, s, tx, ty)
    for name, r in zip(names, res):
        flag = "  ← 超 5 cm，查这个点" if r > 0.05 else ""
        print(f"  残差 {name}: {r * 100:.1f} cm{flag}")

    if args.write:
        if args.scale and abs(s - 1.0) > 0.005:
            sys.exit("带缩放的结果不写 yaml（field_to_map 不支持 scale 字段）")
        write_yaml(args.config, tx, ty, dyaw,
                   "field_to_map" if args.team == "red" else "blue_field_to_map")
    else:
        print("(加 --write 写进 yaml)")


if __name__ == "__main__":
    main()
