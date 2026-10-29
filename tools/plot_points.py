#!/usr/bin/env python3
"""把 decision.yaml 的点画到图上（仿主工作区 docs/field/render_candidates.py 的风格）。

画图规则（与 candidates.png 同款）：
- 同一个 region 一个颜色，不同 region 不同颜色；zone 用 marker 区分
  （home 方块、patrol 圆点），点旁标 id 和 zone 缩写
- 洞：两对正点之间画虚线（闸轴），洞名标在中点；对正点画白芯圆圈，
  旁边标 region（入口/出口由车在哪个区域决定，图上不分）
- 自动按点的外扩定坐标范围（不写死场地尺寸，走廊/RM 场地/任何地图都能画）
- enable_tunnel = false 的配置（没有 region/洞）也能画：全灰色 + 无洞图层

用法：
  python3 tools/plot_points.py                          # 画 config/decision.yaml
  python3 tools/plot_points.py -c config/corridor_demo.yaml
  python3 tools/plot_points.py -c config/plain_demo.yaml -o docs/plain.png
输出默认写到 docs/<配置名>.png（相对本仓库根）。

注意：画的是 yaml 里的**场地系**原始坐标（field_to_map 变换前的），和手填的一致；
导航侧实际使用的地图系坐标 = 加载时变换后的，如需对齐先跑 tools/align_map.py。
"""
import argparse
import io
import os

import yaml

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # 仓库根

# 已知 RM 五区用主工作区同款配色；其他自定义区域按 tab10 轮换
REGION_COLORS = {'own_low': '#1f77b4', 'own_high': '#17becf', 'mid': '#2ca02c',
                 'enemy_low': '#d62728', 'enemy_high': '#ff7f0e'}
TAB10 = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
         '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
ZONE_MARK = {'home': 's', 'patrol': 'o'}
ZONE_NAME = {'home': 'home', 'patrol': 'patrol'}


def region_color(regions, name):
    if name in REGION_COLORS:
        return REGION_COLORS[name]
    return TAB10[regions.index(name) % len(TAB10)]


def load(path):
    doc = yaml.safe_load(io.open(path, encoding='utf-8'))
    pts = []
    for p in doc.get('points') or []:
        pts.append({'id': p['id'], 'x': float(p['x']), 'y': float(p['y']),
                    'zone': p.get('zone', 'patrol'),
                    'region': p.get('region')})   # None = 没标（enable_tunnel=false）
    tunnels = []
    for t in doc.get('tunnels') or []:
        aligns = []
        for a in t.get('align') or []:
            aligns.append({'p': (float(a['p'][0]), float(a['p'][1])),
                           'region': a.get('region')})
        tunnels.append({'name': t.get('name', 'tunnel'),
                        'align': aligns,
                        'center': tuple(t['center']) if t.get('center') else None})
    return doc, pts, tunnels


def draw(doc, pts, tunnels, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    # 标题/图例带中文：挂一个系统里的 CJK 字体（没有就退回默认，仅字形缺失不影响布局）
    for f in matplotlib.font_manager.fontManager.ttflist:
        if 'CJK' in f.name or 'UMing' in f.name or 'UKai' in f.name:
            plt.rcParams['font.family'] = f.name
            break

    enable_tunnel = doc.get('enable_tunnel', True)
    regions = doc.get('regions') or []

    fig, ax = plt.subplots(figsize=(14, 8.0))
    ax.set_facecolor('#f2f2f2')

    # ---- 洞层：闸轴虚线 + 洞名 + 白芯对正点 ----
    # 标签一律带白底半透明框：洞名沿闸轴法向**背离场地质心**一侧放（隧道都挤在
    # 中带，向外偏才有空）；对正点标签沿闸轴**向外**放（洞两端外侧通常是空的）
    white_box = dict(facecolor='white', alpha=0.75, edgecolor='none', pad=1.0)
    import math as _m
    all_x = [p['x'] for p in pts] + [a['p'][0] for t in tunnels for a in t['align']]
    all_y = [p['y'] for p in pts] + [a['p'][1] for t in tunnels for a in t['align']]
    gx, gy = sum(all_x) / len(all_x), sum(all_y) / len(all_y)   # 场地质心
    for t in tunnels:
        if len(t['align']) != 2:
            continue
        (x0, y0), (x1, y1) = t['align'][0]['p'], t['align'][1]['p']
        ax.plot([x0, x1], [y0, y1], '--', color='#888', lw=1.6, zorder=2)
        cx, cy = t['center'] or ((x0 + x1) / 2, (y0 + y1) / 2)
        ax.add_patch(Rectangle((min(x0, x1) - 0.4, min(y0, y1) - 0.4),
                               abs(x1 - x0) + 0.8, abs(y1 - y0) + 0.8,
                               fill=False, ec='#999', ls=':', lw=1.0, zorder=1))
        axlen = _m.hypot(x1 - x0, y1 - y0) or 1.0
        nx, ny = -(y1 - y0) / axlen, (x1 - x0) / axlen        # 单位法向
        outward = 1.0 if (nx * (cx - gx) + ny * (cy - gy)) >= 0 else -1.0
        ax.text(cx + nx * outward * 1.1, cy + ny * outward * 1.1, t['name'], fontsize=7,
                color='#555', ha='center', va='center', bbox=white_box, zorder=5)
        for i, a in enumerate(t['align']):
            px, py = a['p']
            ox, oy = t['align'][1 - i]['p']
            dx, dy = px - ox, py - oy
            dl = _m.hypot(dx, dy) or 1.0
            ax.plot(px, py, 'o', ms=8, mfc='white', mec='#333', mew=1.4, ls='none',
                    zorder=4)
            ax.annotate(f"A{i + 1} {a['region'] or '?'}", (px, py), fontsize=5.4,
                        color='#333', ha='center',
                        xytext=(dx / dl * 15.0, dy / dl * 15.0),
                        textcoords='offset points', bbox=white_box, zorder=5)

    # ---- 点层：颜色 = region，marker = zone，标注 id·zone ----
    for p in pts:
        col = region_color(regions, p['region']) if (enable_tunnel and p['region']) \
            else '#777777'
        ax.plot(p['x'], p['y'], marker=ZONE_MARK.get(p['zone'], 'o'), ms=9, mfc=col,
                mec='k', mew=0.6, ls='none', zorder=3)
        ax.annotate(f"{p['id']}·{ZONE_NAME.get(p['zone'], p['zone'])[:3]}", (p['x'], p['y']),
                    fontsize=6.0, color='#222', xytext=(5, -11),
                    textcoords='offset points', bbox=white_box, zorder=5)

    # ---- 图例：region 颜色 + zone marker + 对正点 ----
    handles = []
    if enable_tunnel and regions:
        handles += [plt.Line2D([], [], marker='o', ls='none', mfc=region_color(regions, r),
                               mec='k', ms=9, label=r) for r in regions]
    else:
        handles.append(plt.Line2D([], [], marker='o', ls='none', mfc='#777777',
                                  mec='k', ms=9, label='point (no region)'))
    handles += [plt.Line2D([], [], marker=m, ls='none', mfc='w', mec='k', ms=9, label=z)
                for z, m in ZONE_MARK.items()]
    if tunnels:
        handles.append(plt.Line2D([], [], marker='o', ls='none', mfc='white', mec='#333',
                                  mew=1.4, ms=8, label='align'))
        handles.append(plt.Line2D([], [], ls='--', color='#888', lw=1.6, label='gate axis'))
    ax.legend(handles=handles, fontsize=7, ncol=2, loc='upper left', framealpha=.9)

    # ---- 坐标范围：点的包围盒外扩 1.5 m，网格按范围选 1/2 m ----
    xs = [p['x'] for p in pts] + [a['p'][0] for t in tunnels for a in t['align']]
    ys = [p['y'] for p in pts] + [a['p'][1] for t in tunnels for a in t['align']]
    x0, x1 = min(xs) - 1.5, max(xs) + 1.5
    y0, y1 = min(ys) - 1.5, max(ys) + 1.5
    step = 2 if (x1 - x0) > 14 or (y1 - y0) > 10 else 1
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    import math
    ax.set_xticks(range(math.floor(x0), math.ceil(x1) + 1, step))
    ax.set_yticks(range(math.floor(y0), math.ceil(y1) + 1, step))
    ax.grid(alpha=.3)
    ax.set_aspect('equal', adjustable='datalim')
    mode = f'region 路由过洞（{len(regions)} 区 {len(tunnels)} 洞）' if enable_tunnel \
        else '过洞关闭（无 region，所有点直连）'
    ax.set_title(f'{os.path.basename(out)}: {len(pts)} points · {mode} · '
                 f'color = region, marker = zone')
    fig.tight_layout()
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    fig.savefig(out, dpi=115)
    print(f'saved: {out}（点 {len(pts)} 个，洞 {len(tunnels)} 条）')


def main():
    ap = argparse.ArgumentParser(description='画 decision.yaml 的点位图')
    ap.add_argument('-c', '--config', default=os.path.join(BASE, 'src/bbt_decide/config/decision.yaml'))
    ap.add_argument('-o', '--out', default=None, help='默认 docs/<配置名>.png')
    args = ap.parse_args()
    out = args.out or os.path.join(BASE, 'docs', os.path.splitext(os.path.basename(args.config))[0] + '.png')
    doc, pts, tunnels = load(args.config)
    draw(doc, pts, tunnels, out)


if __name__ == '__main__':
    main()
