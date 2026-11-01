#!/usr/bin/env python3
"""合并 build/<pkg>/compile_commands.json 到仓库根，并生成 .clangd，给 clangd 用。

为什么需要：colcon 每个包各自生成 build/<pkg>/compile_commands.json，
clangd 只会沿源文件向上找**一个**最近的 compile_commands.json。仓库根若只是
指向某一个包的软链接，其余包（如 ros2_libxr）的文件全部没有编译参数，
clangd 就报"找不到 rclcpp/xxx.hpp / app_framework.hpp"这类头文件错误。

做两件事：
  1) 合并全部包的条目 → 仓库根 compile_commands.json（真实文件）；
  2) 从条目里提取全部 -I / -isystem / -D → 仓库根 .clangd 的 CompileFlags.Add。
     单独打开一个头文件时，clangd 关联 TU 靠索引/启发式（可能关联错，比如把
     SharedTopic.hpp 关联到 libxr 的某个 .cpp，甚至走无参数 fallback）——
     Add 的兜底路径保证无论关联到谁，include 都能解析。

用法：colcon build 之后跑一次
    python3 tools/merge_compile_commands.py
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"

# 只兜底 include 路径与库自身的宏；-std 不加（避免覆盖各 TU 自己的标准版本）。
SKIP_DEFINES = re.compile(r"(ros2_libxr_EXPORTS|bbt_decide_core_EXPORTS)")


def extract_flags(entries):
    includes, isystems, defines = [], [], []
    for e in entries:
        cmd = e.get("command") or " ".join(e.get("arguments", []))
        for flag in cmd.split():
            if flag.startswith("-I") and len(flag) > 2:
                includes.append(flag[2:])
            elif flag.startswith("-D") and len(flag) > 2:
                if not SKIP_DEFINES.search(flag):
                    defines.append(flag[2:])
        for m in re.finditer(r"-isystem\s+(\S+)", cmd):
            isystems.append(m.group(1))
    # 去重保序
    def uniq(xs):
        return list(dict.fromkeys(xs))
    return uniq(includes), uniq(isystems), uniq(defines)


def main() -> int:
    merged = []
    seen = set()
    for cc in sorted(BUILD.glob("*/compile_commands.json")):
        try:
            entries = json.loads(cc.read_text())
        except (OSError, json.JSONDecodeError) as e:
            print(f"跳过 {cc}: {e}", file=sys.stderr)
            continue
        pkg = cc.parent.name
        n = 0
        for e in entries:
            key = (e.get("directory"), e.get("file"), e.get("output"))
            if key in seen:
                continue
            seen.add(key)
            merged.append(e)
            n += 1
        print(f"{pkg}: {n} 条")

    # ---- 头文件补条目（根治 clangd 的头文件解析问题）----
    # 头文件不在 CDB 里时，clangd 靠"关联到某个 TU"的启发式选参数，选错/选不中
    # 就退化为无参数解析（C++17）→ libxr 的 concept 报错、"app_framework.hpp
    # 找不到"等。项目 .clangd 的 CompileFlags 兜底又可能被判 untrusted 而忽略。
    # CDB 条目 clangd 无条件信任——直接给每个头文件合成一条（借用本包代表 TU
    # 的完整命令，换掉输入文件即可）。
    def header_entries(pkg_name, tu_suffix, roots):
        for e in merged:
            f = str(e.get("file", ""))
            if f.endswith(tu_suffix) and pkg_name in f:
                cmd = e.get("command") or ""
                directory = e.get("directory")
                break
        else:
            return 0
        out = 0
        for root in roots:
            root = ROOT / root
            if not root.is_dir():
                continue
            for h in root.rglob("*"):
                if h.suffix not in (".h", ".hpp") or not h.is_file():
                    continue
                hf = str(h)
                if hf in {str(x.get("file", "")) for x in merged}:
                    continue  # 已有真实条目
                # 换掉命令末尾的输入文件（-c <file> 形式）
                new_cmd = re.sub(r"(\s(?:-c\s+)?)[^ ]+\.(?:cpp|cxx|cc|c)(\s*)$",
                                 r"\g<1>" + hf + r"\g<2>", cmd)
                if new_cmd == cmd:  # 没匹配上就直接整体替换末尾 token
                    tokens = cmd.rsplit(" ", 1)
                    new_cmd = (tokens[0] + " " + hf) if len(tokens) == 2 else cmd + " " + hf
                merged.append({"directory": directory, "file": hf,
                               "command": new_cmd})
                out += 1
        return out

    n_hdr = header_entries(
        "ros2_libxr", "src/bridge_node.cpp",
        ["src/ros2_libxr/include"])
    n_hdr += header_entries("bbt_decide", "src/planner.cpp", ["src/bbt_decide/include"])
    print(f"头文件补条目：{n_hdr} 条（沿用本包真实编译参数）")

    out = ROOT / "compile_commands.json"
    if out.is_symlink():
        out.unlink()  # 历史上是指向单包的软链接，替换为真实文件
    out.write_text(json.dumps(merged, ensure_ascii=False, indent=1))
    print(f"共 {len(merged)} 条 → {out}")

    includes, isystems, defines = extract_flags(merged)
    clangd = ROOT / ".clangd"
    lines = ["# 由 tools/merge_compile_commands.py 生成，勿手改；重新 build 后再跑一次脚本。",
             "CompileFlags:", "  Add:"]
    for p in includes:
        lines.append(f"    - '-I{p}'")
    for p in isystems:
        lines.append(f"    - '-isystem'")
        lines.append(f"    - '{p}'")
    for d in defines:
        lines.append(f"    - '-D{d}'")
    clangd.write_text("\n".join(lines) + "\n")
    print(f"兜底 flags：{len(includes)} 个 -I，{len(isystems)} 个 -isystem，"
          f"{len(defines)} 个 -D → {clangd}")

    # The raw-frame bridge is C++17; the historical LibXR dependency is not built.
    bridge_pkg = ROOT / "src/ros2_libxr"
    if bridge_pkg.is_dir():
        nested = bridge_pkg / ".clangd"
        nested.write_text(
            "# 由 tools/merge_compile_commands.py 生成，当前原始帧桥使用 C++17。\n"
            "CompileFlags:\n  Add:\n    - '-std=gnu++17'\n")
        print(f"桥编译标准 → {nested}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
