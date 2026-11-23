#!/usr/bin/env python3
"""检查各验证脚本中变异测试的锚点仍在源码中唯一出现（M1-11 阶段验收的快速预检，不编译）。

变异测试把源码中的一段文字（锚点）替换为已知错误。源码改动后锚点可能不再出现或出现多次，
此时该变异无法注入，验证脚本会在长时间构建之后才报告“注入失败”。本脚本在构建前一次查完全部脚本。

用法：python3 scripts/verify/check_mutation_anchors.py [脚本 ...]（默认 scripts/verify/m0_*.sh 与 m1_*.sh）
每个变异打印一行；全部唯一时退出码 0，否则 1。
"""
import ast
import glob
import os
import re
import sys


def mutations_of(script):
    text = open(script, encoding="utf-8").read()
    block = re.search(r"mutations = \{\n(.*?)\n\}\n", text, re.S)
    if not block:
        return []
    body = re.sub(r"^\s*#.*$", "", block.group(1), flags=re.M)
    table = ast.literal_eval("{" + body + "}")
    listing = re.search(r"<<'LIST'\n(.*?)\nLIST", text, re.S)
    if not listing:
        raise SystemExit(f"{script}：找到变异表但没有 LIST")
    single = re.search(r'^\s*src="([^"]+)"', text, re.M)
    result = []
    for line in listing.group(1).splitlines():
        parts = line.split("|")
        name = parts[0]
        path = parts[1] if len(parts) > 1 else (single.group(1) if single else None)
        if name not in table or path is None:
            raise SystemExit(f"{script}：变异“{name}”在表中缺失或没有源文件")
        result.append((name, path, table[name][0]))
    return result


def main():
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    os.chdir(root)
    scripts = sys.argv[1:] or sorted(glob.glob("scripts/verify/m0_*.sh") + glob.glob("scripts/verify/m1_*.sh"))
    total = 0
    bad = 0
    for script in scripts:
        entries = mutations_of(script)
        for name, path, anchor in entries:
            total += 1
            try:
                count = open(path, encoding="utf-8").read().count(anchor)
            except FileNotFoundError:
                count = -1
            if count != 1:
                bad += 1
                where = "源文件不存在" if count < 0 else f"出现 {count} 次"
                print(f"不唯一 {os.path.basename(script)}：{name}（{path}，{where}）")
        if entries:
            print(f"{os.path.basename(script)}：{len(entries)} 个变异")
    print(f"锚点唯一：{total - bad}/{total}")
    return 0 if bad == 0 and total > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
