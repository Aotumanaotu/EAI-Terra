#!/usr/bin/env python3
"""展开 xacro 文件（M4-08a 模型导入）：使用随 Terra 提供的官方 xacro（third_party/xacro，2.1.1，BSD-3-Clause），不需要 ROS。

用法：xacro_expand.py --xacro-dir <third_party/xacro> [--package 包名=目录 ...] [-o 输出] <输入> [名字:=值 ...]

xacro 中的 $(find 包名) 原本经 ROS 的包索引（ament_index_python）查找；这里换成命令行给出的包目录表。
找不到的包按 xacro 的方式报错（退出码非 0），错误信息中给出用 --package 指定的办法。
"""

import os
import sys
import types


def main(argv):
    xacro_dir = None
    packages = {}
    rest = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--xacro-dir" and i + 1 < len(argv):
            xacro_dir = argv[i + 1]
            i += 2
        elif a == "--package" and i + 1 < len(argv):
            name, sep, path = argv[i + 1].partition("=")
            if not sep or not name or not path:
                print("错误：--package 应为 包名=目录，实际为 %s" % argv[i + 1], file=sys.stderr)
                return 2
            packages[name] = os.path.abspath(path)
            i += 2
        else:
            rest.append(a)
            i += 1
    if xacro_dir is None or not rest:
        print(__doc__, file=sys.stderr)
        return 2

    class PackageNotFoundError(KeyError):
        pass

    def get_package_share_directory(name):
        if name in packages:
            return packages[name]
        raise PackageNotFoundError(
            "找不到包“%s”：导入时用 --package %s=<目录> 指定（目录为含 package.xml 的包目录）" % (name, name))

    ament = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = get_package_share_directory
    ament_packages.PackageNotFoundError = PackageNotFoundError
    ament.packages = ament_packages
    sys.modules["ament_index_python"] = ament
    sys.modules["ament_index_python.packages"] = ament_packages

    sys.dont_write_bytecode = True  # 不在 third_party/xacro 中写 __pycache__
    sys.path.insert(0, xacro_dir)
    import xacro  # noqa: E402  官方实现，未修改

    sys.argv = ["xacro"] + rest
    try:
        xacro.main()
    except SystemExit as e:
        return e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
