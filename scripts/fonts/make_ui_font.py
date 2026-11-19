#!/usr/bin/env python3
"""生成界面字体子集（M1-13，D-028）：Noto Sans SC Regular / Bold → assets/fonts/。

输入为 Noto CJK 官方发布包 Sans2.004 的 18_NotoSansSC.zip（sha256 见下方 SOURCE_SHA256），
用 fontTools（版本见 FONTTOOLS_VERSION）裁剪为界面所需字符：基本拉丁、Latin-1、常用标点与符号、
箭头、数学运算符、方框绘制、CJK 符号与全角字符、GB2312 全部 6763 个汉字。
同一输入与同一 fontTools 版本得到逐字节相同的输出（脚本固定了时间戳等可变字段）。

用法：python3 scripts/fonts/make_ui_font.py <18_NotoSansSC.zip> <输出目录>
需要：pip install fonttools==4.60.1
"""
import hashlib
import io
import os
import sys
import zipfile

SOURCE_URL = "https://github.com/notofonts/noto-cjk/releases/download/Sans2.004/18_NotoSansSC.zip"
SOURCE_SHA256 = "4d107c09ada479d3e48b6e78c83835773cbd9214bf6e12cdb7b60f8e068292ec"
FONTTOOLS_VERSION = "4.60.1"
WEIGHTS = ["Regular", "Bold"]


def charset():
    codepoints = set()
    ranges = [
        (0x0020, 0x007E),  # 基本拉丁
        (0x00A0, 0x00FF),  # Latin-1 补充（°、±、×、÷ 等）
        (0x2000, 0x206F),  # 常用标点（—、…、‰ 等）
        (0x2100, 0x214F),  # 字母式符号（℃、№ 等）
        (0x2190, 0x21FF),  # 箭头
        (0x2200, 0x22FF),  # 数学运算符
        (0x2460, 0x24FF),  # 带圈数字
        (0x2500, 0x257F),  # 方框绘制
        (0x25A0, 0x25FF),  # 几何图形
        (0x3000, 0x303F),  # CJK 符号和标点
        (0xFF00, 0xFFEF),  # 全角 ASCII 与半角片假名
    ]
    for lo, hi in ranges:
        codepoints.update(range(lo, hi + 1))
    # GB2312：全部汉字（16–87 区）与符号区，逐码解码
    for hi in range(0xA1, 0xF8):
        for lo in range(0xA1, 0xFF):
            try:
                ch = bytes([hi, lo]).decode("gb2312")
            except UnicodeDecodeError:
                continue
            codepoints.add(ord(ch))
    return sorted(codepoints)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    source, out_dir = sys.argv[1], sys.argv[2]
    data = open(source, "rb").read()
    digest = hashlib.sha256(data).hexdigest()
    if digest != SOURCE_SHA256:
        print(f"错误：{source} 的 sha256 为 {digest}，与锁定值 {SOURCE_SHA256} 不符", file=sys.stderr)
        return 1

    import fontTools
    from fontTools import subset
    from fontTools.ttLib import TTFont

    if fontTools.version != FONTTOOLS_VERSION:
        print(f"错误：需要 fonttools {FONTTOOLS_VERSION}，当前为 {fontTools.version}", file=sys.stderr)
        return 1

    codepoints = charset()
    os.makedirs(out_dir, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        for weight in WEIGHTS:
            font = TTFont(io.BytesIO(archive.read(f"NotoSansSC-{weight}.otf")), recalcTimestamp=False)
            options = subset.Options()
            options.name_IDs = ["*"]          # 保留版权、许可等名称记录
            options.name_languages = ["*"]
            options.notdef_outline = True
            options.recalc_timestamp = False
            subsetter = subset.Subsetter(options=options)
            subsetter.populate(unicodes=codepoints)
            subsetter.subset(font)
            out = os.path.join(out_dir, f"NotoSansSC-{weight}-Subset.otf")
            font.save(out)
            blob = open(out, "rb").read()
            cmap = font.getBestCmap()
            print(f"{out}: {len(blob)} 字节，{len(cmap)} 个字符，sha256 {hashlib.sha256(blob).hexdigest()}")
        with open(os.path.join(out_dir, "OFL.txt"), "wb") as f:
            f.write(archive.read("LICENSE"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
