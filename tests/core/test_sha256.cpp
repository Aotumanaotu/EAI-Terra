// SHA-256（M3-07）：FIPS 180-4 的标准测试向量；分段更新与一次更新结果相同；文件摘要；读不出的文件。

#include <cstdio>
#include <fstream>
#include <string>

#include "terra/core/sha256.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

}  // namespace

int main(int argc, char** argv) {
    check(terra::sha256Hex("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", "空串");
    check(terra::sha256Hex("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "abc");
    check(terra::sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
              "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1",
          "448 位消息（跨块填充）");
    const std::string million(1000000, 'a');
    check(terra::sha256Hex(million) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0", "一百万个 a");
    terra::Sha256 pieces;
    for (std::size_t i = 0; i < million.size(); i += 997) pieces.update(million.substr(i, 997));
    check(pieces.finishHex() == terra::sha256Hex(million), "分段更新（997 字节一段）与一次更新相同");
    const std::string dir = argc > 1 ? argv[1] : ".";
    const std::string file = dir + "/sha256_test.txt";
    { std::ofstream(file, std::ios::binary) << "abc"; }
    const auto digest = terra::sha256File(file);
    check(digest && *digest == terra::sha256Hex("abc"), "文件摘要");
    check(!terra::sha256File(dir + "/no_such_file").has_value(), "文件不存在时为空");
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
