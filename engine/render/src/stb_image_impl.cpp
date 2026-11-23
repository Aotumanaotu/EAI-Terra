// stb_image 与 stb_image_write 的唯一实现单元（M1-08b 起 HDR，M1-09 起加 PNG 与 JPEG，M1-10 起写 PNG）。其他文件只包含声明。
#define STBI_ONLY_HDR
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
