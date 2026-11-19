// 渲染依赖探针（M1-02）：只验证 GLFW、glad、GLM 能一起构建、链接并在目标环境中工作，
// 类似 M0-02 的 MuJoCo 探针，不是渲染器实现。
//
// 检查：
//   1. GLFW 初始化，创建隐藏窗口与 OpenGL 4.5 core、forward-compatible 上下文（D-021）；
//   2. glad 加载函数指针，实际版本 ≥ 4.5；
//   3. 用 4.5 的直接状态访问（DSA）创建离屏帧缓冲，清屏为已知颜色后回读像素；
//   4. GLM 透视投影与视图矩阵的数值检查（Z 向上，沿用 D-012 坐标约定）。
// 用法：test_render_deps [期望的渲染器子串]
//   自动化测试传入 llvmpipe，确认运行在 Xvfb + Mesa 软件渲染上（D-022）；
//   桌面手动运行不传参数，只打印实际使用的 GPU。

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace {

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

void onGlfwError(int code, const char* description) {
    std::fprintf(stderr, "GLFW 错误 0x%x：%s\n", code, description);
}

std::string glString(GLenum name) {
    const GLubyte* value = glGetString(name);
    return value ? std::string{reinterpret_cast<const char*>(value)} : std::string{"<null>"};
}

void checkOffscreenClear() {
    constexpr GLsizei kSize = 16;
    GLuint renderbuffer = 0;
    GLuint framebuffer = 0;
    glCreateRenderbuffers(1, &renderbuffer);
    glNamedRenderbufferStorage(renderbuffer, GL_RGBA8, kSize, kSize);
    glCreateFramebuffers(1, &framebuffer);
    glNamedFramebufferRenderbuffer(framebuffer, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, renderbuffer);
    const GLenum status = glCheckNamedFramebufferStatus(framebuffer, GL_FRAMEBUFFER);
    check(status == GL_FRAMEBUFFER_COMPLETE, "DSA 离屏帧缓冲完整");

    const GLfloat color[4] = {0.25f, 0.5f, 0.75f, 1.0f};
    glClearNamedFramebufferfv(framebuffer, GL_COLOR, 0, color);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    unsigned char pixel[4] = {0, 0, 0, 0};
    glReadPixels(kSize / 2, kSize / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    std::printf("INFO 回读像素 RGBA = (%u, %u, %u, %u)\n", pixel[0], pixel[1], pixel[2], pixel[3]);
    // 0.25/0.5/0.75 归一化到 8 位为 64/128/191（允许舍入差 1）
    const auto near = [](unsigned value, unsigned expected) { return value + 1 >= expected && value <= expected + 1; };
    check(near(pixel[0], 64) && near(pixel[1], 128) && near(pixel[2], 191) && pixel[3] == 255, "回读像素等于清屏颜色");
    check(glGetError() == GL_NO_ERROR, "OpenGL 无错误");

    glDeleteFramebuffers(1, &framebuffer);
    glDeleteRenderbuffers(1, &renderbuffer);
}

void checkGlm() {
    // 相机在 (-5, 0, 2) 看向原点，Z 向上。原点应投影到视野中心，+Z 方向的点应在中心上方。
    const glm::dmat4 view = glm::lookAt(glm::dvec3{-5.0, 0.0, 2.0}, glm::dvec3{0.0}, glm::dvec3{0.0, 0.0, 1.0});
    const glm::dmat4 proj = glm::perspective(glm::radians(60.0), 16.0 / 9.0, 0.1, 100.0);
    const auto ndc = [&](const glm::dvec3& p) {
        const glm::dvec4 clip = proj * view * glm::dvec4{p, 1.0};
        return glm::dvec3{clip} / clip.w;
    };
    const glm::dvec3 origin = ndc(glm::dvec3{0.0});
    const glm::dvec3 up = ndc(glm::dvec3{0.0, 0.0, 1.0});
    const glm::dvec3 left = ndc(glm::dvec3{0.0, 1.0, 0.0});  // Terra 约定 +Y 向左
    check(std::fabs(origin.x) < 1e-12 && std::fabs(origin.y) < 1e-12, "GLM：注视点投影到视野中心");
    check(up.y > 0.0, "GLM：+Z 投影在屏幕上方");
    check(left.x < 0.0, "GLM：从 -X 方向看向原点时，+Y 投影在屏幕左侧");
    check(origin.z > -1.0 && origin.z < 1.0, "GLM：注视点在裁剪深度范围内");
}

}  // namespace

int main(int argc, char** argv) {
    const char* expectedRenderer = argc > 1 ? argv[1] : nullptr;

    glfwSetErrorCallback(onGlfwError);
    if (!glfwInit()) {
        std::fprintf(stderr, "FAILED: glfwInit 失败（是否有可用的显示服务？）\n");
        return 1;
    }
    std::printf("INFO GLFW %s，平台 %d\n", glfwGetVersionString(), glfwGetPlatform());

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "terra render deps probe", nullptr, nullptr);
    if (window == nullptr) {
        std::fprintf(stderr, "FAILED: 无法创建 OpenGL 4.5 core 上下文\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);

    const int version = gladLoadGL(glfwGetProcAddress);
    const int major = GLAD_VERSION_MAJOR(version);
    const int minor = GLAD_VERSION_MINOR(version);
    const std::string renderer = glString(GL_RENDERER);
    std::printf("INFO OpenGL %d.%d，GL_VERSION=%s\n", major, minor, glString(GL_VERSION).c_str());
    std::printf("INFO GL_RENDERER=%s，GL_VENDOR=%s\n", renderer.c_str(), glString(GL_VENDOR).c_str());
    check(version != 0, "glad 加载 OpenGL 函数");
    check(major > 4 || (major == 4 && minor >= 5), "OpenGL 版本 ≥ 4.5");
    GLint profile = 0;
    glGetIntegerv(GL_CONTEXT_PROFILE_MASK, &profile);
    check((profile & GL_CONTEXT_CORE_PROFILE_BIT) != 0, "core profile");
    if (expectedRenderer != nullptr) {
        check(renderer.find(expectedRenderer) != std::string::npos,
              std::string{"渲染器包含 '"} + expectedRenderer + "'（自动化测试必须运行在固定环境上，D-022）");
    }

    if (version != 0) {
        checkOffscreenClear();
    }
    checkGlm();

    glfwDestroyWindow(window);
    glfwTerminate();
    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
