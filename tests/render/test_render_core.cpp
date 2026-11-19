// 渲染核心测试（M1-06）。
//
// 模式：
//   test_render_core [期望的渲染器子串]   自动化测试（Xvfb + llvmpipe，D-022）：离屏渲染、像素回读、深度测试、
//                                         着色器错误定位、调试输出、资源 RAII
//   test_render_core --show <秒>          展示：在桌面会话用真实 GPU 打开可见窗口，显示旋转的渐变三角形与
//                                         前方的蓝色三角形（深度测试），到时自动关闭；Esc 经 Terra 输入系统提前关闭
//
// 只有测试代码直接调用 glad（注入一个 GL 错误、检查对象是否已删除）；渲染接口本身不暴露 GL 类型。

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <glad/gl.h>

#include "terra/input/input_system.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/render/context.hpp"
#include "terra/render/resources.hpp"

namespace {

namespace r = terra::render;
namespace in = terra::input;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

struct Vertex {
    float position[3];
    float color[3];
};

// 0..2：大三角形，红色，z = 0.5（远）
// 3..5：小三角形，蓝色，z = -0.5（近）
// 6..8：展示用渐变三角形，z = 0
const Vertex kVertices[] = {
    {{-0.9f, -0.9f, 0.5f}, {1.0f, 0.0f, 0.0f}}, {{0.9f, -0.9f, 0.5f}, {1.0f, 0.0f, 0.0f}}, {{0.0f, 0.9f, 0.5f}, {1.0f, 0.0f, 0.0f}},
    {{-0.3f, -0.3f, -0.5f}, {0.0f, 0.0f, 1.0f}}, {{0.3f, -0.3f, -0.5f}, {0.0f, 0.0f, 1.0f}}, {{0.0f, 0.3f, -0.5f}, {0.0f, 0.0f, 1.0f}},
    {{-0.8f, -0.7f, 0.0f}, {0.95f, 0.35f, 0.25f}}, {{0.8f, -0.7f, 0.0f}, {0.25f, 0.75f, 0.45f}}, {{0.0f, 0.85f, 0.0f}, {0.30f, 0.45f, 0.95f}},
};

const char* kVertexShader = R"(#version 450 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
uniform mat4 uTransform;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uTransform * vec4(aPosition, 1.0);
}
)";

const char* kFragmentShader = R"(#version 450 core
in vec3 vColor;
out vec4 fragColor;
void main() {
    fragColor = vec4(vColor, 1.0);
}
)";

// 绕 Z 轴旋转 angle，并按宽高比压缩 x，保持形状不变形。列主序。
void transform(float angle, float aspect, float out[16]) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    const float sx = aspect > 1.0f ? 1.0f / aspect : 1.0f;
    const float sy = aspect > 1.0f ? 1.0f : aspect;
    const float m[16] = {sx * c, sy * s, 0, 0, -sx * s, sy * c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    std::memcpy(out, m, sizeof(m));
}

struct Scene {
    r::ShaderProgram program;
    r::Buffer vertices;
    r::VertexArray vao;
    int transformLocation = -1;
};

bool makeScene(Scene& scene) {
    auto shader = r::ShaderProgram::create({{r::ShaderStage::Vertex, "basic.vert", kVertexShader},
                                            {r::ShaderStage::Fragment, "basic.frag", kFragmentShader}});
    if (!shader.program) {
        check(false, "编译基础着色器：" + (shader.error ? shader.error->summary() : std::string{}));
        return false;
    }
    scene.program = std::move(*shader.program);
    scene.transformLocation = scene.program.uniformLocation("uTransform");
    scene.vertices = r::Buffer::create(sizeof(kVertices), kVertices);
    scene.vao = r::VertexArray::create();
    scene.vao.setVertexBuffer(0, scene.vertices, 0, sizeof(Vertex));
    scene.vao.setFloatAttribute(0, 0, 3, offsetof(Vertex, position));
    scene.vao.setFloatAttribute(1, 0, 3, offsetof(Vertex, color));
    return true;
}

struct Rgba {
    unsigned r, g, b, a;
};

Rgba pixel(const r::Framebuffer& fb, int x, int y) {
    const auto p = fb.readPixels(x, y, 1, 1);
    if (p.size() != 4) return {0, 0, 0, 0};
    return {p[0], p[1], p[2], p[3]};
}

bool near(Rgba p, unsigned red, unsigned green, unsigned blue) {
    const auto within = [](unsigned v, unsigned e) { return v + 1 >= e && v <= e + 1; };
    return within(p.r, red) && within(p.g, green) && within(p.b, blue) && p.a == 255;
}

std::string str(Rgba p) {
    return "(" + std::to_string(p.r) + ", " + std::to_string(p.g) + ", " + std::to_string(p.b) + ", " +
           std::to_string(p.a) + ")";
}

void testDrawing(r::RenderContext& ctx, Scene& scene) {
    auto made = r::Framebuffer::create(64, 64, true);
    check(made.framebuffer.has_value(), "创建 64×64 离屏帧缓冲（颜色 + 深度）：" + made.error.value_or(""));
    if (!made.framebuffer) return;
    r::Framebuffer& fb = *made.framebuffer;
    check(!r::Framebuffer::create(0, 64, true).framebuffer.has_value(), "尺寸 0 的帧缓冲被拒绝");

    float identity[16];
    transform(0.0f, 1.0f, identity);
    scene.program.setUniformMatrix4(scene.transformLocation, identity);
    check(scene.transformLocation >= 0, "找到 uniform uTransform");
    check(scene.program.uniformLocation("noSuchUniform") == -1, "不存在的 uniform 返回 -1");

    const r::Color clearColor{0.1f, 0.2f, 0.3f, 1.0f};  // 回读应为 (26, 51, 77)
    fb.bind();
    scene.program.use();
    scene.vao.bind();

    // 深度测试开启：先画近处的蓝色，再画远处的红色，中心仍应是蓝色
    ctx.setDepthTest(true);
    ctx.clear(clearColor);
    ctx.drawTriangles(3, 3);
    ctx.drawTriangles(0, 3);
    const Rgba center = pixel(fb, 32, 32);
    const Rgba redOnly = pixel(fb, 51, 6);
    const Rgba corner = pixel(fb, 2, 2);
    std::printf("INFO 深度开启：中心 %s，红色区域 %s，角落 %s\n", str(center).c_str(), str(redOnly).c_str(),
                str(corner).c_str());
    check(near(center, 0, 0, 255), "深度测试：中心为近处的蓝色（即使后画远处的红色）");
    check(near(redOnly, 255, 0, 0), "只被红色三角形覆盖的位置为红色");
    check(near(corner, 26, 51, 77), "未覆盖的角落为清屏颜色");

    // 深度测试关闭：后画的红色覆盖蓝色
    ctx.setDepthTest(false);
    ctx.clear(clearColor);
    ctx.drawTriangles(3, 3);
    ctx.drawTriangles(0, 3);
    check(near(pixel(fb, 32, 32), 255, 0, 0), "关闭深度测试：后画的红色覆盖蓝色");

    r::Framebuffer::bindDefault();
}

void testShaderErrors() {
    const char* broken = R"(#version 450 core
out vec4 fragColor;
void main() {
    fragColor = vec4(1.0) + undefinedVariable;
}
)";
    auto compile = r::ShaderProgram::create(
        {{r::ShaderStage::Vertex, "basic.vert", kVertexShader}, {r::ShaderStage::Fragment, "broken.frag", broken}});
    check(!compile.program && compile.error && compile.error->kind == r::ShaderError::Kind::Compile, "编译错误被报告");
    if (compile.error) {
        std::printf("INFO 编译错误：%s\n", compile.error->summary().c_str());
        check(compile.error->name == "broken.frag", "编译错误指出出错的着色器名称");
        check(compile.error->line == 4, "编译错误定位到第 4 行，实际 " +
                                            (compile.error->line ? std::to_string(*compile.error->line) : "无"));
    }

    const char* needsInput = R"(#version 450 core
in vec3 vMissing;
out vec4 fragColor;
void main() {
    fragColor = vec4(vMissing, 1.0);
}
)";
    const char* noOutput = R"(#version 450 core
void main() {
    gl_Position = vec4(0.0);
}
)";
    auto link = r::ShaderProgram::create(
        {{r::ShaderStage::Vertex, "empty.vert", noOutput}, {r::ShaderStage::Fragment, "needs.frag", needsInput}});
    check(!link.program && link.error && link.error->kind == r::ShaderError::Kind::Link, "链接错误被报告");
    if (link.error) std::printf("INFO 链接错误：%s\n", link.error->summary().c_str());

    auto invalid = r::ShaderProgram::create(
        {{r::ShaderStage::Vertex, "a.vert", kVertexShader}, {r::ShaderStage::Vertex, "b.vert", kVertexShader}});
    check(invalid.error && invalid.error->kind == r::ShaderError::Kind::InvalidInput, "两个顶点着色器被拒绝");

    check(r::parseShaderErrorLine("0:4(29): error: `x' undeclared") == 4, "解析 Mesa 格式行号");
    check(r::parseShaderErrorLine("0(12) : error C1008: undefined variable \"x\"") == 12, "解析 NVIDIA 格式行号");
    check(!r::parseShaderErrorLine("0:3(1): warning: unused").has_value(), "警告不当作错误行号");
}

void testDebugOutputAndResources(r::RenderContext& ctx) {
    (void)ctx.takeDebugMessages();
    const std::size_t before = ctx.errorMessageCount();
    glBindBuffer(0x1234, 0);  // 故意的非法枚举：GL_INVALID_ENUM
    (void)glGetError();
    const auto messages = ctx.takeDebugMessages();
    bool sawError = false;
    for (const auto& m : messages) {
        if (m.isError) {
            sawError = true;
            std::printf("INFO 调试输出捕获：id=%u %s\n", m.id, m.text.c_str());
        }
    }
    check(sawError && ctx.errorMessageCount() == before + 1, "调试输出同步捕获注入的 GL 错误");

    // RAII：作用域结束时对象被删除
    std::uint32_t last = 0;
    {
        std::vector<r::Buffer> buffers;
        for (int i = 0; i < 1000; ++i) buffers.push_back(r::Buffer::create(64, nullptr));
        last = buffers.back().name();
        check(glIsBuffer(last) == GL_TRUE, "缓冲存在");
    }
    check(glIsBuffer(last) == GL_FALSE, "1000 个缓冲在作用域结束时被删除");

    r::Buffer a = r::Buffer::create(16, nullptr, true);
    const std::uint32_t name = a.name();
    r::Buffer b = std::move(a);
    check(a.name() == 0 && b.name() == name && b.size() == 16, "移动后原对象为空，所有权转移");
    const float data[4] = {1, 2, 3, 4};
    check(b.update(0, sizeof(data), data), "动态缓冲可以更新");
    check(!b.update(8, sizeof(data), data), "越界更新被拒绝");
    r::Buffer fixed = r::Buffer::create(16, data);
    check(!fixed.update(0, 4, data), "静态缓冲不能更新");
    check(r::Buffer::create(0, nullptr).name() == 0, "零大小返回空缓冲");

    check(glGetError() == GL_NO_ERROR, "结束时没有未处理的 GL 错误");
    check(ctx.errorMessageCount() == before + 1, "除注入的一个外没有其他 GL 错误消息");
}

int runAutomated(const char* expectedRenderer) {
    in::InputSystem input;
    auto platform = terra::platform::glfw::createPlatform(input);
    if (!platform.platform) {
        std::fprintf(stderr, "FAILED: 创建平台：%s\n", platform.error ? platform.error->message.c_str() : "");
        return 1;
    }
    terra::platform::WindowDescription desc;
    desc.title = "terra render core test";
    desc.width = 64;
    desc.height = 64;
    desc.visible = false;
    desc.glDebugContext = true;
    auto window = platform.platform->createWindow(desc);
    if (!window.window) {
        std::fprintf(stderr, "FAILED: 创建窗口：%s\n", window.error ? window.error->message.c_str() : "");
        return 1;
    }

    auto created = r::RenderContext::create(window.window->glProcLoader());
    if (!created.context) {
        std::fprintf(stderr, "FAILED: 初始化渲染上下文：%s\n", created.error.value_or("").c_str());
        return 1;
    }
    r::RenderContext& ctx = *created.context;
    const auto& info = ctx.info();
    std::printf("INFO %s | %s | OpenGL %d.%d | GLSL %s | 调试上下文 %s\n", info.renderer.c_str(), info.vendor.c_str(),
                info.major, info.minor, info.glslVersion.c_str(), info.debugContext ? "是" : "否");
    check(info.major == 4 && info.minor >= 5, "OpenGL ≥ 4.5");
    check(info.debugContext, "调试上下文");
    if (expectedRenderer != nullptr) {
        check(info.renderer.find(expectedRenderer) != std::string::npos,
              std::string{"渲染器包含 '"} + expectedRenderer + "'（D-022）");
    }
    check(!r::RenderContext::create(nullptr).context, "空加载器被拒绝");

    {
        Scene scene;
        if (makeScene(scene)) testDrawing(ctx, scene);
    }
    testShaderErrors();
    testDebugOutputAndResources(ctx);
    return failures == 0 ? 0 : 1;
}

int runShow(double seconds) {
    in::InputSystem input;
    auto platform = terra::platform::glfw::createPlatform(input);
    if (!platform.platform) {
        std::fprintf(stderr, "展示：无法创建平台：%s\n", platform.error ? platform.error->message.c_str() : "");
        return 1;
    }
    in::ActionMapDefinition app{"app", 100, true, {}};
    in::ActionDefinition quit;
    quit.name = "quit";
    quit.type = in::ActionType::Button;
    quit.bindings = {in::Binding::single(in::key(in::Key::Escape))};
    app.actions.push_back(quit);
    (void)input.addActionMap(app);
    const in::ActionHandle quitAction = input.findAction("app", "quit");

    terra::platform::WindowDescription desc;
    desc.title = "EAI Terra · M1-06 渲染核心（" + std::to_string(static_cast<int>(seconds)) + " 秒后自动关闭，Esc 立即关闭）";
    desc.width = 960;
    desc.height = 600;
    auto window = platform.platform->createWindow(desc);
    if (!window.window) {
        std::fprintf(stderr, "展示：无法创建窗口：%s\n", window.error ? window.error->message.c_str() : "");
        return 1;
    }
    auto created = r::RenderContext::create(window.window->glProcLoader());
    if (!created.context) {
        std::fprintf(stderr, "展示：初始化渲染失败：%s\n", created.error.value_or("").c_str());
        return 1;
    }
    r::RenderContext& ctx = *created.context;
    std::printf("INFO 展示：%s，OpenGL %s\n", ctx.info().renderer.c_str(), ctx.info().version.c_str());

    Scene scene;
    if (!makeScene(scene)) return 1;
    const double start = platform.platform->time();
    int frames = 0;
    bool byEscape = false;
    while (!window.window->closeRequested()) {
        platform.platform->pollEvents();
        input.update();
        if (input.state(quitAction).pressed) {
            byEscape = true;
            break;
        }
        const double t = platform.platform->time() - start;
        if (t >= seconds) break;

        const auto fb = window.window->framebufferSize();
        ctx.setViewport(0, 0, fb.width, fb.height);
        ctx.setDepthTest(true);
        ctx.clear({0.07f, 0.08f, 0.10f, 1.0f});
        scene.program.use();
        scene.vao.bind();
        const float aspect = fb.height > 0 ? static_cast<float>(fb.width) / static_cast<float>(fb.height) : 1.0f;
        float m[16];
        transform(static_cast<float>(t) * 0.8f, aspect, m);
        scene.program.setUniformMatrix4(scene.transformLocation, m);
        ctx.drawTriangles(6, 3);  // 旋转的渐变三角形
        transform(0.0f, aspect, m);
        scene.program.setUniformMatrix4(scene.transformLocation, m);
        ctx.drawTriangles(3, 3);  // 静止的蓝色三角形，在前方（深度测试）
        window.window->swapBuffers();
        ++frames;
    }
    const double elapsed = platform.platform->time() - start;
    std::printf("INFO 展示结束（%s）：%d 帧，%.1f 秒，平均 %.0f FPS\n", byEscape ? "Esc" : "到时或关闭窗口", frames,
                elapsed, elapsed > 0 ? static_cast<double>(frames) / elapsed : 0.0);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--show") == 0) {
        const double seconds = argc > 2 ? std::atof(argv[2]) : 3.0;
        return runShow(seconds > 0 ? seconds : 3.0);
    }
    const int rc = runAutomated(argc > 1 ? argv[1] : nullptr);
    if (rc == 0) std::printf("all passed\n");
    return rc;
}
