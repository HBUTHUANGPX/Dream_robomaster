#include "rm/sim.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <array>
#include <cstdlib>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

namespace rm {
Simulation::Simulation(const std::filesystem::path& path) {
  std::array<char, 4096> error{};
  model = mj_loadXML(path.c_str(), nullptr, error.data(), static_cast<int>(error.size()));
  if (!model) throw std::runtime_error("Cannot load " + path.string() + ": " + error.data());
  data = mj_makeData(model);
  if (!data) {
    mj_deleteModel(model);
    model = nullptr;
    throw std::runtime_error("MuJoCo data allocation failed");
  }
  mj_forward(model, data);
}
Simulation::~Simulation() {
  if (data) mj_deleteData(data);
  if (model) mj_deleteModel(model);
}

namespace {
EGLDisplay display_for_device() {
  auto query = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
  auto platform = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
      eglGetProcAddress("eglGetPlatformDisplayEXT"));
  if (query && platform) {
    std::array<EGLDeviceEXT, 32> devices{};
    EGLint count = 0;
    if (query(static_cast<EGLint>(devices.size()), devices.data(), &count)) {
      const char* requested = std::getenv("MUJOCO_EGL_DEVICE_ID");
      int selected = -1;
      if (requested) {
        std::string value(requested);
        std::size_t consumed = 0;
        selected = std::stoi(value, &consumed);
        if (consumed != value.size() || selected < 0 || selected >= count)
          throw std::runtime_error("MUJOCO_EGL_DEVICE_ID is outside available EGL devices");
      }
      for (int i = 0; i < count; ++i) {
        if (selected >= 0 && i != selected) continue;
        auto display = platform(EGL_PLATFORM_DEVICE_EXT, devices[i], nullptr);
        if (display != EGL_NO_DISPLAY && eglInitialize(display, nullptr, nullptr)) return display;
      }
    }
    if (!std::getenv("MUJOCO_EGL_DEVICE_ID")) {
      auto display = platform(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
      if (display != EGL_NO_DISPLAY && eglInitialize(display, nullptr, nullptr)) return display;
    }
  }
  throw std::runtime_error("No EGL device/surfaceless display available; install an EGL driver");
}

std::string base64(const std::vector<unsigned char>& bytes) {
  constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve((bytes.size() + 2) / 3 * 4);
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    unsigned n = static_cast<unsigned>(bytes[i]) << 16;
    if (i + 1 < bytes.size()) n |= static_cast<unsigned>(bytes[i + 1]) << 8;
    if (i + 2 < bytes.size()) n |= bytes[i + 2];
    encoded += alphabet[(n >> 18) & 63];
    encoded += alphabet[(n >> 12) & 63];
    encoded += i + 1 < bytes.size() ? alphabet[(n >> 6) & 63] : '=';
    encoded += i + 2 < bytes.size() ? alphabet[n & 63] : '=';
  }
  return encoded;
}
}  // namespace

struct Renderer::Impl {
  mjModel* model;
  int width, height;
  EGLDisplay display = EGL_NO_DISPLAY;
  EGLContext context = EGL_NO_CONTEXT;
  mjvScene scene{};
  mjrContext render_context{};
  bool scene_ready = false, context_ready = false;
  Impl(mjModel* model_, int width_, int height_) : model(model_), width(width_), height(height_) {}
  void current() {
    if (!eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context))
      throw std::runtime_error("Cannot make EGL context current");
  }
  ~Impl() {
    if (context != EGL_NO_CONTEXT) {
      eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context);
      if (context_ready) mjr_freeContext(&render_context);
      if (scene_ready) mjv_freeScene(&scene);
      eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
      eglDestroyContext(display, context);
    }
    // EGLDisplay is shared by contexts on the same GPU; do not terminate it
    // here and invalidate another live renderer. Process exit releases it.
  }
};

Renderer::Renderer(mjModel* model, int width, int height)
    : impl_(std::make_unique<Impl>(model, width, height)) {
  if (!model || width <= 0 || height <= 0 || width > 4096 || height > 4096)
    throw std::invalid_argument("Invalid renderer model or dimensions");
  auto& p = *impl_;
  p.display = display_for_device();
  const EGLint attributes[] = {EGL_RED_SIZE,
                               8,
                               EGL_GREEN_SIZE,
                               8,
                               EGL_BLUE_SIZE,
                               8,
                               EGL_ALPHA_SIZE,
                               8,
                               EGL_DEPTH_SIZE,
                               24,
                               EGL_STENCIL_SIZE,
                               8,
                               EGL_SURFACE_TYPE,
                               EGL_PBUFFER_BIT,
                               EGL_RENDERABLE_TYPE,
                               EGL_OPENGL_BIT,
                               EGL_NONE};
  EGLConfig config{};
  EGLint count = 0;
  if (!eglChooseConfig(p.display, attributes, &config, 1, &count) || count == 0 ||
      !eglBindAPI(EGL_OPENGL_API))
    throw std::runtime_error("Cannot configure EGL OpenGL framebuffer");
  p.context = eglCreateContext(p.display, config, EGL_NO_CONTEXT, nullptr);
  if (p.context == EGL_NO_CONTEXT) throw std::runtime_error("Cannot create EGL OpenGL context");
  p.current();
  mjv_defaultScene(&p.scene);
  mjr_defaultContext(&p.render_context);
  mjv_makeScene(model, &p.scene, 10000);
  p.scene_ready = true;
  mjr_makeContext(model, &p.render_context, mjFONTSCALE_100);
  p.context_ready = true;
  mjr_resizeOffscreen(width, height, &p.render_context);
  mjr_setBuffer(mjFB_OFFSCREEN, &p.render_context);
  if (p.render_context.currentBuffer != mjFB_OFFSCREEN)
    throw std::runtime_error("MuJoCo offscreen framebuffer unavailable");
}
Renderer::~Renderer() = default;
void Renderer::update(mjData* data, const mjvCamera& camera, const mjvOption* option) {
  auto& p = *impl_;
  p.current();
  mjvOption defaults;
  if (!option) {
    mjv_defaultOption(&defaults);
    option = &defaults;
  }
  auto mutable_camera = camera;
  mjv_updateScene(p.model, data, option, nullptr, &mutable_camera, mjCAT_ALL, &p.scene);
}
cv::Mat Renderer::read() {
  auto& p = *impl_;
  p.current();
  mjrRect viewport{0, 0, p.width, p.height};
  mjr_render(viewport, &p.scene, &p.render_context);
  cv::Mat image(p.height, p.width, CV_8UC3);
  mjr_readPixels(image.data, nullptr, viewport, &p.render_context);
  cv::flip(image, image, 0);
  return image;
}
cv::Mat Renderer::render(mjData* data, const mjvCamera& camera, const mjvOption* option) {
  update(data, camera, option);
  return read();
}
cv::Mat Renderer::render_camera(mjData* data, const std::string& name) {
  mjvCamera camera;
  mjv_defaultCamera(&camera);
  camera.type = mjCAMERA_FIXED;
  camera.fixedcamid = mj_name2id(impl_->model, mjOBJ_CAMERA, name.c_str());
  if (camera.fixedcamid < 0) throw std::invalid_argument("Unknown camera: " + name);
  return render(data, camera);
}
mjvScene& Renderer::scene() { return impl_->scene; }

Json jpeg_response(const cv::Mat& image, int quality) {
  if (image.empty() || image.depth() != CV_8U || (image.channels() != 1 && image.channels() != 3))
    throw std::invalid_argument("JPEG expects nonempty 8-bit RGB or grayscale image");
  cv::Mat bgr;
  if (image.channels() == 3)
    cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
  else
    bgr = image;
  std::vector<unsigned char> bytes;
  if (!cv::imencode(".jpg", bgr, bytes, {cv::IMWRITE_JPEG_QUALITY, quality}))
    throw std::runtime_error("JPEG encoding failed");
  return Json{{"mime", "image/jpeg"}, {"data", base64(bytes)}};
}

std::filesystem::path repo_root(int argc, char** argv) {
  std::filesystem::path root = RM_SOURCE_ROOT;
  if (const char* environment = std::getenv("ROBOMASTER_ROOT")) root = environment;
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--root") {
      if (i + 1 >= argc) throw std::invalid_argument("--root requires a directory");
      root = argv[++i];
    }
  root = std::filesystem::absolute(root).lexically_normal();
  if (!std::filesystem::is_directory(root / "assets"))
    throw std::invalid_argument("Root has no assets directory: " + root.string());
  return root;
}

int rpc_loop(const std::function<Json(const std::string&, const Json&)>& handler) {
  std::string line;
  while (std::getline(std::cin, line)) {
    Json reply;
    try {
      if (line.size() > 65536) throw std::invalid_argument("RPC request exceeds 64 KiB");
      auto request = Json::parse(line);
      if (!request.is_object() || !request.contains("command") || !request["command"].is_string())
        throw std::invalid_argument("RPC request requires a string command");
      auto args = request.value("args", Json::object());
      if (!args.is_object()) throw std::invalid_argument("RPC args must be an object");
      auto result = handler(request.at("command").get<std::string>(), args);
      reply = {{"ok", true}, {"result", std::move(result)}};
    } catch (const std::exception& error) {
      reply = {{"ok", false}, {"error", error.what()}};
    }
    std::cout << reply.dump() << '\n' << std::flush;
  }
  return std::cin.bad() ? 1 : 0;
}
}  // namespace rm
