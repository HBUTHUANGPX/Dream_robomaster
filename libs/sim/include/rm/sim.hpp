#pragma once

#include <mujoco/mujoco.h>

#include <filesystem>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <string>

namespace rm {
using Json = nlohmann::json;

class Simulation {
 public:
  explicit Simulation(const std::filesystem::path& path);
  ~Simulation();
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;
  mjModel* model = nullptr;
  mjData* data = nullptr;
};

// All methods and destruction must run on the thread that constructs Renderer.
class Renderer {
 public:
  explicit Renderer(mjModel* model, int width = 800, int height = 600);
  ~Renderer();
  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;
  cv::Mat render(mjData* data, const mjvCamera& camera, const mjvOption* option = nullptr);
  cv::Mat render_camera(mjData* data, const std::string& camera_name);
  void update(mjData* data, const mjvCamera& camera, const mjvOption* option = nullptr);
  cv::Mat read();
  mjvScene& scene();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

Json jpeg_response(const cv::Mat& rgb_or_gray, int quality = 85);
std::filesystem::path repo_root(int argc, char** argv);
int rpc_loop(const std::function<Json(const std::string&, const Json&)>& handler);
}  // namespace rm
