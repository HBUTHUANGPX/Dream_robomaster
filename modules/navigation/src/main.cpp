#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rm/navigation.hpp"
int main(int argc, char** argv) {
  try {
    auto root = rm::repo_root(argc, argv);
    bool rpc = false;
    double duration = 12, budget = 120;
    std::string localization = "prior", record;
    std::optional<rm::nav::V2> goal;
    double yaw = 0;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto next = [&]() {
        if (++i >= argc) throw std::invalid_argument("Missing argument");
        return std::string(argv[i]);
      };
      if (arg == "--rpc")
        rpc = true;
      else if (arg == "--root")
        next();
      else if (arg == "--duration")
        duration = rm::nav::parse_number(next());
      else if (arg == "--power-budget")
        budget = rm::nav::parse_number(next());
      else if (arg == "--localization")
        localization = next();
      else if (arg == "--record")
        record = next();
      else if (arg == "--goal") {
        double x = rm::nav::parse_number(next()), y = rm::nav::parse_number(next());
        goal = rm::nav::V2(x, y);
      } else if (arg == "--yaw")
        yaw = rm::nav::parse_number(next());
      else if (arg == "--headless") {
      } else if (arg == "--help") {
        std::cout
            << "导航程序用法：\n"
               "rm_navigation --root 仓库路径 [--rpc | --headless --duration 秒数 --goal X Y --yaw "
               "角速度] [--localization prior|slam] [--power-budget 瓦数] [--record raw.jsonl]\n"
               "目标坐标单位为 m。偏航角速度单位为 rad/s。\n"
               "prior 使用先验地图，slam 使用在线建图。网页入口为 ./rm navigation。\n";
        return 0;
      } else
        throw std::invalid_argument("Unknown option: " + arg);
    }
    if (!std::isfinite(duration) || duration <= 0 || duration > 86400)
      throw std::invalid_argument("Duration must be finite and positive");
    rm::nav::Task task(root, localization, budget);
    if (rpc)
      return rm::rpc_loop([&](const std::string& command, const rm::Json& args) {
        return task.handle(command, args);
      });
    if (goal) task.set_goal(goal->x(), goal->y());
    task.set_yaw_rate(yaw);
    std::ofstream out;
    if (!record.empty()) {
      out.open(record);
      if (!out) throw std::runtime_error("Cannot open recording");
      out << rm::Json(
                 {{"type", "metadata"},
                  {"imu_hz", 200},
                  {"lidar_hz", 10},
                  {"scan_model",
                   "instantaneous snapshot, MID-360-inspired reduced rays; no proprietary pattern"},
                  {"estimator", "gravity-aligned ICP; not FAST-LIO"},
                  {"orientation_input", false}})
                 .dump()
          << '\n';
    }
    auto emit = [&](const rm::Json& imu) {
      out << rm::Json({{"type", "imu"}, {"measurement", imu}}).dump() << '\n';
    };
    for (int i = 0; i < int(std::ceil(duration * 10)); ++i) {
      task.step(out.is_open() ? std::function<void(const rm::Json&)>(emit)
                              : std::function<void(const rm::Json&)>{});
      if (out.is_open())
        for (auto& packet : task.sensors.raw)
          out << rm::Json({{"type", "lidar"}, {"measurement", packet.json()}}).dump() << '\n';
      if (task.paused) break;
    }
    std::cout << task.state().dump() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
