#include <chrono>
#include <fstream>
#include <iostream>

#include "rm/navigation.hpp"
int main(int argc, char** argv) {
  try {
    auto root = rm::repo_root(argc, argv);
    std::filesystem::path output = root / "output/navigation_validation_native.json";
    std::string mode = "prior";
    double seconds = 180, yaw = 0;
    std::vector<int> cases{0, 1, 2, 3, 4, 5, 6};
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto next = [&]() {
        if (++i >= argc) throw std::invalid_argument("Missing option argument");
        return std::string(argv[i]);
      };
      if (arg == "--root")
        next();
      else if (arg == "--output")
        output = next();
      else if (arg == "--seconds")
        seconds = rm::nav::parse_number(next());
      else if (arg == "--localization")
        mode = next();
      else if (arg == "--yaw-rate")
        yaw = rm::nav::parse_number(next());
      else if (arg == "--case") {
        double n = rm::nav::parse_number(next());
        if (n != std::floor(n) || n < 0 || n >= 7)
          throw std::invalid_argument("Invalid preset index");
        cases = {int(n)};
      } else
        throw std::invalid_argument("Unknown option: " + arg);
    }
    if (seconds <= 0 || seconds > 86400) throw std::invalid_argument("Invalid validation duration");
    rm::nav::Task task(root, mode);
    rm::Json results = rm::Json::array();
    bool good = true;
    for (int selected : cases) {
      task.reset();
      auto goal = task.presets()[selected];
      task.set_goal(goal["x"], goal["y"]);
      task.set_yaw_rate(yaw);
      double max_error = 0, max_z = 0, peak_speed = 0, squared_cross = 0;
      int in_place = 0, moving_spin = 0, steps = 0;
      std::vector<double> times;
      rm::Json trajectory = rm::Json::array();
      auto start = std::chrono::steady_clock::now();
      for (int i = 0; i < int(seconds * 10); ++i) {
        auto tick = std::chrono::steady_clock::now();
        task.step();
        times.push_back(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tick)
                .count());
        ++steps;
        auto d = task.simulation.data;
        rm::nav::V3 truth = Eigen::Map<rm::nav::V3>(d->qpos);
        max_error = std::max(max_error, (truth - task.mapper->pose.head<3>()).norm());
        max_z = std::max(max_z, truth.z() - .076);
        peak_speed = std::max(peak_speed, Eigen::Map<rm::nav::V2>(d->qvel).norm());
        double cross = INFINITY;
        for (auto p : task.path) cross = std::min(cross, (p.head<2>() - truth.head<2>()).norm());
        squared_cross += cross * cross;
        in_place += task.velocity.head<2>().norm() < .05 && std::abs(task.velocity.z()) > .1;
        moving_spin += task.velocity.head<2>().norm() > .15 && std::abs(task.velocity.z()) > .1;
        if (i % 10 == 0)
          trajectory.push_back({d->time, truth.x(), truth.y(), truth.z(), task.mapper->pose[0],
                                task.mapper->pose[1], task.mapper->pose[2], task.mapper->pose[3]});
        if (!task.goal || task.paused) break;
      }
      double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      std::sort(times.begin(), times.end());
      auto d = task.simulation.data;
      double error = (Eigen::Map<rm::nav::V2>(d->qpos) - rm::nav::V2(goal["x"], goal["y"])).norm();
      rm::Json warnings = rm::Json::array();
      bool warned = false;
      for (int i = 0; i < mjNWARNING; ++i) {
        warnings.push_back(d->warning[i].number);
        warned |= d->warning[i].number != 0;
      }
      rm::Json result = {{"case", selected},
                         {"goal", goal},
                         {"status", task.status},
                         {"sim_seconds", (steps - 1) * .1},
                         {"wall_seconds", wall},
                         {"truth_error_xy", error},
                         {"max_localization_error_m", max_error},
                         {"max_ground_height_m", max_z},
                         {"warnings", warnings},
                         {"trajectory", trajectory},
                         {"controller", "全向 DWA + PID"},
                         {"peak_truth_speed_m_s", peak_speed},
                         {"power_peak_w", task.power_peak_w},
                         {"power_budget_w", task.drive.budget},
                         {"energy_j", task.energy_j},
                         {"in_place_turn_seconds", in_place * .1},
                         {"moving_spin_seconds", moving_spin * .1},
                         {"cross_track_rmse_m", std::sqrt(squared_cross / steps)},
                         {"step_wall_p50_ms", times[times.size() / 2]},
                         {"step_wall_p95_ms", times[size_t(.95 * (times.size() - 1))]}};
      good &= task.status == "已到达" && error <= .2 && !warned &&
              task.power_peak_w <= task.drive.budget + 1e-6;
      results.push_back(result);
      result.erase("trajectory");
      std::cout << result.dump() << std::endl;
      if (!output.parent_path().empty()) std::filesystem::create_directories(output.parent_path());
      std::ofstream out(output);
      if (!out) throw std::runtime_error("Cannot write validation output");
      out << results.dump(2) << '\n';
    }
    return good ? 0 : 2;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
