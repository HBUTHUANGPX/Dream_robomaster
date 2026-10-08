#include <fstream>
#include <iostream>

#include "rm/rosbag.hpp"
int main(int argc, char** argv) {
  try {
    auto root = rm::repo_root(argc, argv);
    std::filesystem::path output = root / "output/lio_input.bag";
    double seconds = 100;
    int selected = 6;
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
      else if (arg == "--case") {
        double value = rm::nav::parse_number(next());
        if (value != std::floor(value) || value < 0 || value >= 7)
          throw std::invalid_argument("Invalid preset index");
        selected = int(value);
      } else
        throw std::invalid_argument("Unknown option: " + arg);
    }
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 86400 || selected < 0 || selected >= 7)
      throw std::invalid_argument("Invalid recording duration or preset index");
    rm::nav::Task task(root);
    auto goal = task.presets().at(selected);
    rm::nav::RawBag bag(output);
    rm::Json truth = rm::Json::array();
    mj_forward(task.simulation.model, task.simulation.data);
    bag.imu(task.sensors.imu(task.simulation.data));
    bag.cloud(task.sensors.raw[0]);
    for (int i = 0; i < int((seconds + 2) * 10); ++i) {
      if (i == 20) task.set_goal(goal["x"].get<double>(), goal["y"].get<double>());
      task.step([&](const rm::Json& imu) { bag.imu(imu); });
      bag.cloud(task.sensors.raw[0]);
      rm::Json row = rm::Json::array({task.simulation.data->time});
      for (int j = 0; j < 3; ++j)
        row.push_back(task.simulation.data->site_xpos[3 * task.sensors.imu_site + j]);
      for (int j = 0; j < 4; ++j)
        row.push_back(task.simulation.data->sensordata[task.sensors.attitude_address + j]);
      truth.push_back(row);
      if (i >= 20 && (!task.goal || task.paused)) break;
    }
    for (int i = 0; i < 5; ++i) mj_step(task.simulation.model, task.simulation.data);
    mj_forward(task.simulation.model, task.simulation.data);
    bag.imu(task.sensors.imu(task.simulation.data));
    bag.close();
    auto packet = task.sensors.raw[0].json();
    rm::Json metadata = {
        {"status", task.status},
        {"goal", goal},
        {"counts", {{"lidar", bag.count[0]}, {"imu", bag.count[1]}}},
        {"imu_hz", 200},
        {"lidar_hz", 10},
        {"scan_model",
         "instantaneous snapshot; all points share scan timestamp; FAST-LIO lidar_type=4"},
        {"sensors", "top LiDAR only; ideal raw accelerometer and gyro; no AHRS input"},
        {"extrinsic_T", packet["translation"]},
        {"extrinsic_R", packet["rotation"]},
        {"truth_columns", {"stamp", "imu_x", "imu_y", "imu_z", "qw", "qx", "qy", "qz"}},
        {"ground_truth", truth}};
    auto sidecar = output;
    sidecar.replace_extension(".json");
    std::ofstream out(sidecar);
    if (!out) throw std::runtime_error("Cannot create recording metadata");
    out << metadata.dump(2) << '\n';
    std::cout << rm::Json({{"output", output.string()},
                           {"status", task.status},
                           {"counts", metadata["counts"]}})
                     .dump()
              << '\n';
    return task.status == "已到达" ? 0 : 2;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
