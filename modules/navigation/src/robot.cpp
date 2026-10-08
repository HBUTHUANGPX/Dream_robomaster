#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rm/navigation.hpp"
int main(int argc, char** argv) {
  try {
    auto root = rm::repo_root(argc, argv);
    double duration = 12;
    bool viewer = false;
    std::optional<rm::nav::V3> requested;
    std::string snapshot;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto next = [&]() {
        if (++i >= argc) throw std::invalid_argument("Missing argument");
        return std::string(argv[i]);
      };
      if (arg == "--root")
        next();
      else if (arg == "--headless") {
      } else if (arg == "--viewer") {
        viewer = true;
      } else if (arg == "--duration")
        duration = rm::nav::parse_number(next());
      else if (arg == "--velocity") {
        double x = rm::nav::parse_number(next()), y = rm::nav::parse_number(next()),
               w = rm::nav::parse_number(next());
        requested = rm::nav::V3(x, y, w);
      } else if (arg == "--snapshot")
        snapshot = next();
      else
        throw std::invalid_argument("Unknown option: " + arg);
    }
    if (!std::isfinite(duration) || duration <= 0 || duration > 86400 ||
        (requested && !requested->allFinite()))
      throw std::invalid_argument("Nonfinite/invalid simulation arguments");
    rm::Simulation sim(root / "assets/robot.xml");
    rm::nav::Drive drive;
    auto m = sim.model;
    auto d = sim.data;
    for (int i = 0; i < int(.5 / m->opt.timestep); ++i) mj_step(m, d);
    double start = d->time;
    std::array<rm::nav::V3, 4> demo{rm::nav::V3(.4, 0, 0), rm::nav::V3(0, .4, 0),
                                    rm::nav::V3(0, 0, .7), rm::nav::V3::Zero()};
    if (viewer) rm::nav::robot_viewer(sim, requested, duration);
    for (int i = 0; !viewer && i < int(std::round(duration / m->opt.timestep)); ++i) {
      auto command = requested.value_or(demo[int((d->time - start) / 3) % 4]);
      auto speeds = drive.wheels(command);
      speeds /= std::max(1., speeds.cwiseAbs().maxCoeff() / 35);
      for (int j = 0; j < 4; ++j) d->ctrl[j] = speeds[j];
      mj_step(m, d);
    }
    if (!snapshot.empty()) {
      auto parent = std::filesystem::path(snapshot).parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
      rm::Renderer render(m, 1280, 960);
      mjvCamera camera;
      mjv_defaultCamera(&camera);
      for (int i = 0; i < 3; ++i) camera.lookat[i] = d->qpos[i] + (i == 2 ? .08 : 0);
      camera.distance = 1.25;
      camera.azimuth = 135;
      camera.elevation = -28;
      auto rgb = render.render(d, camera);
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      if (!cv::imwrite(snapshot, bgr)) throw std::runtime_error("Snapshot write failed");
    }
    double w = d->qpos[3], x = d->qpos[4], y = d->qpos[5], z = d->qpos[6], mass = 0;
    for (int i = 0; i < m->nbody; ++i) mass += m->body_mass[i];
    rm::Json warnings = rm::Json::array();
    for (int i = 0; i < mjNWARNING; ++i) warnings.push_back(d->warning[i].number);
    std::cout << rm::Json(
                     {{"time_s", d->time},
                      {"position_m", rm::nav::json(rm::nav::V3(Eigen::Map<rm::nav::V3>(d->qpos)))},
                      {"yaw_rad", std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))},
                      {"mass_kg", mass},
                      {"warnings", warnings}})
                     .dump()
              << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
