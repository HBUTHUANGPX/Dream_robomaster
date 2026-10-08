#include <algorithm>

#include "rm/navigation.hpp"
namespace rm::nav {
Task::Task(const std::filesystem::path& r, std::string mode, double budget)
    : root(r),
      localization(std::move(mode)),
      terrain(r / "assets/arena/rmuc2023.stl"),
      simulation(r / "assets/navigation.xml"),
      sensors(simulation.model),
      drive(simulation.model->body_subtreemass[sensors.chassis], budget),
      planner(terrain, drive) {
  if (localization != "prior" && localization != "slam")
    throw std::invalid_argument("Unknown localization mode");
  auto triangles = load_stl(root / "assets/arena/rmuc2023.stl");
  std::vector<double> area;
  std::vector<Triangle> visible;
  double sum = 0;
  for (auto t : triangles) {
    V3 center = (t.v[0] + t.v[1] + t.v[2]) / 3;
    if (t.normal.z() > -.1 || center.z() > .3) {
      sum += (t.v[1] - t.v[0]).cross(t.v[2] - t.v[0]).norm() / 2;
      area.push_back(sum);
      visible.push_back(t);
    }
  }
  std::mt19937 rng(36);
  std::uniform_real_distribution<double> uniform(0, 1);
  prior.reserve(160000);
  prior_normals.reserve(160000);
  for (int i = 0; i < 160000; ++i) {
    size_t index = std::lower_bound(area.begin(), area.end(), uniform(rng) * sum) - area.begin();
    auto& t = visible[index];
    double a = uniform(rng), b = uniform(rng);
    if (a + b > 1) {
      a = 1 - a;
      b = 1 - b;
    }
    prior.push_back(t.v[0] + a * (t.v[1] - t.v[0]) + b * (t.v[2] - t.v[0]));
    prior_normals.push_back(t.normal);
  }
  reset();
}
void Task::reset() {
  auto m = simulation.model;
  auto d = simulation.data;
  mj_resetData(m, d);
  for (int i = 0; i < m->nu; ++i) {
    m->actuator_forcerange[2 * i] = -10;
    m->actuator_forcerange[2 * i + 1] = 10;
    m->actuator_biasprm[mjNBIAS * i] = 0;
  }
  for (int i = 0; i < 500; ++i) mj_step(m, d);
  mapper = std::make_unique<Mapper>(V4(terrain.spawn.x(), terrain.spawn.y(), .176, 0),
                                    localization == "prior" ? prior : Points{},
                                    localization == "prior" ? prior_normals : Points{});
  path.clear();
  local.clear();
  trail.clear();
  scan.clear();
  goal.reset();
  reference.reset();
  status = "请选择绿色可达区域";
  paused = false;
  velocity.setZero();
  measured_velocity.setZero();
  index = 0;
  stuck_time = 0;
  last_position = mapper->pose.head<2>();
  localization_failures = 0;
  slip_seconds = 0;
  yaw_rate_request = power_w = power_peak_w = energy_j = mechanical_w = 0;
  tracker.reset();
  for (auto& p : speed_feedback) p.reset();
  planner.candidates = planner.admissible = 0;
  sense();
}
void Task::sense() {
  auto [cloud, odom] = sensors.sample(simulation.data, .1);
  V4 previous = mapper->pose;
  mapper->update(cloud, odom);
  M3 r = rotation(mapper->pose[3]);
  V3 lidar_velocity = r.transpose() * (mapper->pose.head<3>() - previous.head<3>()) / .1;
  measured_velocity.head<2>() = .5 * measured_velocity.head<2>() + .5 * lidar_velocity.head<2>();
  measured_velocity.z() = sensors.velocity.z();
  double slip = (sensors.velocity - measured_velocity).head<2>().norm();
  slip_seconds = slip > .25 && velocity.head<2>().norm() > .05 ? slip_seconds + .1 : 0;
  scan.clear();
  for (auto p : cloud) scan.push_back(r * p + mapper->pose.head<3>());
  trail.push_back(mapper->pose.head<3>());
  if (trail.size() > 3000) trail.erase(trail.begin());
  localization_failures = mapper->matched < 30 ? localization_failures + 1 : 0;
}
void Task::set_goal(double x, double y) {
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 || x >= terrain.width ||
      y >= terrain.height)
    throw std::invalid_argument("目标超出地图");
  auto newpath = terrain.plan(mapper->pose.head<3>(), V3(x, y, terrain.height_at(x, y)));
  goal = terrain.point(terrain.cell(x, y));
  path = std::move(newpath);
  index = 0;
  stuck_time = simulation.data->time;
  last_position = mapper->pose.head<2>();
  status = "导航中";
  tracker.reset();
  reference.reset();
  for (auto& p : speed_feedback) p.reset();
}
void Task::set_yaw_rate(double rate) {
  if (!std::isfinite(rate) || std::abs(rate) > 1.2)
    throw std::invalid_argument("Yaw rate must be within ±1.2 rad/s");
  yaw_rate_request = rate;
}
V3 Task::stop() {
  tracker.reset();
  reference.reset();
  local.clear();
  for (auto& p : speed_feedback) p.reset();
  return V3::Zero();
}
V3 Task::control() {
  if (paused || !goal) return stop();
  auto pose = mapper->pose;
  if (localization_failures >= 8) {
    status = "定位失效：停车等待有效点云";
    return stop();
  }
  status = "导航中";
  double remaining = (goal->head<2>() - pose.head<2>()).norm();
  if (remaining < .13 && std::abs(pose[2] - .076 - goal->z()) < .12 &&
      measured_velocity.head<2>().norm() < .25) {
    status = "已到达";
    goal.reset();
    return stop();
  }
  int end = std::min(int(path.size()), index + 20), nearest = index;
  double best = INFINITY;
  for (int i = index; i < end; ++i) {
    double distance = (path[i].head<2>() - pose.head<2>()).norm();
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }
  index = nearest;
  int look = index;
  while (look < int(path.size()) - 1 && (path[look].head<2>() - pose.head<2>()).norm() < .4) ++look;
  V3 target = path[look];
  double slope = std::abs(target.z() - (pose[2] - .076)) /
                 std::max((target.head<2>() - pose.head<2>()).norm(), .2);
  double speed = std::min(slope < .1 ? 1.2 : .55, std::max(.08, remaining * 1.5)),
         yaw = yaw_rate_request;
  int ahead = index;
  while (ahead < int(path.size()) - 1 && (path[ahead].head<2>() - pose.head<2>()).norm() < 1.)
    ++ahead;
  double rise = std::abs(path[ahead].z() - (pose[2] - .076));
  if (std::abs(yaw) < 1e-6 && (rise > .06 || slip_seconds > .5)) {
    V2 direction = path[ahead].head<2>() - pose.head<2>();
    double error = wrap(std::atan2(direction.y(), direction.x()) - pose[3]);
    error = std::fmod(error + pi / 2 + 2 * pi, pi) - pi / 2;
    double moving = std::max(measured_velocity.head<2>().norm(), velocity.head<2>().norm());
    yaw = std::clamp(error, -.6, .6) * std::min(1., moving / .15);
  }
  planner.gravity = 9.81 * sensors.attitude.row(2).head<2>().transpose();
  if ((pose.head<2>() - last_position).norm() > .12) {
    last_position = pose.head<2>();
    stuck_time = simulation.data->time;
  } else if (simulation.data->time - stuck_time > 15) {
    status = "无进展：已停车，请重新选择目标";
    goal.reset();
    return stop();
  }
  Points obstacles;
  for (auto p : scan) {
    double ground = terrain.height_at(p.x(), p.y());
    if (p.z() > ground + .16 && p.z() < pose[2] + .6) obstacles.emplace_back(p.x(), p.y(), 0);
  }
  Points remaining_path(path.begin() + index, path.end());
  auto [nominal, trajectory] =
      planner.plan(pose, measured_velocity, remaining_path, *goal, obstacles, speed, yaw);
  if (trajectory.empty()) {
    status = "局部障碍或制动距离不足：停车";
    return stop();
  }
  V3 ref = reference.value_or(V3(pose[0], pose[1], pose[3]));
  V3 proposed = tracker.update(pose, ref, nominal, .1);
  auto [command, accepted] =
      planner.constrain(pose, measured_velocity, proposed, nominal, obstacles, speed);
  if (!accepted)
    tracker.reset();
  else
    tracker.accept(proposed, command);
  auto [rollout, normal] = planner.rollout(pose, command);
  reference = rollout[1];
  local.clear();
  for (int i = 1; i <= normal; ++i)
    local.emplace_back(rollout[i].x(), rollout[i].y(),
                       terrain.height_at(rollout[i].x(), rollout[i].y()) + .025);
  return command;
}
void Task::step(const std::function<void(const Json&)>& callback) {
  auto m = simulation.model;
  auto d = simulation.data;
  auto fault = [&] {
    bool warning = false;
    for (int i = 0; i < mjNWARNING; ++i) warning |= d->warning[i].number != 0;
    return warning || sensors.attitude(2, 2) < .7;
  };
  auto halt = [&] {
    mju_zero(d->ctrl, m->nu);
    velocity.setZero();
    local.clear();
    paused = true;
    status = "姿态或物理异常：请重置仿真";
  };
  if (paused) {
    mju_zero(d->ctrl, m->nu);
    velocity = stop();
    return;
  }
  if (fault()) {
    halt();
    return;
  }
  velocity = control();
  V3 motor = velocity;
  if (goal && velocity.norm() > .01)
    for (int i = 0; i < 3; ++i)
      motor[i] += speed_feedback[i].update(velocity[i] - measured_velocity[i], .1);
  V4 speeds = drive.wheels(motor);
  speeds /= std::max(1., speeds.cwiseAbs().maxCoeff() / 35.);
  for (int i = 0; i < 4; ++i) d->ctrl[i] = speeds[i];
  V3 force(drive.mass * 9.81 * sensors.attitude(2, 0), drive.mass * 9.81 * sensors.attitude(2, 1),
           0);
  V4 load = drive.force_to_torque * force;
  for (int i = 0; i < 4; ++i) m->actuator_biasprm[i * mjNBIAS] = load[i];
  double power_sum = 0, mechanical_sum = 0;
  int steps = int(std::round(.1 / m->opt.timestep));
  for (int i = 0; i < steps; ++i) {
    V4 omega;
    for (int j = 0; j < 4; ++j) omega[j] = d->qvel[sensors.dofs[j]];
    V4 torque = drive.limit_torque(4 * (speeds - omega) + load, omega);
    for (int j = 0; j < 4; ++j) {
      m->actuator_forcerange[2 * j] = -std::abs(torque[j]);
      m->actuator_forcerange[2 * j + 1] = std::abs(torque[j]);
    }
    mj_step(m, d);
    V4 actual = Eigen::Map<V4>(d->actuator_force);
    double power = drive.electrical(actual, omega);
    power_sum += power;
    mechanical_sum += actual.dot(omega);
    power_peak_w = std::max(power_peak_w, power);
    energy_j += power * m->opt.timestep;
    if (callback && (i + 1) % 5 == 0) {
      mj_forward(m, d);
      callback(sensors.imu(d));
    }
  }
  power_w = power_sum / steps;
  mechanical_w = mechanical_sum / steps;
  sense();
  if (fault()) halt();
}
Json Task::state() const {
  double error = (mapper->pose.head<3>() - Eigen::Map<V3>(simulation.data->qpos)).norm();
  return {{"time", std::round(simulation.data->time * 100) / 100},
          {"status", status},
          {"pose", json(mapper->pose)},
          {"goal", goal ? json(*goal) : Json(nullptr)},
          {"global_path", json(path)},
          {"local_path", json(local)},
          {"trail", json(trail, std::max(size_t(1), trail.size() / 800))},
          {"cloud", json(scan, std::max(size_t(1), scan.size() / 1500))},
          {"slam_points", mapper->map_points.size()},
          {"scan_points", scan.size()},
          {"localization_error", error},
          {"icp_rmse", std::isfinite(mapper->rmse) ? Json(mapper->rmse) : Json(nullptr)},
          {"speed", measured_velocity.head<2>().norm()},
          {"paused", paused},
          {"progress", double(index) / std::max(1, int(path.size()) - 1)},
          {"controller", "全向 DWA + PID"},
          {"dwa_candidates", planner.candidates},
          {"dwa_admissible", planner.admissible},
          {"tracking_error", json(tracker.error)},
          {"yaw_rate_request", yaw_rate_request},
          {"body_velocity", json(velocity)},
          {"power_w", power_w},
          {"power_peak_w", power_peak_w},
          {"power_budget_w", drive.budget},
          {"energy_j", energy_j},
          {"mechanical_power_w", mechanical_w}};
}
Json Task::presets() const {
  Json result = Json::array();
  const char* names[] = {"平地",     "红方高地", "红方公路高地", "中央起伏区",
                         "蓝方高地", "跨场巡航", "坡道停驻"};
  std::array<V2, 7> targets{V2(7.5, 7), V2(2, 6),    V2(6.8, 9.5), V2(8.8, 14),
                            V2(13, 22), V2(7.5, 24), V2(9.2, 10.3)};
  for (int i = 0; i < 7; ++i) {
    int best = -1;
    double distance = INFINITY;
    for (int cell = 0; cell < terrain.rows * terrain.cols; ++cell)
      if (terrain.reachable[cell]) {
        double d = (terrain.point(cell).head<2>() - targets[i]).squaredNorm();
        if (d < distance) {
          distance = d;
          best = cell;
        }
      }
    if (best >= 0) {
      auto p = terrain.point(best);
      result.push_back({{"name", names[i]}, {"x", p.x()}, {"y", p.y()}, {"z", p.z()}});
    }
  }
  return result;
}
Json Task::map() const {
  auto heights = terrain.heights;
  for (auto& h : heights)
    if (!std::isfinite(h)) h = -1;
  return {{"localization", localization}, {"width", terrain.width},
          {"height", terrain.height},     {"resolution", res},
          {"cols", terrain.cols},         {"rows", terrain.rows},
          {"heights", heights},           {"reachable", terrain.reachable},
          {"presets", presets()},         {"spawn", json(terrain.spawn)}};
}
Json Task::frame() {
  if (!renderer) renderer = std::make_unique<Renderer>(simulation.model, 960, 600);
  mjvCamera camera;
  mjv_defaultCamera(&camera);
  camera.distance = 2.4;
  camera.elevation = -38;
  camera.azimuth = 135;
  for (int i = 0; i < 3; ++i) camera.lookat[i] = mapper->pose[i] + (i == 2 ? .1 : 0);
  mjvOption option;
  mjv_defaultOption(&option);
  option.geomgroup[3] = 0;
  renderer->update(simulation.data, camera, &option);
  auto& scene = renderer->scene();
  for (int which = 0; which < 2; ++which) {
    auto& points = which ? local : path;
    float color[4] = {which ? 1.f : .1f, which ? .5f : .85f, which ? .1f : 1.f, 1};
    for (size_t i = 0; i + 2 < points.size() && scene.ngeom < scene.maxgeom; i += 2) {
      auto& geom = scene.geoms[scene.ngeom++];
      mjtNum zero[3] = {0, 0, 0}, identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      mjv_initGeom(&geom, mjGEOM_CAPSULE, zero, zero, identity, color);
      V3 a = points[i] + V3(0, 0, .06), b = points[i + 2] + V3(0, 0, .06);
      mjv_connector(&geom, mjGEOM_CAPSULE, which ? .032 : .018, a.data(), b.data());
    }
  }
  return jpeg_response(renderer->read());
}
Json Task::handle(const std::string& command, const Json& args) {
  if (command == "state") return state();
  if (command == "map") return map();
  if (command == "tick") {
    step();
    return state();
  }
  if (command == "frame") {
    if (args.contains("view") && args["view"] != "scene")
      throw std::invalid_argument("Unknown view");
    return frame();
  }
  auto number = [&](const char* key) {
    if (!args.contains(key) || !args[key].is_number())
      throw std::invalid_argument(std::string("Numeric field required: ") + key);
    double n = args[key].get<double>();
    if (!std::isfinite(n)) throw std::invalid_argument("Nonfinite command");
    return n;
  };
  if (command == "reset")
    reset();
  else if (command == "pause")
    paused = !paused;
  else if (command == "goal") {
    double x = number("x"), y = number("y");
    set_goal(x, y);
  } else if (command == "yaw")
    set_yaw_rate(number("rate"));
  else
    throw std::invalid_argument("Unknown navigation command");
  return {{"ok", true}, {"message", status}};
}
}  // namespace rm::nav
