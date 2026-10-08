#include <iostream>
#include <stdexcept>

#include "rm/navigation.hpp"
using namespace rm::nav;
void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
Points line(V3 a, V3 b, int n = 51) {
  Points p;
  for (int i = 0; i < n; ++i) p.push_back(a + (b - a) * double(i) / (n - 1));
  return p;
}
void algorithms(const std::filesystem::path& root) {
  PID pid(1, .5, .1, .4);
  for (int i = 0; i < 200; ++i) check(std::abs(pid.update(10, .1)) <= .4, "PID saturation");
  check(std::abs(pid.integral) < .01, "PID windup");
  Tracker tracker;
  V4 heading_pose(2, 2, .176, pi - .01);
  auto tracked = tracker.update(heading_pose, V3(1.95, 2, -pi + .01), V3(.2, 0, .1), .1);
  check(tracked.x() > .2 && tracked.z() > .1 && tracked.z() < .3 && std::abs(tracked.y()) < .002,
        "Body-frame PID or wrapped heading");
  tracker.reset();
  for (int i = 0; i < 100; ++i) {
    auto proposed = tracker.update(V4(2, 2, .176, 0), V3(2.02, 2, .01), V3(.3, 0, .2), .1);
    tracker.accept(proposed, V3(.2, 0, 0));
  }
  check(tracker.pid[0].integral == 0 && tracker.pid[2].integral == 0,
        "Tracker downstream saturation windup");
  Drive drive;
  for (V4 omega : {V4::Constant(25).eval(), V4::Zero().eval(), V4(-20, 20, 20, -20)})
    check(drive.electrical(drive.limit_torque(V4::Constant(10), omega), omega) <= 120 + 1e-8,
          "Power budget exceeded");
  check(drive.predict(V3(0, 1, 0), V3(0, 1, 0), .1).power >
            drive.predict(V3(1, 0, 0), V3(1, 0, 0), .1).power,
        "Directional rolling power");
  Terrain flat(200, 200);
  DWA dwa(flat, drive);
  V4 pose(5, 5, .176, 0);
  for (V3 goal : {V3(5, 8, .1), V3(2, 5, .1)}) {
    auto [cmd, tr] = dwa.plan(pose, V3::Zero(), line(V3(5, 5, .1), goal), goal, {});
    check((goal.y() > 5 ? cmd.y() : -cmd.x()) > .02 && std::abs(cmd.z()) < .05 && !tr.empty(),
          "Omni direction");
  }
  auto [cmd, tr] =
      dwa.plan(pose, V3::Zero(), line(V3(5, 5, .1), V3(8, 5, .1)), V3(8, 5, .1), {}, 1.2, .6);
  check(cmd.x() > .02 && cmd.z() > .02, "Concurrent yaw and translation");
  DWA short_horizon(flat, drive, .2);
  check(!short_horizon.safe(pose, V3(0, 1, 0), {V3(5, 5.85, 0)}), "Lateral braking tail");
  check(!dwa.safe(V4(19.99, 5, .176, 0), V3(.4, 0, 0), {}), "Map boundary");
  Terrain edge_terrain(100, 100);
  DWA edge_planner(edge_terrain, drive);
  edge_terrain.valid[20 * 100 + 19] = 0;
  check(!edge_planner.safe(V4(1.98, 1.99, .176, pi / 4), V3(.45, 0, 0), {}),
        "Swept diagonal crossed invalid corner");
  std::fill(edge_terrain.valid.begin(), edge_terrain.valid.end(), 1);
  std::fill(edge_terrain.valid.begin(), edge_terrain.valid.begin() + 2000, 0);
  check(!edge_planner.safe(V4(2, 2.06, .176, -pi / 2), V3(.035, 0, 0), {}),
        "Tracking clearance margin violated");
  check(edge_planner.safe(V4(2, 2.01, .176, pi / 2), V3(.035, 0, 0), {}),
        "Escape away from nearby edge blocked");
  V4 detour_pose(2, 2, .176, 0);
  V3 detour_velocity = V3::Zero();
  auto detour_path = line(V3(2, 2, .1), V3(5, 2, .1), 31);
  Points obstacles{V3(3, 2.1, 0)};
  for (int iteration = 0; iteration < 240; ++iteration) {
    auto nearest =
        std::min_element(detour_path.begin(), detour_path.end(), [&](const V3& a, const V3& b) {
          return (a.head<2>() - detour_pose.head<2>()).norm() <
                 (b.head<2>() - detour_pose.head<2>()).norm();
        });
    auto [command, trajectory] =
        dwa.plan(detour_pose, detour_velocity, Points(nearest, detour_path.end()),
                 detour_path.back(), obstacles);
    check(!trajectory.empty(), "Obstacle detour stalled");
    auto step = dwa.rollout(detour_pose, command).first[1];
    detour_pose[0] = step.x();
    detour_pose[1] = step.y();
    detour_pose[3] = step.z();
    detour_velocity = command;
    check((detour_pose.head<2>() - obstacles[0].head<2>()).norm() > .4,
          "Obstacle detour entered footprint");
    if ((detour_pose.head<2>() - detour_path.back().head<2>()).norm() < .13) break;
  }
  check((detour_pose.head<2>() - detour_path.back().head<2>()).norm() < .13,
        "Obstacle detour failed to reach goal");
  for (V3 current : {V3(1.6, .3, 1.5), V3(-.3, -1.6, -1.5)}) {
    auto [lo, hi] = dwa.window(current, .45);
    check((hi - lo).minCoeff() >= 0 && (lo - current).head<2>().cwiseAbs().maxCoeff() <= .1600001 &&
              (hi - current).head<2>().cwiseAbs().maxCoeff() <= .1600001,
          "Overspeed window");
  }
  std::mt19937 rng(17);
  std::uniform_real_distribution<double> u(-3, 3);
  Points prior;
  for (int i = 0; i < 2000; ++i) {
    V3 p(u(rng), u(rng), u(rng));
    p[i % 3] = (i % 3 == 0 ? -3.7 : (i % 3 == 1 ? 3.2 : -.8));
    prior.push_back(p);
  }
  V4 actual(.5, -.3, .4, .3);
  Points scan;
  for (int i = 0; i < 1700; ++i)
    scan.push_back(rotation(-actual[3]) * (prior[i] - actual.head<3>()));
  Mapper m(actual + V4(.18, -.14, .1, .045), prior);
  m.update(scan, V4::Zero());
  check((m.pose - actual).norm() < .04 && m.matched > 1000, "ICP transform recovery");
  Mapper online(V4::Zero());
  online.update(prior, V4::Zero());
  actual = V4(.15, .1, .03, .025);
  scan.clear();
  for (auto p : prior) scan.push_back(rotation(-actual[3]) * (p - actual.head<3>()));
  online.update(scan, actual + V4(.1, -.08, .06, .02));
  check((online.pose - actual).norm() < .04, "Online ICP map");
  auto count = online.map_points.size();
  for (auto& p : scan) p += V3::Constant(100);
  online.update(scan, V4::Zero());
  check(online.map_points.size() == count && online.matched == 0, "Rejected scan polluted map");
  Mapper sparse(V4(1, 2, 3, pi / 2), prior);
  sparse.update({}, V4(.3, .1, .2, .1));
  check((sparse.pose - V4(.9, 2.3, 3.2, pi / 2 + .1)).norm() < 1e-10,
        "Sparse odometry propagation");
  auto before_invalid = sparse.pose;
  bool bad_odometry = false;
  try {
    sparse.update(prior, V4(NAN, 0, 0, 0));
  } catch (const std::invalid_argument&) {
    bad_odometry = true;
  }
  check(bad_odometry && sparse.pose == before_invalid, "Invalid odometry changed estimator state");
  Mapper huge(V4::Zero());
  huge.update({V3(1e300, 0, 0)}, V4::Zero());
  check(huge.map_points.empty(), "Unrepresentable voxel coordinate accepted");
  Points plane_prior, plane_normals, plane_scan;
  for (int i = 0; i < 12000; ++i) {
    plane_prior.emplace_back(u(rng), u(rng), 0);
    plane_normals.push_back(V3::UnitZ());
    if (i % 2 == 0) plane_scan.emplace_back(u(rng), u(rng), 0);
  }
  for (int axis = 0; axis < 2; ++axis)
    for (int i = 0; i < 500; ++i) {
      V3 p(u(rng), u(rng), 1.1 + .3 * u(rng));
      p[axis] = 3;
      plane_prior.push_back(p);
      plane_normals.push_back(axis == 0 ? V3::UnitX() : V3::UnitY());
      if (i % 2 == 0) {
        p = V3(u(rng), u(rng), 1.1 + .3 * u(rng));
        p[axis] = 3;
        plane_scan.push_back(p);
      }
    }
  Mapper planes(V4::Zero(), plane_prior, plane_normals);
  for (int i = 0; i < 5; ++i) {
    planes.update(plane_scan, V4(.12, -.08, 0, .01));
    check(planes.pose.norm() < .02 && planes.matched > 30, "Dense floor hid wheel slip");
  }
  Terrain terrain(root / "assets/arena/rmuc2023.stl");
  for (V3 goal : {V3(1.95, 5.95, 0), V3(6.75, 9.45, 0), V3(8.85, 13.85, 0), V3(12.95, 21.95, 0),
                  V3(7.45, 23.95, 0)}) {
    auto path = terrain.plan(terrain.spawn, goal);
    check(!path.empty(), "CAD route unavailable");
    for (size_t i = 1; i < path.size(); ++i)
      check(std::abs(path[i].z() - path[i - 1].z()) < .085, "Unsafe height edge");
  }
  bool rejected = false;
  try {
    terrain.plan(terrain.spawn, V3(1.2, 1.35, 0));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  check(rejected, "Isolated roof accepted");
}
void physics(const std::filesystem::path& root) {
  rm::Simulation incomplete(root / "assets/robot.xml");
  bool missing_mount = false;
  try {
    Sensors invalid(incomplete.model);
  } catch (const std::runtime_error&) {
    missing_mount = true;
  }
  check(missing_mount, "Missing LiDAR mounting site accepted");
  Task task(root);
  check(task.scan.size() > 700 && task.mapper->matched > 30, "Dual lidar / prior registration");
  check((task.sensors.raw[0].translation - V3(0, 0, .438)).norm() < 1e-6, "Top lidar extrinsic");
  check((task.sensors.raw[1].translation - V3(-.11, 0, .332)).norm() < 1e-6,
        "Ground lidar extrinsic");
  auto attitude = Eigen::Map<V4>(task.simulation.data->sensordata + task.sensors.attitude_address);
  V4 original_attitude = attitude;
  attitude *= 2;
  task.sensors.sample(task.simulation.data, .1);
  check((task.sensors.attitude.transpose() * task.sensors.attitude - M3::Identity()).norm() < 1e-10,
        "AHRS quaternion normalization");
  attitude.setZero();
  bool invalid_attitude = false;
  try {
    task.sensors.sample(task.simulation.data, .1);
  } catch (const std::invalid_argument&) {
    invalid_attitude = true;
  }
  check(invalid_attitude, "Zero AHRS quaternion accepted");
  attitude = original_attitude;
  task.set_goal(7.5, 5);
  double heading = 0;
  for (int i = 0; i < 70 && task.goal; ++i) {
    task.step();
    heading = std::max(heading, std::abs(task.mapper->pose[3]));
  }
  check(task.status == "已到达", "Physical sideways goal failed");
  check(heading < .3, "Strafe rotated toward goal");
  check(task.power_peak_w <= task.drive.budget + 1e-7, "Physical power exceeds budget");
  check(task.state()["localization_error"].get<double>() < .1, "Localization accuracy");
  task.reset();
  task.set_goal(7.5, 6.0);
  task.set_yaw_rate(.6);
  int moving_spin = 0;
  for (int i = 0; i < 80 && task.goal; ++i) {
    task.step();
    moving_spin += task.measured_velocity.head<2>().norm() > .2 && task.measured_velocity.z() > .2;
  }
  check(task.status == "已到达" && moving_spin >= 10 && std::abs(task.mapper->pose[3]) > .8,
        "Physical simultaneous spin and translation");
  check(task.power_peak_w <= task.drive.budget + 1e-7, "Spinning power exceeds budget");
  task.reset();
  task.set_goal(8, 4);
  task.simulation.data->warning[0].number = 1;
  double before = task.simulation.data->time;
  task.step();
  check(task.paused && task.simulation.data->time == before && task.velocity.norm() == 0,
        "Physics fault resumed motion");
}
int main(int argc, char** argv) {
  try {
    if (argc < 2) throw std::runtime_error("root required");
    if (argc > 2)
      physics(argv[1]);
    else
      algorithms(argv[1]);
    std::cout << "navigation checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
