#include <algorithm>

#include "rm/navigation.hpp"
namespace rm::nav {
void PID::reset() {
  integral = integral_before = previous = derivative = 0;
  has_previous = false;
}
double PID::update(double error, double dt) {
  if (!std::isfinite(error) || !std::isfinite(dt) || dt <= 0)
    throw std::invalid_argument("Invalid PID input");
  double raw = has_previous ? (error - previous) / dt : 0;
  derivative += dt / (tau + dt) * (raw - derivative);
  previous = error;
  has_previous = true;
  integral_before = integral;
  double next = integral + error * dt, value = kp * error + ki * next + kd * derivative;
  if (std::abs(value) <= limit || value * error < 0) integral = next;
  return std::clamp(kp * error + ki * integral + kd * derivative, -limit, limit);
}
void Tracker::reset() {
  for (auto& p : pid) p.reset();
  error.setZero();
}
V3 Tracker::update(const V4& pose, const V3& reference, const V3& feedforward, double dt) {
  V2 delta = reference.head<2>() - pose.head<2>();
  double c = std::cos(pose[3]), s = std::sin(pose[3]);
  error = V3(c * delta.x() + s * delta.y(), -s * delta.x() + c * delta.y(),
             wrap(reference.z() - pose[3]));
  V3 result = feedforward;
  for (int i = 0; i < 3; ++i) result[i] += pid[i].update(error[i], dt);
  return result;
}
void Tracker::accept(const V3& proposed, const V3& executed) {
  for (int i = 0; i < 3; ++i)
    if (std::abs(proposed[i] - executed[i]) > 1e-9) pid[i].integral = pid[i].integral_before;
}
Drive::Drive(double m, double b) : mass(m), inertia(m * (.57 * .57 + .57 * .57) / 12), budget(b) {
  if (!std::isfinite(b) || b < 20)
    throw std::invalid_argument("Power budget must be at least 20 W");
  J << 1, -1, -.36, 1, 1, .36, 1, 1, -.36, 1, -1, .36;
  J /= .076;
  force_to_torque = J * (J.transpose() * J).inverse();
}
V4 Drive::wheels(const V3& command) const { return J * command; }
double Drive::electrical(const V4& torque, const V4& omega) const {
  return torque.cwiseProduct(omega).cwiseMax(0).sum() / .8 + .12 * torque.squaredNorm() +
         .003 * omega.squaredNorm() + 4;
}
V4 Drive::limit_torque(const V4& request, const V4& omega) const {
  V4 torque = request.cwiseMax(-10).cwiseMin(10);
  double base = 4 + .003 * omega.squaredNorm(),
         linear = torque.cwiseProduct(omega).cwiseMax(0).sum() / .8,
         quadratic = .12 * torque.squaredNorm(), available = std::max(0., budget - base);
  if (linear + quadratic <= available) return torque;
  double denom = linear + std::sqrt(linear * linear + 4 * quadratic * available);
  return torque * (denom == 0 ? 0 : std::min(1., 2 * available / denom));
}
Drive::Prediction Drive::predict(const V3& command, const V3& current, double dt,
                                 const V2& gravity) const {
  V3 acceleration = (command - current) / dt;
  acceleration.head<2>() += command.z() * V2(-command.y(), command.x());
  V3 force;
  force.head<2>() =
      mass * acceleration.head<2>() +
      mass * 9.81 *
          V2(.035, .070).cwiseProduct((command.head<2>() / .05).cwiseMax(-1).cwiseMin(1)) +
      mass * gravity;
  force.z() = inertia * acceleration.z() + .15 * command.z();
  V4 torque = force_to_torque * force, omega = wheels(command);
  return {electrical(torque, omega), torque, omega};
}
bool Drive::feasible(const V3& command, const V3& current, double dt, const V2& gravity) const {
  auto p = predict(command, current, dt, gravity);
  return p.power <= budget && p.torque.cwiseAbs().maxCoeff() <= 10 &&
         p.omega.cwiseAbs().maxCoeff() <= 35;
}
std::pair<V3, V3> DWA::window(const V3& current, double speed) const {
  V3 lo, hi;
  for (int i = 0; i < 3; ++i) {
    double delta = i == 2 ? .24 : .16, limit = i == 2 ? 1.2 : speed;
    double rl = current[i] - delta, rh = current[i] + delta;
    lo[i] = std::max(rl, -limit);
    hi[i] = std::min(rh, limit);
    if (rh < -limit) lo[i] = hi[i] = rh;
    if (rl > limit) lo[i] = hi[i] = rl;
  }
  return {lo, hi};
}
std::pair<Points, int> DWA::rollout(const V4& pose, const V3& command) const {
  V3 state(pose[0], pose[1], pose[3]);
  V2 v = command.head<2>();
  double w = command.z();
  int normal = std::max(1, int(std::round(horizon / .1))),
      tail = int(std::ceil(std::max(v.norm(), std::abs(w) / 2.4) / .1)) + 1;
  Points points{state};
  for (int i = 0; i < normal + tail; ++i) {
    V2 next = v;
    double next_w = w;
    if (i >= normal) {
      next = v * std::max(0., 1 - .1 / std::max(v.norm(), 1e-12));
      next_w = std::copysign(std::max(0., std::abs(w) - .24), w);
    }
    double yaw = state.z() + w * .05;
    state.x() += (v.x() * std::cos(yaw) - v.y() * std::sin(yaw)) * .1;
    state.y() += (v.x() * std::sin(yaw) + v.y() * std::cos(yaw)) * .1;
    state.z() += w * .1;
    points.push_back(state);
    v = next;
    w = next_w;
  }
  return {points, normal};
}
bool DWA::terrain_check(const std::vector<V2>& xy, double* minimum) const {
  double travel = 0, initial_edge = 0, clearance = INFINITY;
  int previous = -1;
  for (size_t i = 0; i < xy.size(); ++i) {
    auto p = xy[i];
    if (p.x() < 0 || p.y() < 0 || p.x() >= terrain.width || p.y() >= terrain.height) return false;
    int cell = terrain.cell(p.x(), p.y()), r = cell / terrain.cols, c = cell % terrain.cols;
    if (!terrain.valid[cell]) return false;
    if (i) {
      travel += (p - xy[i - 1]).norm();
      if (!terrain.valid_at(previous / terrain.cols, c) ||
          !terrain.valid_at(r, previous % terrain.cols))
        return false;
      double dz = std::abs(terrain.heights[cell] - terrain.heights[previous]),
             dc = std::hypot(c - previous % terrain.cols, r - previous / terrain.cols) * res;
      if (dz > dc * std::tan(28 * pi / 180) + .008) return false;
    }
    double edge = std::min({p.x(), p.y(), terrain.width - p.x(), terrain.height - p.y()});
    for (int dr = -1; dr <= 1; ++dr)
      for (int dc = -1; dc <= 1; ++dc)
        if (!terrain.valid_at(r + dr, c + dc)) {
          double dx = std::max({(c + dc) * res - p.x(), p.x() - (c + dc + 1) * res, 0.}),
                 dy = std::max({(r + dr) * res - p.y(), p.y() - (r + dr + 1) * res, 0.});
          edge = std::min(edge, std::hypot(dx, dy));
        }
    if (i == 0) initial_edge = edge;
    if (edge < std::min(.035, initial_edge + .05 * travel) - 1e-9) return false;
    clearance = std::min(clearance, terrain.clearance[cell]);
    previous = cell;
  }
  if (minimum) *minimum = clearance;
  return true;
}
bool DWA::line_clear(const V2& a, const V2& b) const {
  int n = std::max(2, int((b - a).norm() / .025) + 2);
  std::vector<V2> points;
  for (int i = 0; i < n; ++i) points.push_back(a + (b - a) * double(i) / (n - 1));
  return terrain_check(points);
}
bool DWA::safe(const V4& pose, const V3& command, const Points& obstacles, double* minimum) const {
  if (!command.allFinite()) return false;
  auto [states, normal] = rollout(pose, command);
  (void)normal;
  double max_length = 0;
  for (size_t i = 1; i < states.size(); ++i)
    max_length = std::max(max_length, (states[i] - states[i - 1]).head<2>().norm());
  int subdivisions = std::max(1, int(std::ceil(max_length / .05)));
  std::vector<V2> xy;
  for (size_t i = 0; i + 1 < states.size(); ++i)
    for (int j = 0; j < subdivisions; ++j)
      xy.push_back((states[i] + (states[i + 1] - states[i]) * double(j) / subdivisions).head<2>());
  xy.push_back(states.back().head<2>());
  double clearance = INFINITY;
  if (!terrain_check(xy, &clearance)) return false;
  for (size_t i = 0; i < xy.size(); ++i)
    for (auto obstacle : obstacles) {
      double distance = (xy[i] - obstacle.head<2>()).norm();
      if (distance <= .40 + (i ? command.head<2>().norm() * .1 / (2 * subdivisions) : 0) + 1e-6)
        return false;
      clearance = std::min(clearance, distance - .40 + .38);
    }
  if (minimum) *minimum = clearance;
  return true;
}
bool DWA::motion_valid(const V3& command, const V3& current, double speed) const {
  return command.head<2>().norm() <= std::max(speed, current.head<2>().norm() - .16) + 1e-8 &&
         (command - current).head<2>().norm() <= .16 + 1e-8 &&
         drive.feasible(command, current, .1, gravity);
}
std::pair<V3, Points> DWA::plan(const V4& pose, const V3& current, const Points& path,
                                const V3& goal, const Points& obstacles, double speed, double yaw) {
  if (path.empty()) return {V3::Zero(), {}};
  auto [lo, hi] = window(current, speed);
  size_t last = path.size() - 1;
  for (size_t i = 0; i < path.size(); ++i)
    if ((path[i].head<2>() - pose.head<2>()).norm() >=
        std::max(.65, current.head<2>().norm() * horizon + .4)) {
      last = i;
      break;
    }
  V2 target = path.front().head<2>();
  for (int i = int(last); i >= 0; --i)
    if (line_clear(pose.head<2>(), path[i].head<2>())) {
      target = path[i].head<2>();
      break;
    }
  auto scan_clear = [&](const V2& to) {
    int n = std::max(2, int((to - pose.head<2>()).norm() / .025) + 2);
    for (int i = 0; i < n; ++i) {
      V2 p = pose.head<2>() + (to - pose.head<2>()) * double(i) / (n - 1);
      for (auto o : obstacles)
        if ((p - o.head<2>()).norm() <= .42) return false;
    }
    return true;
  };
  bool detour = false;
  if (!obstacles.empty() && !scan_clear(target)) {
    double heading = std::atan2(target.y() - pose[1], target.x() - pose[0]), best = INFINITY;
    V2 chosen = target;
    for (int i = 0; i < 25; ++i) {
      double angle = -pi / 2 + i * pi / 24;
      for (double reach : {.65, .4, .25}) {
        V2 p = pose.head<2>() + reach * V2(std::cos(heading + angle), std::sin(heading + angle));
        if (line_clear(pose.head<2>(), p) && scan_clear(p)) {
          double score = (p - target).norm() + .1 * std::abs(angle);
          if (score < best) {
            best = score;
            chosen = p;
          }
        }
      }
    }
    if (std::isfinite(best)) {
      target = chosen;
      detour = true;
    }
  }
  std::array<std::vector<double>, 3> axes;
  for (int i = 0; i < 3; ++i) {
    int n = i < 2 ? 5 : 7;
    for (int j = 0; j < n; ++j) axes[i].push_back(lo[i] + (hi[i] - lo[i]) * j / (n - 1));
    axes[i].push_back(std::clamp(0., lo[i], hi[i]));
    std::sort(axes[i].begin(), axes[i].end());
    axes[i].erase(std::unique(axes[i].begin(), axes[i].end()), axes[i].end());
  }
  Points commands;
  for (double x : axes[0])
    for (double y : axes[1])
      for (double w : axes[2]) commands.emplace_back(x, y, w);
  V2 delta = target - pose.head<2>();
  double c = std::cos(pose[3]), s = std::sin(pose[3]);
  V2 direction(c * delta.x() + s * delta.y(), -s * delta.x() + c * delta.y());
  V2 desired =
      direction / std::max(direction.norm(), 1e-9) * std::min(speed, direction.norm() / horizon);
  for (V2 requested : {desired, V2::Zero().eval()}) {
    V2 change = requested - current.head<2>(),
       xy = current.head<2>() + change * std::min(1., .16 / std::max(change.norm(), 1e-9));
    auto yaw_samples = axes[2];
    yaw_samples.push_back(std::clamp(yaw, lo.z(), hi.z()));
    for (double w : yaw_samples)
      commands.push_back(V3(xy.x(), xy.y(), w).cwiseMax(lo).cwiseMin(hi));
  }
  std::sort(commands.begin(), commands.end(), [](auto a, auto b) {
    for (int i = 0; i < 3; ++i) {
      if (a[i] < b[i]) return true;
      if (a[i] > b[i]) return false;
    }
    return false;
  });
  commands.erase(
      std::unique(commands.begin(), commands.end(), [](auto a, auto b) { return a == b; }),
      commands.end());
  candidates = int(commands.size());
  admissible = 0;
  double best = INFINITY;
  V3 command = V3::Zero();
  Points trajectory;
  for (auto candidate : commands) {
    if (!motion_valid(candidate, current, speed)) continue;
    double clearance;
    if (!safe(pose, candidate, obstacles, &clearance)) continue;
    ++admissible;
    auto [states, normal] = rollout(pose, candidate);
    V2 endpoint = states[normal].head<2>();
    double path_distance = INFINITY;
    for (auto p : path) path_distance = std::min(path_distance, (endpoint - p.head<2>()).norm());
    double power = drive.predict(candidate, current, .1, gravity).power;
    double cost = (detour ? .25 : 3.) * path_distance + 2 * (target - endpoint).norm() +
                  .8 * std::abs(candidate.z() - yaw) + .025 / std::max(clearance, .05) +
                  .03 * power / drive.budget;
    if ((goal.head<2>() - pose.head<2>()).norm() < .4)
      cost = 3 * (endpoint - goal.head<2>()).norm() + .8 * std::abs(candidate.z() - yaw) +
             .03 * power / drive.budget;
    if (cost < best) {
      best = cost;
      command = candidate;
      trajectory.assign(states.begin() + 1, states.begin() + normal + 1);
    }
  }
  return {command, trajectory};
}
std::pair<V3, bool> DWA::constrain(const V4& pose, const V3& current, const V3& proposed,
                                   const V3& nominal, const Points& obstacles, double speed) {
  auto [lo, hi] = window(current, speed);
  V3 command = proposed.cwiseMax(lo).cwiseMin(hi);
  if (motion_valid(command, current, speed) && safe(pose, command, obstacles))
    return {command, true};
  return {nominal, false};
}
}  // namespace rm::nav
