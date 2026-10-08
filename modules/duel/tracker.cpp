#include "tracker.hpp"

#include <algorithm>
#include <limits>
#include <tuple>
namespace rm::duel {
void RotorTracker::predict(double t) {
  if (!x) return;
  double dt = std::max(0., t - time);
  Covariance F = Covariance::Identity(), Q = Covariance::Zero();
  for (auto [p, v, n] :
       {std::tuple{0, 1, 4.}, std::tuple{2, 3, 4.}, std::tuple{4, 5, .1}, std::tuple{6, 7, .5}}) {
    F(p, v) = dt;
    Q(p, p) = n * std::pow(dt, 4) / 4;
    Q(p, v) = Q(v, p) = n * std::pow(dt, 3) / 2;
    Q(v, v) = n * dt * dt;
  }
  Q(8, 8) = Q(9, 9) = .0001 * dt;
  Q(10, 10) = .00001 * dt;
  *x = (F * (*x)).eval();
  P = (F * P * F.transpose() + Q).eval();
  time = t;
}
Vec3 RotorTracker::center(double t) const {
  const auto& s = x.value();
  double dt = std::max(0., t - time);
  return {s[0] + s[1] * dt, s[2] + s[3] * dt, s[4] + s[5] * dt};
}
std::array<double, 4> RotorTracker::armor_yaws(double t) const {
  std::array<double, 4> a;
  for (int i = 0; i < 4; ++i)
    a[i] = x.value()[6] + x.value()[7] * std::max(0., t - time) + i * pi / 2;
  return a;
}
std::array<Vec3, 4> RotorTracker::armor_positions(double t) const {
  auto y = armor_yaws(t);
  auto c = center(t);
  std::array<Vec3, 4> a;
  for (int i = 0; i < 4; ++i) {
    double r = (*x)[i % 2 ? 9 : 8];
    a[i] = c + Vec3(r * std::cos(y[i]), r * std::sin(y[i]), i % 2 ? dz() : 0);
  }
  return a;
}
std::pair<double, int> RotorTracker::association(const Observation& o, double t) const {
  if (!x) return {0, 0};
  if (o.number != number) return {std::numeric_limits<double>::infinity(), 0};
  auto p = armor_positions(t);
  auto y = armor_yaws(t);
  std::pair<double, int> best{std::numeric_limits<double>::infinity(), 0};
  for (int i = 0; i < 4; ++i) {
    double cost =
        (o.position - p[i]).squaredNorm() / (.12 * .12) + std::pow(wrap(o.yaw - y[i]) / .45, 2);
    if (cost < best.first) best = {cost, i};
  }
  return best;
}
std::optional<Observation> RotorTracker::update(const std::vector<Observation>& input, double t) {
  std::vector<Observation> obs;
  for (auto& o : input)
    if (!o.number.empty() && o.confidence >= .8 && o.position.allFinite() && std::isfinite(o.yaw))
      obs.push_back(o);
  if (!x || state == "LOST" || t - last_seen > .6) {
    state = "LOST";
    count = 0;
    if (obs.empty()) return {};
    auto& o = obs.front();
    Vec3 c = o.position - Vec3(.255 * std::cos(o.yaw), .255 * std::sin(o.yaw), 0);
    State s;
    s << c.x(), 0, c.y(), 0, c.z(), 0, o.yaw, 0, .255, .255, 0;
    x = s;
    P.setZero();
    P.diagonal() << .03, 1, .03, 1, .01, .2, .15, 2, .008, .008, .01;
    index = 0;
    count = 1;
    time = last_seen = t;
    number = o.number;
    state = "DETECTING";
    return o;
  }
  predict(t);
  double cost = std::numeric_limits<double>::infinity();
  int oi = -1, idx = 0;
  for (size_t j = 0; j < obs.size(); ++j) {
    auto candidate = association(obs[j], t);
    if (candidate.first < cost) {
      cost = candidate.first;
      idx = candidate.second;
      oi = static_cast<int>(j);
    }
  }
  if (oi >= 0 && cost < 30) {
    auto& o = obs[oi];
    double yaw = (*x)[6] + idx * pi / 2, r = (*x)[idx % 2 ? 9 : 8];
    Eigen::Vector4d pred, meas;
    pred << armor_positions(t)[idx], yaw;
    meas << o.position, yaw + wrap(o.yaw - yaw);
    Eigen::Matrix<double, 4, 11> H = Eigen::Matrix<double, 4, 11>::Zero();
    H(0, 0) = H(1, 2) = H(2, 4) = H(3, 6) = 1;
    H(0, 6) = -r * std::sin(yaw);
    H(1, 6) = r * std::cos(yaw);
    H(0, idx % 2 ? 9 : 8) = std::cos(yaw);
    H(1, idx % 2 ? 9 : 8) = std::sin(yaw);
    if (idx % 2) H(2, 10) = 1;
    Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
    R.topLeftCorner<3, 3>() = o.covariance;
    R(3, 3) = o.yaw_std * o.yaw_std;
    auto residual = (meas - pred).eval();
    Eigen::Matrix4d S = H * P * H.transpose() + R;
    auto solver = S.ldlt();
    innovation = residual.dot(solver.solve(residual));
    if (innovation < 40) {
      Eigen::Matrix<double, 11, 4> K = solver.solve((H * P).eval()).transpose();
      *x += K * residual;
      Covariance A = Covariance::Identity() - K * H;
      P = (A * P * A.transpose() + K * R * K.transpose()).eval();
      P = ((P + P.transpose()) * .5).eval();
      (*x)[8] = std::clamp((*x)[8], .12, .4);
      (*x)[9] = std::clamp((*x)[9], .12, .4);
      (*x)[10] = std::clamp((*x)[10], -.1, .1);
      if (idx != index) ++switches;
      index = idx;
      last_seen = t;
      ++count;
      state = count >= 4 ? "TRACKING" : "DETECTING";
      return o;
    }
  }
  if (!obs.empty()) ++rejected;
  if (t - last_seen > .6) {
    state = "LOST";
    count = 0;
  } else if (count >= 4)
    state = "TEMP_LOST";
  return {};
}
}  // namespace rm::duel
