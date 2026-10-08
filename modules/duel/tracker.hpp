#pragma once
#include <array>
#include <vector>

#include "physics.hpp"
namespace rm::duel {
struct Observation {
  Vec3 position;
  double yaw = 0;
  std::string number;
  double confidence = 0;
  Mat3 covariance = Mat3::Identity() * .015 * .015;
  double yaw_std = .20;
  int detection = -1;
};
class RotorTracker {
 public:
  using State = Eigen::Matrix<double, 11, 1>;
  using Covariance = Eigen::Matrix<double, 11, 11>;
  std::optional<State> x;
  Covariance P = Covariance::Identity();
  double time = 0, last_seen = -1, innovation = 0;
  std::string state = "LOST", number;
  int count = 0, switches = 0, index = 0, rejected = 0;
  void predict(double);
  Vec3 center(double) const;
  std::array<double, 4> armor_yaws(double) const;
  std::array<Vec3, 4> armor_positions(double) const;
  std::pair<double, int> association(const Observation&, double) const;
  std::optional<Observation> update(const std::vector<Observation>&, double);
  bool ready(double t) const { return x && count >= 4 && t - last_seen <= .18 && state != "LOST"; }
  double r2() const { return x ? (*x)[9] : .255; }
  double dz() const { return x ? (*x)[10] : 0.; }
};
}  // namespace rm::duel
