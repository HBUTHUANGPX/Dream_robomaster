#pragma once
#include <Eigen/Dense>
#include <optional>
#include <string>
namespace rm::duel {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
constexpr double pi = 3.14159265358979323846;
constexpr double projectile_radius = .0085;
double wrap(double angle);
struct Heat {
  double limit = 40, rate = 12, value = 0;
  bool locked = false, permanent = false;
  int tick = 0;
  explicit Heat(const std::string& profile = "cooling");
  void cool(double time);
  bool fire(double time, bool protect = true);
};
std::optional<double> segment_box(const Vec3&, const Vec3&, const Vec3&);
std::optional<double> segment_ellipsoid(const Vec3&, const Vec3&, const Vec3&);
std::optional<double> segment_cylinder(const Vec3&, const Vec3&, double, double);
struct Projectile {
  int owner;
  Vec3 position, velocity;
  double born;
};
}  // namespace rm::duel
