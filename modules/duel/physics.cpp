#include "physics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace rm::duel {
double wrap(double a) { return a - 2 * pi * std::floor((a + pi) / (2 * pi)); }
Heat::Heat(const std::string& profile) {
  if (profile == "burst") {
    limit = 170;
    rate = 5;
  } else if (profile != "cooling")
    throw std::invalid_argument("unknown heat profile");
}
void Heat::cool(double time) {
  int next = static_cast<int>(std::floor(time * 10 + 1e-8));
  value = std::max(0., value - std::max(0, next - tick) * rate / 10);
  tick = next;
  if (value == 0 && !permanent) locked = false;
}
bool Heat::fire(double time, bool protect) {
  cool(time);
  if (locked || (protect && value + 10 > limit + 1e-8)) return false;
  value += 10;
  locked = value > limit;
  permanent = permanent || value >= limit + 100;
  return true;
}
std::optional<double> segment_box(const Vec3& a, const Vec3& b, const Vec3& s) {
  double lo = 0, hi = 1;
  for (int i = 0; i < 3; ++i) {
    double d = b[i] - a[i];
    if (std::abs(d) < 1e-12) {
      if (std::abs(a[i]) > s[i]) return {};
    } else {
      double u = (-s[i] - a[i]) / d, v = (s[i] - a[i]) / d;
      if (u > v) std::swap(u, v);
      lo = std::max(lo, u);
      hi = std::min(hi, v);
      if (lo > hi) return {};
    }
  }
  return lo;
}
std::optional<double> segment_ellipsoid(const Vec3& start, const Vec3& end, const Vec3& r) {
  Vec3 a = start.cwiseQuotient(r), d = (end - start).cwiseQuotient(r);
  double c = a.squaredNorm() - 1;
  if (c <= 0) return 0.;
  double aa = d.squaredNorm(), b = a.dot(d), disc = b * b - aa * c;
  if (aa < 1e-15 || disc < 0) return {};
  double f = (-b - std::sqrt(disc)) / aa;
  if (f >= 0 && f <= 1) return f;
  return {};
}
std::optional<double> segment_cylinder(const Vec3& a, const Vec3& b, double r, double h) {
  Vec3 d = b - a;
  if (a.head<2>().norm() <= r && std::abs(a.z()) <= h) return 0.;
  double best = 2, A = d.head<2>().squaredNorm(), B = a.head<2>().dot(d.head<2>()),
         C = a.head<2>().squaredNorm() - r * r, disc = B * B - A * C;
  auto accept = [&](double f) {
    if (f >= 0 && f <= 1) best = std::min(best, f);
  };
  if (A > 1e-15 && disc >= 0)
    for (double f : {(-B - std::sqrt(disc)) / A, (-B + std::sqrt(disc)) / A})
      if (std::abs(a.z() + f * d.z()) <= h) accept(f);
  if (std::abs(d.z()) > 1e-15)
    for (double z : {-h, h}) {
      double f = (z - a.z()) / d.z();
      if ((a + f * d).head<2>().norm() <= r) accept(f);
    }
  if (best <= 1) return best;
  return {};
}
}  // namespace rm::duel
