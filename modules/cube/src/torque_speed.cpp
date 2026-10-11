#include "rm/torque_speed.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace rm::cube {
namespace {
// 来源和 SHA-256 见 assets/rx_gripper/tn/README.md；数组包含零速锚点。
struct Curve {
  std::array<double, 14> speed, torque;
};
constexpr Curve motor32{
    {0., 11.2, 26.2, 52.3, 78.3, 104.2, 130.3, 155.3, 181.2, 207.2, 233.4, 259.3, 285.3, 309.6},
    {21.32, 21.32, 21.19, 21.05, 20.89, 20.89, 20.67, 20.54, 20.62, 20.46, 20.23, 19.84, 19.66,
     19.12}};
constexpr Curve motor34{
    {0., 11.2, 26.3, 52.4, 78.2, 104.1, 130.2, 155.2, 181.2, 207.2, 233.2, 259.3, 285.3, 310.3},
    {21.31, 21.31, 21.17, 21.15, 21.05, 21.05, 21.12, 20.88, 20.69, 20.54, 20.38, 20.29, 19.61,
     19.21}};
constexpr double step_rpm = .05;
constexpr double index_scale = (60. / (2. * std::numbers::pi)) / step_rpm;
constexpr size_t last_measured_bin = 6192;  // floor(min(309.6,310.3)/.05 + 1e-12)
using Table = std::array<double, last_measured_bin + 2>;

double endpoint(double h0, double h1, double d0, double d1) {
  const double slope = ((2. * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
  if (slope * d0 <= 0.) return 0.;
  if (d0 * d1 < 0. && std::abs(slope) > 3. * std::abs(d0)) return 3. * d0;
  return slope;
}

std::array<double, 14> slopes(const Curve& curve) {
  std::array<double, 13> h{}, d{};
  for (size_t i = 0; i < h.size(); ++i) {
    h[i] = curve.speed[i + 1] - curve.speed[i];
    d[i] = (curve.torque[i + 1] - curve.torque[i]) / h[i];
  }
  std::array<double, 14> result{};
  for (size_t i = 1; i + 1 < result.size(); ++i) {
    if (d[i - 1] * d[i] <= 0.) continue;
    const double left = 2. * h[i] + h[i - 1], right = h[i] + 2. * h[i - 1];
    result[i] = (left + right) / (left / d[i - 1] + right / d[i]);
  }
  result.front() = endpoint(h[0], h[1], d[0], d[1]);
  result.back() = endpoint(h[12], h[11], d[12], d[11]);
  return result;
}

double evaluate(const Curve& c, const std::array<double, 14>& d, double speed) {
  const size_t i = std::min<size_t>(
      std::upper_bound(c.speed.begin(), c.speed.end(), speed) - c.speed.begin() - 1, 12);
  const double width = c.speed[i + 1] - c.speed[i];
  const double t = (speed - c.speed[i]) / width, t2 = t * t, t3 = t2 * t;
  return std::max(0., (2. * t3 - 3. * t2 + 1.) * c.torque[i] + (t3 - 2. * t2 + t) * width * d[i] +
                          (-2. * t3 + 3. * t2) * c.torque[i + 1] + (t3 - t2) * width * d[i + 1]);
}

Table build() {
  Table table{};  // 最后一格保留为零 sentinel。
  const auto d32 = slopes(motor32), d34 = slopes(motor34);
  double previous = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i <= last_measured_bin; ++i) {
    const double speed = double(i) * step_rpm;
    previous = std::min({previous, evaluate(motor32, d32, speed), evaluate(motor34, d34, speed)});
    table[i] = previous;
  }
  return table;
}
}  // namespace

double rx_s_torque_limit(double velocity_rad_s) {
  if (!std::isfinite(velocity_rad_s))
    throw std::invalid_argument("S motor TN requires finite joint velocity");
  // 先以浮点数限制索引，避免巨大有限输入乘法溢出后的整数转换。
  const double bin = std::ceil(std::abs(velocity_rad_s) * index_scale);
  if (bin > double(last_measured_bin)) return 0.;
  static const Table limits = build();  // C++ 保证首次初始化线程安全；后续只有查表。
  return limits[static_cast<size_t>(bin)];
}
}  // namespace rm::cube
