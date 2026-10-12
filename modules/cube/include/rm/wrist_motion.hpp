#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm::cube {
struct WristSample {
  double position, velocity, acceleration;
};
inline double wrist_duration(double angle, double speed, double acceleration, double dt,
                             bool zero_jerk = false) {
  if (!std::isfinite(angle) || !std::isfinite(speed) || !std::isfinite(acceleration) ||
      !std::isfinite(dt) || speed <= 0 || acceleration <= 0 || dt <= 0)
    throw std::invalid_argument("Invalid wrist motion limits");
  const double distance = std::abs(angle);
  const double t = std::max((zero_jerk ? 2.1875 : 1.875) * distance / speed,
                            std::sqrt((zero_jerk ? 16.8 / std::sqrt(5.) : 10. / std::sqrt(3.)) *
                                      distance / acceleration));
  if (t > 3600) throw std::invalid_argument("Wrist duration exceeds one hour");
  return std::max(dt, std::ceil(t / dt) * dt);
}
inline WristSample wrist_sample(double start, double target, double duration, double time,
                                bool zero_jerk = false) {
  const double u = std::clamp(time / duration, 0., 1.), delta = target - start;
  if (zero_jerk)
    return {start + delta * u * u * u * u * (35 + u * (-84 + u * (70 - 20 * u))),
            delta * 140 * u * u * u * std::pow(1 - u, 3) / duration,
            delta * 420 * u * u * (1 - u) * (1 - u) * (1 - 2 * u) / (duration * duration)};
  return {start + delta * u * u * u * (10 + u * (-15 + 6 * u)),
          delta * 30 * u * u * (1 - u) * (1 - u) / duration,
          delta * 60 * u * (1 - u) * (1 - 2 * u) / (duration * duration)};
}
}  // namespace rm::cube
