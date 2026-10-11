#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm::cube {
struct WristSample {
  double position, velocity, acceleration;
};
inline double wrist_duration(double angle, double speed, double acceleration, double dt) {
  if (!std::isfinite(angle) || !std::isfinite(speed) || !std::isfinite(acceleration) ||
      !std::isfinite(dt) || speed <= 0 || acceleration <= 0 || dt <= 0)
    throw std::invalid_argument("Invalid wrist motion limits");
  const double distance = std::abs(angle);
  const double t = std::max(1.875 * distance / speed,
                            std::sqrt((10. / std::sqrt(3.)) * distance / acceleration));
  if (t > 3600) throw std::invalid_argument("Wrist duration exceeds one hour");
  return std::max(dt, std::ceil(t / dt) * dt);
}
inline WristSample wrist_sample(double start, double target, double duration, double time) {
  const double u = std::clamp(time / duration, 0., 1.), delta = target - start;
  return {start + delta * u * u * u * (10 + u * (-15 + 6 * u)),
          delta * 30 * u * u * (1 - u) * (1 - u) / duration,
          delta * 60 * u * (1 - u) * (1 - 2 * u) / (duration * duration)};
}
}  // namespace rm::cube
