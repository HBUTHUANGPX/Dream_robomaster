#pragma once
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace rm::cube {
struct WristSample {
  double position, velocity, acceleration;
};
inline double wrist_duration(double angle, double speed, double acceleration, double dt,
                             bool zero_jerk = false, bool brake_half_turn = false,
                             double cruise_fraction = 0.) {
  if (!std::isfinite(cruise_fraction) || cruise_fraction < 0 || cruise_fraction > .9 ||
      (zero_jerk && cruise_fraction > 0))
    throw std::invalid_argument("Invalid wrist cruise fraction");
  if (!std::isfinite(angle) || !std::isfinite(speed) || !std::isfinite(acceleration) ||
      !std::isfinite(dt) || speed <= 0 || acceleration <= 0 || dt <= 0)
    throw std::invalid_argument("Invalid wrist motion limits");
  const double distance = std::abs(angle);
  if (brake_half_turn && std::abs(distance - std::numbers::pi) < 1e-9) {
    const double duration =
        2 * wrist_duration(angle / 2, speed, acceleration, dt, zero_jerk, false, cruise_fraction);
    if (duration > 3600) throw std::invalid_argument("Wrist duration exceeds one hour");
    return duration;
  }
  const double ramp = (1 - cruise_fraction) / 2;
  const double peak_v = cruise_fraction > 0 ? 1 / (1 - ramp) : (zero_jerk ? 2.1875 : 1.875);
  const double peak_a = cruise_fraction > 0
                            ? std::numbers::pi / (2 * ramp * (1 - ramp))
                            : (zero_jerk ? 16.8 / std::sqrt(5.) : 10. / std::sqrt(3.));
  const double t = std::max(peak_v * distance / speed, std::sqrt(peak_a * distance / acceleration));
  if (t > 3600) throw std::invalid_argument("Wrist duration exceeds one hour");
  return std::max(dt, std::ceil(t / dt) * dt);
}
inline WristSample wrist_sample(double start, double target, double duration, double time,
                                bool zero_jerk = false, bool brake_half_turn = false,
                                double cruise_fraction = 0.) {
  if (!std::isfinite(cruise_fraction) || cruise_fraction < 0 || cruise_fraction > .9 ||
      (zero_jerk && cruise_fraction > 0))
    throw std::invalid_argument("Invalid wrist cruise fraction");
  if (brake_half_turn && std::abs(std::abs(target - start) - std::numbers::pi) < 1e-9) {
    const double middle = (start + target) / 2;
    return time <= duration / 2
               ? wrist_sample(start, middle, duration / 2, time, zero_jerk, false, cruise_fraction)
               : wrist_sample(middle, target, duration / 2, time - duration / 2, zero_jerk, false,
                              cruise_fraction);
  }
  const double u = std::clamp(time / duration, 0., 1.), delta = target - start;
  if (cruise_fraction > 0) {
    const double ramp = (1 - cruise_fraction) / 2, peak = 1 / (1 - ramp);
    const bool decelerating = u > 1 - ramp;
    const double x = decelerating ? 1 - u : u;
    double p, v, a;
    if (x < ramp) {
      const double phase = std::numbers::pi * x / ramp;
      p = .5 * peak * (x - ramp / std::numbers::pi * std::sin(phase));
      v = .5 * peak * (1 - std::cos(phase));
      a = .5 * peak * std::numbers::pi / ramp * std::sin(phase);
    } else {
      p = peak * (x - ramp / 2);
      v = peak;
      a = 0;
    }
    if (decelerating) {
      p = 1 - p;
      a = -a;
    }
    return {start + delta * p, delta * v / duration, delta * a / (duration * duration)};
  }
  if (zero_jerk)
    return {start + delta * u * u * u * u * (35 + u * (-84 + u * (70 - 20 * u))),
            delta * 140 * u * u * u * std::pow(1 - u, 3) / duration,
            delta * 420 * u * u * (1 - u) * (1 - u) * (1 - 2 * u) / (duration * duration)};
  return {start + delta * u * u * u * (10 + u * (-15 + 6 * u)),
          delta * 30 * u * u * (1 - u) * (1 - u) / duration,
          delta * 60 * u * (1 - u) * (1 - 2 * u) / (duration * duration)};
}
}  // namespace rm::cube
