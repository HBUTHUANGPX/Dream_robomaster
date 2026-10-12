#include <iostream>
#include <numbers>

#include "rm/wrist_motion.hpp"
int main() {
  using namespace rm::cube;
  for (bool zero_jerk : {false, true})
    for (double angle : {-std::numbers::pi, std::numbers::pi}) {
      const double t = wrist_duration(angle, 12., 120., .0005, zero_jerk, true);
      const double quarter = wrist_duration(angle / 2, 12., 120., .0005, zero_jerk);
      if (t != 2 * quarter) throw std::runtime_error("Half turn cost must include both ramps");
      for (int k = 0; k <= 10000; ++k) {
        const double time = t * k / 10000.;
        const auto r = wrist_sample(.7, .7 + angle, t, time, zero_jerk, true);
        if (std::abs(r.velocity) > 12. + 1e-9 || std::abs(r.acceleration) > 120. + 1e-9)
          throw std::runtime_error("Braked half turn exceeded motion limits");
      }
      for (double time : {0., t / 2, t}) {
        const auto r = wrist_sample(.7, .7 + angle, t, time, zero_jerk, true);
        if (std::abs(r.position - (.7 + angle * time / t)) > 1e-12 ||
            std::abs(r.velocity) > 1e-12 || std::abs(r.acceleration) > 1e-12)
          throw std::runtime_error("Half turn braking discontinuity");
      }
    }
  for (bool zero_jerk : {false, true})
    for (double angle :
         {-std::numbers::pi, -std::numbers::pi / 2, std::numbers::pi / 2, std::numbers::pi})
      for (double speed : {4., 8., 16.}) {
        double t = wrist_duration(angle, speed, 80., .001, zero_jerk);
        for (int k = 0; k <= 10000; ++k) {
          auto r = wrist_sample(.7, .7 + angle, t, t * k / 10000., zero_jerk);
          if (std::abs(r.velocity) > speed + 1e-9 || std::abs(r.acceleration) > 80. + 1e-9)
            throw std::runtime_error("Wrist trajectory exceeded motion limits");
        }
        for (double time : {0., t}) {
          auto r = wrist_sample(.7, .7 + angle, t, time, zero_jerk);
          if (std::abs(r.velocity) > 1e-12 || std::abs(r.acceleration) > 1e-12 ||
              std::abs(r.position - (time == 0 ? .7 : .7 + angle)) > 1e-12)
            throw std::runtime_error("Wrist endpoint discontinuity");
          if (zero_jerk) {
            const double h = t * 1e-7;
            auto adjacent = wrist_sample(.7, .7 + angle, t, time == 0 ? h : t - h, true);
            if (std::abs(adjacent.acceleration / h * t * t * t / angle) > 1e-4)
              throw std::runtime_error("Wrist endpoint jerk is not zero");
          }
        }
      }
  for (double angle : {-std::numbers::pi / 2, std::numbers::pi / 2}) {
    const double plain = wrist_duration(angle, 12., 120., .0005);
    const double braked = wrist_duration(angle, 12., 120., .0005, false, true);
    if (plain != braked) throw std::runtime_error("Quarter turn timing changed");
    for (int k = 0; k <= 100; ++k) {
      auto a = wrist_sample(.7, .7 + angle, plain, plain * k / 100.);
      auto b = wrist_sample(.7, .7 + angle, braked, braked * k / 100., false, true);
      if (a.position != b.position || a.velocity != b.velocity || a.acceleration != b.acceleration)
        throw std::runtime_error("Quarter turn profile changed");
    }
  }
  std::cout << "腕轨迹速度、加速度边界及端点检查通过\n";
}
