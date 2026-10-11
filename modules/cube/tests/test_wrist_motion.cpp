#include <iostream>
#include <numbers>

#include "rm/wrist_motion.hpp"
int main() {
  using namespace rm::cube;
  for (double angle :
       {-std::numbers::pi, -std::numbers::pi / 2, std::numbers::pi / 2, std::numbers::pi})
    for (double speed : {4., 8., 16.}) {
      double t = wrist_duration(angle, speed, 80., .001);
      for (int k = 0; k <= 10000; ++k) {
        auto r = wrist_sample(.7, .7 + angle, t, t * k / 10000.);
        if (std::abs(r.velocity) > speed + 1e-9 || std::abs(r.acceleration) > 80. + 1e-9)
          throw std::runtime_error("Wrist trajectory exceeded motion limits");
      }
      for (double time : {0., t}) {
        auto r = wrist_sample(.7, .7 + angle, t, time);
        if (std::abs(r.velocity) > 1e-12 || std::abs(r.acceleration) > 1e-12 ||
            std::abs(r.position - (time == 0 ? .7 : .7 + angle)) > 1e-12)
          throw std::runtime_error("Wrist endpoint discontinuity");
      }
    }
  std::cout << "腕轨迹速度、加速度边界及端点检查通过\n";
}
