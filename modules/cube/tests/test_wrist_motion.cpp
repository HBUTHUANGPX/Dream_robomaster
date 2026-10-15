#include <iostream>
#include <limits>
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
  for (double fraction : {.25, .5, .75, .9})
    for (double angle : {-std::numbers::pi, std::numbers::pi / 2})
      for (bool brake : {false, true}) {
        const double t = wrist_duration(angle, 2., 10., .0005, false, brake, fraction);
        if (brake && std::abs(angle) == std::numbers::pi) {
          if (t != 2 * wrist_duration(angle/2, 2., 10., .0005, false, false, fraction))
            throw std::runtime_error("匀速段半圈计费未包含两次刹车");
          auto mid = wrist_sample(.7, .7+angle, t, t/2, false, true, fraction);
          if (std::abs(mid.position-(.7+angle/2)) > 1e-12 ||
              std::abs(mid.velocity) > 1e-12 || std::abs(mid.acceleration) > 1e-12)
            throw std::runtime_error("匀速段半圈中点未刹停");
        }
        for (int k = 0; k <= 2000; ++k) {
          const double time = t * k / 2000.;
          auto r = wrist_sample(.7, .7 + angle, t, time, false, brake, fraction);
          if (std::abs(r.velocity) > 2. + 1e-9 || std::abs(r.acceleration) > 10. + 1e-9)
            throw std::runtime_error("匀速段曲线超过速度或加速度限制");
          if (k > 0 && k < 2000) {
            const double h = t * 1e-7;
            auto a = wrist_sample(.7, .7 + angle, t, time - h, false, brake, fraction);
            auto b = wrist_sample(.7, .7 + angle, t, time + h, false, brake, fraction);
            if (std::abs((b.position - a.position) / (2*h) - r.velocity) > 1e-5 ||
                std::abs((b.velocity - a.velocity) / (2*h) - r.acceleration) > 1e-4)
              throw std::runtime_error("匀速段曲线的位置、速度、加速度不一致: f=" + std::to_string(fraction) +
                  " k=" + std::to_string(k) + " brake=" + std::to_string(brake) +
                  " dv=" + std::to_string((b.position-a.position)/(2*h)-r.velocity) +
                  " da=" + std::to_string((b.velocity-a.velocity)/(2*h)-r.acceleration));
          }
        }
        for (double time : {0., t}) {
          auto r = wrist_sample(.7, .7 + angle, t, time, false, brake, fraction);
          if (std::abs(r.position - (time == 0 ? .7 : .7 + angle)) > 1e-12 ||
              std::abs(r.velocity) > 1e-12 || std::abs(r.acceleration) > 1e-12)
            throw std::runtime_error("匀速段曲线端点不连续");
        }
      }
  if (wrist_duration(std::numbers::pi / 2, 2., 10., .0005, false, false, .5) >=
      .9 * wrist_duration(std::numbers::pi / 2, 2., 10., .0005))
    throw std::runtime_error("相同运动限制下匀速段曲线未减少执行时间");
  for (double fraction : {-1., 1., std::numeric_limits<double>::quiet_NaN()}) {
    bool duration_rejected = false, sample_rejected = false;
    try { wrist_duration(1., 2., 10., .001, false, false, fraction); }
    catch (const std::invalid_argument&) { duration_rejected = true; }
    try { wrist_sample(0., 1., 1., .5, false, false, fraction); }
    catch (const std::invalid_argument&) { sample_rejected = true; }
    if (!duration_rejected || !sample_rejected)
      throw std::runtime_error("非法匀速段比例未被拒绝");
  }
  bool conflict_rejected = false;
  try { wrist_duration(1., 2., 10., .001, true, false, .5); }
  catch (const std::invalid_argument&) { conflict_rejected = true; }
  if (!conflict_rejected) throw std::runtime_error("互斥轨迹参数未被拒绝");
  std::cout << "腕轨迹速度、加速度边界及端点检查通过\n";
}
