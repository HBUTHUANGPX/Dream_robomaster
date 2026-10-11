#pragma once
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include "rm/torque_speed.hpp"

namespace rm::cube {
// 在物理步开始时查询TN，只改变执行器输出边界，不裁剪关节速度。
class RxWristTn {
 public:
  struct Axis {
    int actuator = -1, dof = -1;
    double static_limit = 0, step_limit = 0;
    double peak_speed_rad_s = 0, peak_torque_nm = 0, max_excess_nm = 0;
    int reduced_steps = 0;
  };
  std::array<Axis, 2> axes;
  RxWristTn(mjModel* m) {
    for (int h = 0; h < 2; ++h) {
      const std::string hand = h == 0 ? "A" : "B";
      auto& a = axes[h];
      a.actuator = mj_name2id(m, mjOBJ_ACTUATOR, (hand + "_yaw_drive").c_str());
      const int joint = mj_name2id(m, mjOBJ_JOINT, (hand + "_yaw").c_str());
      if (a.actuator < 0 || joint < 0)
        throw std::invalid_argument("TN requires both RX wrist actuators");
      if (!m->actuator_forcelimited[a.actuator] || m->actuator_trntype[a.actuator] != mjTRN_JOINT ||
          m->actuator_trnid[2 * a.actuator] != joint || m->actuator_gear[6 * a.actuator] != 1)
        throw std::invalid_argument("TN requires force-limited unit-gear wrist actuators");
      a.dof = m->jnt_dofadr[joint];
      a.static_limit = std::min(-m->actuator_forcerange[2 * a.actuator],
                                m->actuator_forcerange[2 * a.actuator + 1]);
      if (!(a.static_limit > 0)) throw std::invalid_argument("Invalid wrist static torque limit");
    }
  }
  void before_step(mjModel* m, const mjData* d) {
    for (auto& a : axes) {
      const double speed = d->qvel[a.dof];
      a.step_limit = std::min(a.static_limit, rx_s_torque_limit(speed));
      a.peak_speed_rad_s = std::max(a.peak_speed_rad_s, std::abs(speed));
      a.reduced_steps += a.step_limit < a.static_limit;
      m->actuator_forcerange[2 * a.actuator] = -a.step_limit;
      m->actuator_forcerange[2 * a.actuator + 1] = a.step_limit;
    }
  }
  void after_step(const mjData* d) {
    for (auto& a : axes) {
      const double torque = std::abs(d->actuator_force[a.actuator]);
      if (!std::isfinite(torque)) throw std::runtime_error("Non-finite wrist torque");
      a.peak_torque_nm = std::max(a.peak_torque_nm, torque);
      a.max_excess_nm = std::max(a.max_excess_nm, torque - a.step_limit);
      if (a.max_excess_nm > 1e-9) throw std::runtime_error("Wrist torque exceeds TN bound");
    }
  }
};
}  // namespace rm::cube
