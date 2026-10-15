#pragma once

namespace rm::cube {
// RX-P2 S 电机减速器输出端实测 TN 下包络，输入 rad/s，返回非负 Nm。
// 取绝对速度并按 0.05 rpm 向上取格；超过公共测量网格返回零。
// 非有限速度抛出 std::invalid_argument。无静态倍率、无速度裁剪。
double rx_s_torque_limit(double velocity_rad_s);
// RX-P2 F 电机实测下包络；相同取格规则，测量上限189.9 rpm。
double rx_f_torque_limit(double velocity_rad_s);
}  // namespace rm::cube
