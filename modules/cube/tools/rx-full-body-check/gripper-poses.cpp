// 来源：modules/cube/src/rx_gripper.cpp 的闭环初始化公式。
// 离线副本仅将电机角从固定 -2.25 rad 扩展为参数；不执行动态仿真。
#include <mujoco/mujoco.h>

#include <Eigen/Geometry>
#include <stdexcept>
#include <string>
namespace rm::cube {
namespace {
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
const Vec A(-.06784728, .038, 0), B(-.0235475, .03009246, 0), E(-.04332617, .031006855, -.0095),
    C(-.04814728, -.0043544, -.0122515), O(-.04814728, 0, 0);
constexpr double phase = -.8389783259706423, rod = .0357943041426583;
Mat rx(double t) { return Eigen::AngleAxisd(t, Vec::UnitX()).toRotationMatrix(); }
Mat rz(double t) { return Eigen::AngleAxisd(t, Vec::UnitZ()).toRotationMatrix(); }
double angle(double motor) {
  auto f = [&](double t) {
    return (A + rz(t) * (E - A) - (O + rx(phase - motor) * (C - O))).squaredNorm() - rod * rod;
  };
  double lo = -.6, hi = .8;
  for (int i = 0; i < 70; ++i) {
    double mid = (lo + hi) / 2;
    if (f(mid) * f(lo) > 0)
      lo = mid;
    else
      hi = mid;
  }
  return (lo + hi) / 2;
}
int id(const mjModel* model, mjtObj type, const std::string& name) {
  const int result = mj_name2id(model, type, name.c_str());
  if (result < 0) throw std::runtime_error("RX 模型缺少对象: " + name);
  return result;
}
void initialize_hand(mjModel* model, mjData* data, bool left, double motor = -2.25) {
  const bool swapped = mj_name2id(model, mjOBJ_BODY, "A_r_gripper_link") >= 0;
  const std::string hand = left != swapped ? "A" : "B";
  const std::string prefix = hand + (left ? "_l" : "_r");
  const double t = angle(motor);
  auto put = [&](const std::string& suffix, double value) {
    const int joint = id(model, mjOBJ_JOINT, prefix + suffix);
    data->qpos[model->jnt_qposadr[joint]] = value;
  };
  put("_gripper_motor_joint", motor);
  data->ctrl[id(model, mjOBJ_ACTUATOR, hand + "_fingers_actuator")] = motor;
  Mat S = Vec(1, -1, -1).asDiagonal(), T = left ? S : Mat::Identity(), rm = rx(phase - motor);
  for (const std::string branch : {"positive", "negative"}) {
    Mat U = branch == "positive" ? Mat::Identity() : S;
    const int finger = (T * U * B).y() > 0 ? 2 : 1;
    put("_" + branch + "_outer_joint", t);
    put("_" + branch + "_inner_joint", t);
    put("_finger" + std::to_string(finger) + "_joint", -t);
    Vec delta = (A + rz(t) * (E - A)) - (O + rm * (C - O));
    Mat rg = Eigen::Quaterniond::FromTwoVectors(E - C, delta).toRotationMatrix(), tr = T * U;
    Eigen::Quaterniond q(tr * rm.transpose() * rg * tr.transpose());
    const int joint = id(model, mjOBJ_JOINT, prefix + "_" + branch + "_rod_ball");
    const int adr = model->jnt_qposadr[joint];
    data->qpos[adr] = q.w();
    data->qpos[adr + 1] = q.x();
    data->qpos[adr + 2] = q.y();
    data->qpos[adr + 3] = q.z();
  }
}
}  // namespace
void set_probe_hand(mjModel* m, mjData* d, bool left, double motor) {
  initialize_hand(m, d, left, motor);
}
}  // namespace rm::cube
