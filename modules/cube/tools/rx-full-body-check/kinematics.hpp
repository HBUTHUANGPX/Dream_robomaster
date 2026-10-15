#pragma once
// 来源：原单向候选的离线运动学探针；仅保留固定几何、FK 和连续分支 IK，不执行姿态搜索。
#include <mujoco/mujoco.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <vector>
using V = Eigen::Vector3d;
using M = Eigen::Matrix3d;
using Q = Eigen::Matrix<double, 6, 1>;
using J6 = Eigen::Matrix<double, 6, 6>;
using Json = nlohmann::json;
M rot(int a, double q) { return Eigen::AngleAxisd(q, V::Unit(a)).toRotationMatrix(); }
struct Pose {
  V p;
  M r;
};
Q lo(bool l) {
  Q q;
  if (l)
    q << -3.229, -.262, -2.007, -.524, -3.054, -1.623;
  else
    q << -3.229, -1.833, -3.054, -.524, -2.007, -1.623;
  return q;
}
Q hi(bool l) {
  Q q;
  if (l)
    q << 1.92, 1.833, 3.054, 2.094, 2.007, 1.623;
  else
    q << 1.92, .262, 2.007, 2.094, 3.054, 1.623;
  return q;
}
Pose fk(const Q& q, bool left, J6* J = nullptr) {
  double s = left ? 1 : -1;
  V o[] = {V(0, s * .041, .22),     V(-.028, s * .089, 0), V(.028, 0, -.037),
           V(0, -s * .03, -.113),   V(.037, s * .028, 0),  V(.113, -s * .028, 0),
           V(.11235, s * .02975, 0)};
  int ax[] = {1, 0, 2, 1, 0, 1};
  Pose f{V::Zero(), M::Identity()};
  V axes[6], points[6];
  for (int i = 0; i < 6; i++) {
    f.p += f.r * o[i];
    points[i] = f.p;
    axes[i] = f.r * V::Unit(ax[i]);
    f.r = f.r * rot(ax[i], q[i]);
  }
  f.p += f.r * o[6];
  if (J)
    for (int i = 0; i < 6; i++) {
      J->block<3, 1>(0, i) = axes[i].cross(f.p - points[i]);
      J->block<3, 1>(3, i) = .15 * axes[i];
    }
  return f;
}
Q err(Pose a, Pose b) {
  Q e;
  e.head<3>() = b.p - a.p;
  Eigen::AngleAxisd v(b.r * a.r.transpose());
  e.tail<3>() = .15 * v.angle() * v.axis();
  return e;
}
bool ik(Pose goal, bool left, Q& q) {
  auto l = lo(left), h = hi(left);
  for (int it = 0; it < 100; it++) {
    J6 J;
    auto cur = fk(q, left, &J);
    auto e = err(cur, goal);
    if (e.norm() < 1e-7) return true;
    Q dq = J.transpose() * (J * J.transpose() + .000002 * J6::Identity()).ldlt().solve(e);
    if (dq.norm() > .3) dq *= .3 / dq.norm();
    Q next = (q + dq).cwiseMax(l).cwiseMin(h);
    if ((next - q).norm() < 1e-8) return false;
    q = next;
  }
  return false;
}
Pose goals[2] = {
    {V(.058710888335, 0, .1), Eigen::Quaterniond(0, 0, 1, 1).normalized().toRotationMatrix()},
    {V(0, -.058710888335, .1), Eigen::Quaterniond(.5, .5, .5, .5).toRotationMatrix()}};
struct Config {
  V base;
  double yaw, roll, pitch;
  bool swap;
  int flip;
};
Pose torso(const Config& c) {
  M y = rot(2, c.yaw);
  return {c.base + y * (V(0, 0, .07) + rot(0, c.roll) * V(0, 0, .01)),
          y * rot(0, c.roll) * rot(1, c.pitch)};
}
Pose target(const Config& c, int hand, double angle) {
  auto t = torso(c);
  M turn = Eigen::AngleAxisd(angle, hand == 0 ? V(1, 0, 0) : V(0, -1, 0)).toRotationMatrix();
  return {t.r.transpose() * (goals[hand].p - t.p),
          t.r.transpose() * turn * goals[hand].r * rot(0, c.flip & (1 << hand) ? M_PI : 0)};
}

inline Json arr(const Q& q) {
  Json a = Json::array();
  for (auto x : q) a.push_back(x);
  return a;
}
