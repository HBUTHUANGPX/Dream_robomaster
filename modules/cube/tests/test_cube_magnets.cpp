#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "rm/cube_magnets.hpp"

namespace {
using Vec = std::array<mjtNum, 3>;
using Quat = std::array<mjtNum, 4>;
constexpr Quat identity{1, 0, 0, 0};
constexpr double strength = .7;

void need(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const char* message) {
  need(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
void near_vec(const Vec& actual, const Vec& expected, const char* message) {
  for (int k = 0; k < 3; ++k) near(actual[k], expected[k], 1e-10, message);
}
Vec rotate(const Vec& value, const Quat& q) {
  Vec result;
  mju_rotVecQuat(result.data(), value.data(), q.data());
  return result;
}

struct Fixture {
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> d{nullptr, mj_deleteData};
  std::array<int, 2> bodies{}, qpos{}, dofs{};

  explicit Fixture(int count = 24) {
    // 两个独立自由刚体。仅0号site互相靠近，其他site远离作用范围。
    // site偏离质心，避免只测到平移而漏掉力矩或雅可比刷新错误。
    std::ostringstream xml;
    xml << std::setprecision(17)
        << "<mujoco><option gravity='0 0 0'/><worldbody>"
           "<site name='unrelated_site' pos='0 0 0' size='.0001'/>";
    for (int side = 0; side < 2; ++side) {
      const char* kind = side == 0 ? "corner" : "edge";
      xml << "<body name='" << kind
          << "'><freejoint/>"
             "<inertial pos='0 0 0' mass='.01' diaginertia='.00001 .00001 .00001'/>";
      for (int i = 0; i < count; ++i) {
        xml << "<site name='cube_" << kind << "_magnet_" << i << "' size='.0001' pos='";
        if (i == 0)
          xml << ".001 .002 .0007";
        else
          xml << .02 * i << ' ' << (side == 0 ? .1 : -.1) << " 0";
        xml << "'/>";
      }
      xml << "</body>";
    }
    xml << "</worldbody></mujoco>";
    char error[1024]{};
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
        mj_parseXMLString(xml.str().c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
    need(bool(spec), error);
    m.reset(mj_compile(spec.get(), nullptr));
    need(bool(m), mjs_getError(spec.get()));
    d.reset(mj_makeData(m.get()));
    need(bool(d), "无法创建磁吸测试状态");
    for (int side = 0; side < 2; ++side) {
      bodies[side] = mj_name2id(m.get(), mjOBJ_BODY, side == 0 ? "corner" : "edge");
      const int joint = m->body_jntadr[bodies[side]];
      qpos[side] = m->jnt_qposadr[joint];
      dofs[side] = m->jnt_dofadr[joint];
    }
  }

  // 故意不刷新缓存：apply必须自行处理积分后更新的qpos。
  void pose(const Vec& separation, const Quat& q = identity, const Vec& shift = {}) {
    for (int side = 0; side < 2; ++side) {
      const Vec p = side == 0 ? Vec{} : rotate(separation, q);
      for (int k = 0; k < 3; ++k) d->qpos[qpos[side] + k] = p[k] + shift[k];
      mju_copy4(d->qpos + qpos[side] + 3, q.data());
    }
  }
  void clear_force() { mju_zero(d->qfrc_applied, m->nv); }
  Vec force(int side) const {
    const auto* f = d->qfrc_applied + dofs[side];
    return {f[0], f[1], f[2]};
  }
  Vec torque(int side) const {
    // freejoint的平移广义力为世界坐标，旋转广义力为刚体局部坐标。
    Vec result;
    mju_rotVecQuat(result.data(), d->qfrc_applied + dofs[side] + 3, d->xquat + 4 * bodies[side]);
    return result;
  }
  void check_balance() const {
    const Vec a = force(0), b = force(1);
    Vec torque_sum{};
    for (int k = 0; k < 3; ++k) near(a[k] + b[k], 0, 1e-11, "磁吸违反作用反作用");
    for (int side = 0; side < 2; ++side) {
      const Vec f = force(side), t = torque(side);
      Vec moment;
      mju_cross(moment.data(), d->xpos + 3 * bodies[side], f.data());
      for (int k = 0; k < 3; ++k) torque_sum[k] += t[k] + moment[k];
    }
    near_vec(torque_sum, {}, "磁吸对世界原点产生净力矩");
  }
};

void check_forces() {
  Fixture f;
  rm::cube::CubeMagnets magnets(f.m.get());
  need(magnets.corner_count() == 24 && magnets.edge_count() == 24, "未正确收集24+24个磁吸site");
  const Vec separation{.0024, .0012, -.0009};
  f.pose(separation);
  const std::vector<mjtNum> initial_qpos(f.d->qpos, f.d->qpos + f.m->nq);
  magnets.apply(f.m.get(), f.d.get(), strength);
  need(std::equal(initial_qpos.begin(), initial_qpos.end(), f.d->qpos), "施力改变了qpos");
  f.check_balance();
  need(mju_dot3(f.force(0).data(), separation.data()) > 0, "磁吸必须吸引而不是排斥");
  need(mju_norm3(f.torque(0).data()) > 1e-6, "偏心磁吸未产生刚体力矩");
  const Vec force = f.force(0), torque = f.torque(0);
  const double energy = magnets.potential(f.m.get(), f.d.get(), strength);
  need(energy < 0, "作用范围内势能应为负");
  const std::vector<mjtNum> applied(f.d->qfrc_applied, f.d->qfrc_applied + f.m->nv);
  // 不得清除已有外力，重复调用应累加。
  for (int i = 0; i < f.m->nv; ++i) f.d->qfrc_applied[i] = .03 * (i + 1);
  magnets.apply(f.m.get(), f.d.get(), strength);
  magnets.apply(f.m.get(), f.d.get(), strength);
  for (int i = 0; i < f.m->nv; ++i)
    near(f.d->qfrc_applied[i], .03 * (i + 1) + 2 * applied[i], 1e-12, "磁吸覆盖了已有外力");

  Vec axis{1, 2, 3};
  mju_normalize3(axis.data());
  Quat q;
  mju_axisAngle2Quat(q.data(), axis.data(), .83);
  f.pose(separation, q, {.2, -.3, .4});
  f.clear_force();
  magnets.apply(f.m.get(), f.d.get(), strength);
  near_vec(f.force(0), rotate(force, q), "整体旋转后吸力不协变或缓存未刷新");
  near_vec(f.torque(0), rotate(torque, q), "整体旋转后力矩不协变或雅可比未刷新");
  near(magnets.potential(f.m.get(), f.d.get(), strength), energy, 1e-13, "刚性变换改变了势能");
  f.check_balance();

  // 在旋转后的位姿上对全部12个广义速度方向差分，覆盖平移力和局部旋转力矩。
  const std::vector<mjtNum> base(f.d->qpos, f.d->qpos + f.m->nq);
  const std::vector<mjtNum> generalized(f.d->qfrc_applied, f.d->qfrc_applied + f.m->nv);
  std::vector<mjtNum> direction(f.m->nv, 0);
  constexpr double h = 1e-7;
  for (int i = 0; i < f.m->nv; ++i) {
    direction[i] = 1;
    double energies[2];
    for (int sign = 0; sign < 2; ++sign) {
      mju_copy(f.d->qpos, base.data(), f.m->nq);
      mj_integratePos(f.m.get(), f.d->qpos, direction.data(), sign == 0 ? h : -h);
      mj_kinematics(f.m.get(), f.d.get());
      energies[sign] = magnets.potential(f.m.get(), f.d.get(), strength);
    }
    near(generalized[i], -(energies[0] - energies[1]) / (2 * h), 2e-8,
         "磁吸广义力与势能负梯度不一致");
    direction[i] = 0;
  }
}

void check_range_and_pairing() {
  Fixture f;
  rm::cube::CubeMagnets magnets(f.m.get());
  constexpr double range = rm::cube::CubeMagnets::range_m;
  f.pose({range / std::sqrt(5.), 0, 0});
  magnets.apply(f.m.get(), f.d.get(), strength);
  near(mju_norm3(f.force(0).data()), strength, 1e-12, "单对峰值吸力未归一化");
  for (double distance : {0., range, range + .001}) {
    f.pose({distance, 0, 0});
    f.clear_force();
    magnets.apply(f.m.get(), f.d.get(), strength);
    for (int i = 0; i < f.m->nv; ++i)
      near(f.d->qfrc_applied[i], 0, 1e-12, "重合点或远距磁吸力不为零");
    const double expected = distance == 0 ? -strength * range / (6 / std::sqrt(5.) * .64) : 0;
    near(magnets.potential(f.m.get(), f.d.get(), strength), expected, 1e-14,
         "重合点或截断处势能错误");
  }
  f.pose({.003, 0, 0});
  f.clear_force();
  magnets.apply(f.m.get(), f.d.get(), strength);
  const Vec force = f.force(0);
  const double energy = magnets.potential(f.m.get(), f.d.get(), strength);
  // 让edge_0靠近corner_23，验证配对不是按site编号固定绑定。
  const int corner = mj_name2id(f.m.get(), mjOBJ_SITE, "cube_corner_magnet_23");
  const int edge = mj_name2id(f.m.get(), mjOBJ_SITE, "cube_edge_magnet_0");
  Vec separation;
  for (int k = 0; k < 3; ++k)
    separation[k] = f.m->site_pos[3 * corner + k] - f.m->site_pos[3 * edge + k];
  separation[0] += .003;
  f.pose(separation);
  f.clear_force();
  magnets.apply(f.m.get(), f.d.get(), strength);
  near_vec(f.force(0), force, "未对所有角棱site执行近距配对");
  near(magnets.potential(f.m.get(), f.d.get(), strength), energy, 1e-13, "更换邻近site后势能错误");
  f.check_balance();

  // 强度0必须保留外力和缓存；potential只读，不能修改外力或位姿。
  const std::vector<mjtNum> cache(f.d->site_xpos, f.d->site_xpos + 3 * f.m->nsite);
  f.pose({.004, 0, 0});
  const std::vector<mjtNum> qpos(f.d->qpos, f.d->qpos + f.m->nq);
  const std::vector<mjtNum> forces(f.d->qfrc_applied, f.d->qfrc_applied + f.m->nv);
  magnets.apply(f.m.get(), f.d.get(), 0);
  near(magnets.potential(f.m.get(), f.d.get(), 0), 0, 0, "强度0时势能不为零");
  magnets.potential(f.m.get(), f.d.get(), strength);
  need(std::equal(cache.begin(), cache.end(), f.d->site_xpos), "零强度或只读势能刷新了缓存");
  need(std::equal(qpos.begin(), qpos.end(), f.d->qpos), "零强度或只读势能改变了qpos");
  need(std::equal(forces.begin(), forces.end(), f.d->qfrc_applied), "零强度或只读势能改变了外力");
  for (double invalid :
       {-1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    bool rejected = false;
    try {
      magnets.apply(f.m.get(), f.d.get(), invalid);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    need(rejected, "未拒绝非法磁吸强度");
  }
}
}  // namespace

int main() {
  try {
    check_forces();
    check_range_and_pairing();
    Fixture old_scene(0);
    rm::cube::CubeMagnets magnets(old_scene.m.get());
    need(magnets.corner_count() == 0 && magnets.edge_count() == 0, "旧场景site计数错误");
    old_scene.d->qfrc_applied[0] = .125;
    magnets.apply(old_scene.m.get(), old_scene.d.get(), strength);
    near(old_scene.d->qfrc_applied[0], .125, 0, "旧场景外力被改写");
    near(magnets.potential(old_scene.m.get(), old_scene.d.get(), strength), 0, 0,
         "无site场景的磁吸势能不为零");
    std::cout << "块间磁吸守恒、势能梯度、刚性变换、作用范围及状态保留检查通过\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
