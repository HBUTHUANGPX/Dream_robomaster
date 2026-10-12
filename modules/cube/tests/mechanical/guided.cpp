#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "replay_structure.hpp"
#include "rm/cube_magnets.hpp"

namespace {
void need(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, const char* message) {
  need(std::isfinite(actual) && std::abs(actual - expected) < 1e-9, message);
}
}  // namespace

// 检查实际生成的 RX guided 场景；只做正运动学，不代替动态还原验收。
int main(int argc, char** argv) {
  try {
    if (argc != 2 && argc != 3)
      throw std::invalid_argument("用法: guided 场景XML [预期魔方质量kg]");
    char error[2048]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(
        mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
    if (!m) throw std::runtime_error(error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
    need(bool(d), "无法创建测试状态");
    constexpr std::string_view faces = "URFDLB";
    need(mechanical_test::center_preload_addresses(m.get(), faces).empty(),
         "输入必须是无 preload 的 guided 模型");
    const int core = mj_name2id(m.get(), mjOBJ_BODY, "mechanical_core");
    const int cj = m->body_jntadr[core];
    need(m->body_jntnum[core] == 1 && m->jnt_type[cj] == mjJNT_FREE,
         "RX 核心必须具有一个 freejoint");
    int free_count = 0, ball_count = 0, shells = 0, first_ball = -1;
    for (int j = 0; j < m->njnt; ++j) free_count += m->jnt_type[j] == mjJNT_FREE;
    need(free_count == 1, "guided RX 只能保留核心一个 freejoint");
    for (int b = 1; b < m->nbody; ++b) {
      const char* name = mj_id2name(m.get(), mjOBJ_BODY, b);
      if (!name || !std::string_view(name).starts_with("piece_")) continue;
      const int j = m->body_jntadr[b];
      for (int k = 0; k < 3; ++k) {
        near(m->body_pos[3 * b + k], 0, "块原点必须与核心同心");
        near(m->jnt_pos[3 * j + k], 0, "关节锚点必须与核心同心");
      }
      const int site = mj_name2id(m.get(), mjOBJ_SITE, (std::string(name) + "_center").c_str());
      need(site >= 0 && m->site_bodyid[site] == b, "块中心 site 缺失或归属错误");
      int nonzero = 0;
      for (int k = 0; k < 3; ++k) nonzero += std::abs(m->site_pos[3 * site + k]) > .001;
      need((nonzero == 1 && m->jnt_type[j] == mjJNT_HINGE) ||
               (nonzero >= 2 && m->jnt_type[j] == mjJNT_BALL),
           "中心应为 hinge，棱角应为 ball");
      int active = 0;
      for (int g = m->body_geomadr[b]; g < m->body_geomadr[b] + m->body_geomnum[b]; ++g) {
        if (!m->geom_contype[g] && !m->geom_conaffinity[g]) continue;
        ++active;
        const char* gn = mj_id2name(m.get(), mjOBJ_GEOM, g);
        need(
            gn && std::string(gn) == std::string(name) + "_shell" && m->geom_type[g] == mjGEOM_MESH,
            "块只能保留 shell mesh 碰撞");
      }
      need(active == 1, "每块必须恰有一个可碰撞 shell");
      shells += active;
      if (m->jnt_type[j] != mjJNT_BALL) continue;
      ++ball_count;
      first_ball = j;
      near(m->dof_frictionloss[m->jnt_dofadr[j]], 0, "ball 不应继承中心轴承摩擦");
      // 非平凡核心位姿下，局部 ball 旋转必须带动块中心绕核心运动。
      mj_resetData(m.get(), d.get());
      const int ca = m->jnt_qposadr[cj], qa = m->jnt_qposadr[j];
      d->qpos[ca] = .003;
      d->qpos[ca + 1] = -.002;
      const mjtNum core_q[4]{std::sqrt(.5), std::sqrt(.5), 0, 0};
      mju_copy4(d->qpos + ca + 3, core_q);
      const mjtNum turn[4]{std::sqrt(.5), 0, 0, std::sqrt(.5)};
      mju_copy4(d->qpos + qa, turn);
      mj_forward(m.get(), d.get());
      mjtNum local[3], rotated[3];
      mju_rotVecQuat(local, m->site_pos + 3 * site, turn);
      mju_rotVecQuat(rotated, local, core_q);
      for (int k = 0; k < 3; ++k)
        near(d->site_xpos[3 * site + k], d->xpos[3 * core + k] + rotated[k],
             "ball 局部旋转或父核心变换错误");
    }
    need(shells == 26 && ball_count == 20, "必须有26个 shell 和20个 ball");
    const bool spheres = mj_name2id(m.get(), mjOBJ_GEOM, "A_left_corner_0_tip") >= 0;
    if (spheres) {
      int sphere_count = 0;
      for (const auto* hand : {"A", "B"})
        for (const auto* side : {"left", "right"}) {
          const std::string prefix = std::string(hand) + "_" + side;
          const int box = mj_name2id(m.get(), mjOBJ_GEOM, (prefix + "_tip").c_str());
          need(box >= 0 && !m->geom_contype[box] && !m->geom_conaffinity[box],
               "四球模式的原方块不得参与碰撞");
          near(m->body_mass[m->geom_bodyid[box]], .002, "四球不得增加指垫与颈部的总质量");
          for (int p = 0; p < m->npair; ++p)
            need(m->pair_geom1[p] != box && m->pair_geom2[p] != box, "显式接触不得重新启用方块");
          for (int i = 0; i < 4; ++i) {
            const int g = mj_name2id(m.get(), mjOBJ_GEOM,
                                     (prefix + "_corner_" + std::to_string(i) + "_tip").c_str());
            need(g >= 0 && m->geom_type[g] == mjGEOM_SPHERE &&
                     m->geom_bodyid[g] == m->geom_bodyid[box],
                 "四球应固定在原指垫刚体上");
            ++sphere_count;
            near(m->geom_pos[3 * g + 1] -
                     std::copysign(m->geom_size[3 * g], m->geom_pos[3 * box + 1]),
                 m->geom_pos[3 * box + 1] -
                     std::copysign(m->geom_size[3 * box + 1], m->geom_pos[3 * box + 1]),
                 "改变球半径不得移动原夹持接触面");
            for (int k = 0; k < 3; ++k)
              need(std::abs(m->geom_pos[3 * g + k] - m->geom_pos[3 * box + k]) +
                           m->geom_size[3 * g] <=
                       m->geom_size[3 * box + k] + 1e-10,
                   "小球超出原有回腕净空包络");
            int pairs = 0;
            for (int p = 0; p < m->npair; ++p)
              pairs += m->pair_geom1[p] == g || m->pair_geom2[p] == g;
            need(pairs == 26, "每个小球应与26块外壳配置摩擦接触");
          }
        }
      need(sphere_count == 16, "两夹爪共应有16个小球");
    }
    for (int g = m->body_geomadr[core]; g < m->body_geomadr[core] + m->body_geomnum[core]; ++g)
      need(!m->geom_contype[g] && !m->geom_conaffinity[g], "核心只能作为视觉几何");
    if (argc == 3) {
      const double expected = std::stod(argv[2]);
      need(std::isfinite(expected) && expected > 0, "预期质量无效");
      double total = m->body_mass[core];
      for (int b = 1; b < m->nbody; ++b) {
        const char* name = mj_id2name(m.get(), mjOBJ_BODY, b);
        if (!name || !std::string_view(name).starts_with("piece_")) continue;
        total += m->body_mass[b];
        near(m->body_mass[b], expected * 3. / 88., "块质量未同步缩放");
        for (int k = 0; k < 3; ++k)
          near(m->body_inertia[3 * b + k], expected / .088 * 1.6e-7, "块惯性未同步缩放");
      }
      near(total, expected, "魔方总质量错误");
      near(m->body_mass[core], expected * 10. / 88., "核心质量未同步缩放");
      for (int k = 0; k < 3; ++k)
        near(m->body_inertia[3 * core + k], expected / .088 * 5e-7, "核心惯性未同步缩放");
    }
    // 显式 pair 也不能把核心或内部保持几何重新激活。
    for (int p = 0; p < m->npair; ++p)
      for (int g : {m->pair_geom1[p], m->pair_geom2[p]}) {
        const char* body = mj_id2name(m.get(), mjOBJ_BODY, m->geom_bodyid[g]);
        need(m->geom_bodyid[g] != core, "显式 pair 不得激活核心碰撞");
        if (body && std::string_view(body).starts_with("piece_")) {
          const char* name = mj_id2name(m.get(), mjOBJ_GEOM, g);
          need(name && std::string_view(name).ends_with("_shell"), "显式 pair 不得激活内部几何");
        }
      }
    rm::cube::CubeMagnets magnets(m.get());
    if (magnets.corner_count() || magnets.edge_count()) {
      need(magnets.corner_count() == 24 && magnets.edge_count() == 24, "磁吸点应为24角加24棱");
      mj_resetData(m.get(), d.get());
      magnets.apply(m.get(), d.get(), .3);
      const double solved_energy = magnets.potential(m.get(), d.get(), .3);
      need(solved_energy < 0, "初始排列磁吸势能应为负");
      // 仅用于区分浮点噪声，不是实验标定的势垒或扭矩阈值。
      constexpr double energy_tolerance = 1e-12, perturbation = .01;
      constexpr double quarter_turn = std::numbers::pi / 2;
      constexpr int scan_steps = 900;  // 每0.1度采样一次，窄于采样间距的势能井仍可能漏检。
      bool magnetic_stability = true;
      std::vector<mjtNum> direction(m->nv, 0);
      auto sample = [&](double angle) {
        mj_resetData(m.get(), d.get());
        mj_integratePos(m.get(), d->qpos, direction.data(), angle);
        magnets.apply(m.get(), d.get(), .3);
        const double energy = magnets.potential(m.get(), d.get(), .3);
        // direction是单位层转角的广义速度，内积给出与层转角共轭的扭矩。
        const double torque = mju_dot(d->qfrc_applied, direction.data(), m->nv);
        need(std::isfinite(energy) && std::isfinite(torque), "磁吸扫描出现非有限值");
        return std::pair{energy, torque};
      };
      int nonrestoring_balls = 0, penetrating_balls = 0;
      double deepest_ball_penetration = 0;
      auto check_restoring = [&](const char* name, int axis, double angle, double reference,
                                 double reference_energy, bool require_restoring = true) {
        const auto [energy, torque] = sample(angle);
        const double work = torque * (angle - reference);
        if (energy - reference_energy <= energy_tolerance || work >= -energy_tolerance) {
          if (!require_restoring) {
            // 单块可在间隙内被吸向接触面。磁吸单独的负刚度不是整体机构失稳判据。
            ++nonrestoring_balls;
            mj_collision(m.get(), d.get());
            double penetration = 0;
            for (int i = 0; i < d->ncon; ++i) {
              const auto& contact = d->contact[i];
              const char* a = mj_id2name(m.get(), mjOBJ_GEOM, contact.geom[0]);
              const char* b = mj_id2name(m.get(), mjOBJ_GEOM, contact.geom[1]);
              if (a && b && std::string_view(a).ends_with("_shell") &&
                  std::string_view(b).ends_with("_shell"))
                penetration = std::max(penetration, -contact.dist);
            }
            penetrating_balls += penetration > 1e-12;
            deepest_ball_penetration = std::max(deepest_ball_penetration, penetration);
            return;
          }
          magnetic_stability = false;
          std::cerr << std::setprecision(12) << "磁吸复位检查失败 对象=" << name << " 轴=" << axis
                    << " 角度rad=" << angle << " 参考角rad=" << reference
                    << " 势能增量J=" << energy - reference_energy << " 虚功J=" << work << '\n';
        }
      };
      int perturbed_balls = 0;
      for (int b = 1; b < m->nbody; ++b) {
        const char* name = mj_id2name(m.get(), mjOBJ_BODY, b);
        if (!name || !std::string_view(name).starts_with("piece_")) continue;
        const int j = m->body_jntadr[b];
        if (m->jnt_type[j] != mjJNT_BALL) continue;
        ++perturbed_balls;
        for (int axis = 0; axis < 3; ++axis) {
          std::fill(direction.begin(), direction.end(), 0);
          direction[m->jnt_dofadr[j] + axis] = 1;
          for (int sign : {-1, 1})
            check_restoring(name, axis, sign * perturbation, 0, solved_energy, false);
        }
      }
      std::cout << "单块磁吸诊断：非恢复方向=" << nonrestoring_balls
                << "，其中外壳穿透=" << penetrating_balls
                << "，最深穿透m=" << deepest_ball_penetration
                << "；仅作接触耦合诊断，不要求磁吸独立稳定全部自由度\n";
      need(perturbed_balls == 20, "磁吸扰动检查必须覆盖20个ball的三个轴和正负方向");
      for (int axis = 0; axis < 3; ++axis)
        for (int side : {-1, 1}) {
          std::fill(direction.begin(), direction.end(), 0);
          int layer_balls = 0;
          for (int b = 1; b < m->nbody; ++b) {
            const char* name = mj_id2name(m.get(), mjOBJ_BODY, b);
            if (!name || !std::string_view(name).starts_with("piece_")) continue;
            const int site =
                mj_name2id(m.get(), mjOBJ_SITE, (std::string(name) + "_center").c_str());
            if (side * m->site_pos[3 * site + axis] < .01) continue;
            const int j = m->body_jntadr[b];
            if (m->jnt_type[j] == mjJNT_BALL) {
              direction[m->jnt_dofadr[j] + axis] = side;
              ++layer_balls;
            } else {
              // 中心没有磁吸site，但仍随整层绕外法向转动。
              direction[m->jnt_dofadr[j]] = side * m->jnt_axis[3 * j + axis];
            }
          }
          need(layer_balls == 8, "层扫描必须包含4角4棱");
          const std::string layer = std::string(axis == 0   ? "X"
                                                : axis == 1 ? "Y"
                                                            : "Z") +
                                    (side > 0 ? "+" : "-");
          for (double endpoint : {0., quarter_turn}) {
            const double energy = sample(endpoint).first;
            near(energy, solved_energy, "层转动两端必须具有同等磁吸势能");
            for (int v = 0; v < m->nv; ++v)
              near(d->qfrc_applied[v], 0, "对齐排列不应存在磁吸净广义力");
            for (int sign : {-1, 1})
              check_restoring(layer.c_str(), axis, endpoint + sign * perturbation, endpoint,
                              energy);
          }
          // 合法曲线先上升，再下降；中间允许零斜率平台。
          // 一旦下降后又显著回升，就存在额外势能井（包含平底井）。
          double peak_layer_torque = 0, peak_angle = 0;
          double running_peak = solved_energy, running_min = solved_energy, min_angle = 0;
          bool descending = false, extra_well = false;
          double well_angle = 0, well_depth = 0;
          for (int k = 0; k <= scan_steps; ++k) {
            const double angle = quarter_turn * k / scan_steps;
            const auto [energy, torque] = sample(angle);
            if (std::abs(torque) > peak_layer_torque) {
              peak_layer_torque = std::abs(torque);
              peak_angle = angle;
            }
            if (!descending) {
              running_peak = std::max(running_peak, energy);
              if (running_peak - energy > energy_tolerance) {
                descending = true;
                running_min = energy;
                min_angle = angle;
              }
            } else if (energy < running_min) {
              running_min = energy;
              min_angle = angle;
            } else if (energy - running_min > energy_tolerance) {
              extra_well = true;
              if (energy - running_min > well_depth) {
                well_angle = min_angle;
                well_depth = energy - running_min;
              }
            }
          }
          std::cout << std::setprecision(12) << "磁吸层扫描 层=" << layer
                    << " 单对峰值力N=0.3 采样间距deg=0.1 peak_layer_torque=" << peak_layer_torque
                    << " 单位=N*m 峰值角度rad=" << peak_angle << " 额外势能井=" << extra_well
                    << '\n';
          if (extra_well) {
            magnetic_stability = false;
            std::cerr << "额外磁吸势能井 层=" << layer << " 井底角度rad=" << well_angle
                      << " 回升势能J=" << well_depth << '\n';
          }
        }
      need(magnetic_stability, "磁吸局部复位或层转动势能检查失败，详见上述观测");
    }
    // 缺失一个 ball 的无 preload 模型必须被 replay 分类器拒绝。
    m->jnt_type[first_ball] = mjJNT_HINGE;
    bool rejected = false;
    try {
      mechanical_test::center_preload_addresses(m.get(), faces);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    m->jnt_type[first_ball] = mjJNT_BALL;
    need(rejected, "不完整的 guided 结构未被拒绝");
    std::cout << "guided 结构、20块正运动学及无 preload 分类检查通过；未执行动力学验收\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
