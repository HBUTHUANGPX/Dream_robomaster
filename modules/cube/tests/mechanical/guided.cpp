#include <mujoco/mujoco.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

#include "replay_structure.hpp"

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
