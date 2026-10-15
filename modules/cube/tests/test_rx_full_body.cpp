#include <fstream>
#include <iostream>

#include "mechanical_rx.hpp"
#include "rm/rx_gripper.hpp"
#include "rx_full_body.hpp"
int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("需要仓库根目录");
    const std::filesystem::path root = argv[1];
    std::string scene = rm::cube::mechanical_rx_scene(
        "<mujoco><compiler angle='radian'/><option timestep='.0005' "
        "integrator='implicitfast'/><default><geom solref='.001 1' contype='16' "
        "conaffinity='31'/></default><worldbody/><actuator/></mujoco>",
        root / "assets/rx_gripper/mujoco_linkage_v5", .002, 1.2, .008, .074, true);
    scene = rm::cube::rx_full_body_scene(scene, root / "assets/rx_full_body");
    char error[2048]{};
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
        mj_parseXMLString(scene.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
    if (!spec) throw std::runtime_error(error);
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(mj_compile(spec.get(), nullptr),
                                                          mj_deleteModel);
    if (!m) throw std::runtime_error(mjs_getError(spec.get()));
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
    if (m->nu != 31 || mj_name2id(m.get(), mjOBJ_JOINT, "A_yaw") >= 0 ||
        mj_name2id(m.get(), mjOBJ_JOINT, "B_yaw") >= 0)
      throw std::runtime_error("整机不能保留虚拟腕轴");
    rm::cube::initialize_rx_grippers(m.get(), d.get());
    rm::cube::RxFullBody body(m.get(), d.get(), root / "assets/rx_full_body");
    auto report = body.report(d.get());
    for (auto hand : {"A", "B"})
      if (report[hand]["position_error_m"].get<double>() > 1e-6 ||
          report[hand]["orientation_error_rad"].get<double>() > 1e-6)
        throw std::runtime_error("真实手臂零位与规划坐标不一致");
    for (auto side : {"l", "r"}) {
      std::string prefix = side;
      int wrist = mj_name2id(m.get(), mjOBJ_BODY, (prefix + "_wrist_pitch_link").c_str());
      bool attached = false;
      for (int b = 1; b < m->nbody; b++)
        if (m->body_parentid[b] == wrist) {
          const char* n = mj_id2name(m.get(), mjOBJ_BODY, b);
          attached |= n && std::string_view(n).ends_with("_gripper_link");
        }
      if (!attached) throw std::runtime_error("夹爪未连接原腕关节");
    }
    int base = mj_name2id(m.get(), mjOBJ_BODY, "base_link");
    for (int g = m->body_geomadr[base]; g < m->body_geomadr[base] + m->body_geomnum[base]; g++)
      if (m->geom_contype[g] && std::abs(m->geom_solref[g * mjNREF] - .02) > 1e-12)
        throw std::runtime_error("机身继承了魔方接触参数");
    if (!report["contacts"].empty()) throw std::runtime_error("抓持零位存在穿模");
    const std::vector<mjtNum> initial(d->qpos, d->qpos + m->nq);
    for (int hand = 0; hand < 2; ++hand) {
      const int sign = hand == 0 ? -1 : 1;
      for (int degree = 1; degree <= 90; ++degree)
        body.reference(hand, sign * degree * rm::cube::pi / 180, .05);
      for (int degree = 89; degree >= 0; --degree)
        body.reference(hand, sign * degree * rm::cube::pi / 180, .05);
    }
    if (!std::equal(initial.begin(), initial.end(), d->qpos))
      throw std::runtime_error("参考轨迹不应直接修改物理状态");
    bool rejected = false;
    try {
      body.reference(0, rm::cube::pi / 2, .05);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    if (!rejected) throw std::runtime_error("未拒绝超出已验证角度的指令");
    rm::cube::RxFullBody centered(m.get(), d.get(), root / "assets/rx_full_body", 4);
    int core_joint = mj_name2id(m.get(), mjOBJ_JOINT, "mechanical_core_free");
    if (core_joint < 0) throw std::runtime_error("缺少魔方核心自由关节");
    d->qpos[m->jnt_qposadr[core_joint]] = -.0004;
    mj_forward(m.get(), d.get());
    const std::vector<mjtNum> displaced(d->qpos, d->qpos + m->nq);
    for (int i = 0; i < 1500; ++i) centered.control(d.get());
    const auto correction = centered.report(d.get()).at("center_offset_m");
    if (correction[0].get<double>() < .0009 || correction[0].get<double>() > .001000001)
      throw std::runtime_error("中心偏移应反向补偿并限制在1毫米内");
    if (!std::equal(displaced.begin(), displaced.end(), d->qpos))
      throw std::runtime_error("中心补偿不得直接修正物体或关节状态");
    std::cout << report.dump(2) << "\n全身连接、原生执行器、目标位姿和接触参数隔离检查通过\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
