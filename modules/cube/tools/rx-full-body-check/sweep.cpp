#include <map>

#include "kinematics.hpp"
namespace rm::cube {
void set_probe_hand(mjModel*, mjData*, bool, double);
}
int run(int argc, char** argv) {
  if (argc != 4) throw std::runtime_error("用法：rx-full-body-check 场景.xml 候选.json 结果.json");
  Json j;
  std::ifstream candidate(argv[2]);
  if (!candidate) throw std::runtime_error("无法读取候选文件");
  candidate >> j;
  if (j.at("swap").get<bool>() || j.at("A").at("arm") != "right" || j.at("B").at("arm") != "left" ||
      j.at("waist_yaw").get<double>() != 0 || j.at("A").at("sign") != -1 ||
      j.at("B").at("sign") != 1)
    throw std::runtime_error("本工具只检查 A右臂负90度、B左臂正90度、腰yaw为0 的已保存候选");
  for (int h = 0; h < 2; h++) {
    const auto& t = j.at("targets").at(h ? "B" : "A");
    for (int i = 0; i < 3; i++) goals[h].p[i] = t.at("position").at(i).get<double>();
    const auto& q = t.at("quaternion");
    Eigen::Quaterniond r(q.at(0).get<double>(), q.at(1).get<double>(), q.at(2).get<double>(),
                         q.at(3).get<double>());
    if (std::abs(r.norm() - 1) > 1e-6) throw std::runtime_error("工具目标四元数必须归一化");
    goals[h].r = r.toRotationMatrix();
  }

  Config c;
  c.base = V(j["base_xyz"][0], j["base_xyz"][1], j["base_xyz"][2]);
  c.yaw = j["base_yaw"];
  c.roll = j["waist_roll"];
  c.pitch = j["waist_pitch"];
  c.swap = j["swap"];
  c.flip = j["flip_bits"];
  char error[2048];
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
  if (!model) throw std::runtime_error(error);
  auto* m = model.get();
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(m), mj_deleteData);
  auto* d = data.get();
  auto body_id = [&](const char* name) {
    int b = mj_name2id(m, mjOBJ_BODY, name);
    if (b < 0) throw std::runtime_error(std::string("场景缺少body：") + name);
    return b;
  };
  body_id("A_r_gripper_link");
  body_id("B_l_gripper_link");
  body_id("waist_pitch_link");
  const int base = body_id("base_link");
  for (int i = 0; i < 3; i++) m->body_pos[3 * base + i] = c.base[i];
  Eigen::Quaterniond r(rot(2, c.yaw));
  m->body_quat[4 * base] = r.w();
  m->body_quat[4 * base + 1] = r.x();
  m->body_quat[4 * base + 2] = r.y();
  m->body_quat[4 * base + 3] = r.z();
  auto setq = [&](const std::string& name, double value) {
    int id = mj_name2id(m, mjOBJ_JOINT, name.c_str());
    if (id < 0) throw std::runtime_error(name);
    if (!std::isfinite(value) || value < m->jnt_range[2 * id] || value > m->jnt_range[2 * id + 1])
      throw std::runtime_error("关节超限：" + name);
    d->qpos[m->jnt_qposadr[id]] = value;
  };
  setq("waist_yaw_joint", 0);
  setq("waist_roll_joint", c.roll);
  setq("waist_pitch_joint", c.pitch);
  const char* suffix[] = {"shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint",
                          "elbow_joint",          "wrist_roll_joint",    "wrist_pitch_joint"};
  std::vector<Q> path[2];
  double margin = 10, max_delta = 0;
  for (int h = 0; h < 2; h++) {
    Q q;
    for (int k = 0; k < 6; k++) q[k] = j[h ? "B" : "A"]["q"][0][k];
    for (int a = 0; a <= 90; a++) {
      Q prev = q;
      if (!ik(target(c, h, (h ? 1 : -1) * a * M_PI / 180), h == 1, q))
        throw std::runtime_error("1度路径不可达");
      path[h].push_back(q);
      margin = std::min(margin, std::min((q - lo(h == 1)).minCoeff(), (hi(h == 1) - q).minCoeff()));
      max_delta = std::max(max_delta, (q - prev).norm());
    }
  }
  std::vector<std::string> bodies(m->nbody);
  std::vector<int> group(m->nbody);
  int torso = -1;
  for (int b = 0; b < m->nbody; b++) {
    const char* n = mj_id2name(m, mjOBJ_BODY, b);
    bodies[b] = n ? n : "";
    if (bodies[b].starts_with("A_") || bodies[b].starts_with("B_"))
      group[b] = 2;
    else if (b > 0 && (bodies[b] == "base_link" || bodies[b].ends_with("_link")))
      group[b] = 1;
    if (bodies[b] == "waist_pitch_link") torso = b;
  }
  Json records = Json::array();
  double max_position = 0, max_rotation = 0;
  int total = 0;
  auto test = [&](int hand, int degree, double motor_a, double motor_b, const std::string& mode) {
    for (int h = 0; h < 2; h++) {
      const auto& q = path[h][h == hand ? degree : 0];
      for (int k = 0; k < 6; k++) setq(std::string(h ? "l_" : "r_") + suffix[k], q[k]);
      rm::cube::set_probe_hand(m, d, h == 1, h ? motor_b : motor_a);
    }
    mj_forward(m, d);
    total++;
    for (int h = 0; h < 2; h++) {
      int b = mj_name2id(m, mjOBJ_BODY, h ? "B_l_gripper_link" : "A_r_gripper_link");
      Eigen::Map<const V> p(d->xpos + 3 * b);
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(d->xmat + 9 * b);
      auto t = target(c, h, (h == hand ? (h ? 1 : -1) * degree : 0) * M_PI / 180);
      auto frame = ::torso(c);
      max_position = std::max(max_position, (p - goals[h].p).norm());
      max_rotation =
          std::max(max_rotation, Eigen::AngleAxisd((frame.r * t.r) * R.transpose()).angle());
    }
    std::map<std::string, double> unwanted, cube;
    for (int i = 0; i < d->ncon; i++) {
      const auto& ct = d->contact[i];
      if (ct.dist >= -1e-5) continue;
      int b1 = m->geom_bodyid[ct.geom[0]], b2 = m->geom_bodyid[ct.geom[1]];
      if (!group[b1] && !group[b2]) continue;
      const auto key = bodies[b1] + " / " + bodies[b2];
      auto& target = (group[b1] == 0 || group[b2] == 0) ? cube : unwanted;
      target[key] = std::min(target[key], double(ct.dist));
    }
    double clearance = .5;
    for (int g = 0; g < m->ngeom; g++)
      if (m->geom_bodyid[g] == torso && m->geom_contype[g])
        for (int t = 0; t < m->ngeom; t++)
          if (group[m->geom_bodyid[t]] == 2 && m->geom_contype[t])
            clearance = std::min(clearance, double(mj_geomDistance(m, d, g, t, .5, nullptr)));
    records.push_back({{"moving_hand", hand ? "B" : "A"},
                       {"angle_degrees", (hand ? 1 : -1) * degree},
                       {"opposite_angle_degrees", 0},
                       {"motor_A_rad", motor_a},
                       {"motor_B_rad", motor_b},
                       {"mode", mode},
                       {"waist_to_grippers_min_distance_m", clearance},
                       {"robot_gripper_penetrations_m", unwanted},
                       {"robot_or_gripper_vs_scene_penetrations_m", cube}});
  };
  for (int hand = 0; hand < 2; hand++)
    for (double a : {-2.25, -1.20})
      for (double b : {-2.25, -1.20})
        for (int degree = 0; degree <= 90; degree++)
          test(hand, degree, a, b, "单手1度路径；开闭端点组合");
  for (int hand = 0; hand < 2; hand++)
    for (int degree : {0, 90})
      for (int moving_jaw = 0; moving_jaw < 2; moving_jaw++)
        for (double other : {-2.25, -1.20})
          for (int k = 1; k < 10; k++) {
            double motor = -2.25 + 1.05 * k / 10;
            test(hand, degree, moving_jaw == 0 ? motor : other, moving_jaw == 1 ? motor : other,
                 "腕角端点；夹爪开闭10等分");
          }
  double minclear = .5, maxpenetration = 0;
  int bad = 0;
  Json pairs = Json::object();
  for (const auto& v : records) {
    minclear = std::min(minclear, double(v["waist_to_grippers_min_distance_m"]));
    if (!v["robot_gripper_penetrations_m"].empty()) bad++;
    for (const auto& [key, value] : v["robot_gripper_penetrations_m"].items()) {
      double x = value;
      maxpenetration = std::max(maxpenetration, -x);
      if (!pairs.contains(key) || x < double(pairs[key])) pairs[key] = x;
    }
  }
  Json result = {{"candidate", argv[2]},
                 {"scene", argv[1]},
                 {"mujoco", mj_versionString()},
                 {"cases", total},
                 {"robot_gripper_collision_cases", bad},
                 {"maximum_robot_gripper_penetration_m", maxpenetration},
                 {"collision_pairs", pairs},
                 {"waist_to_grippers_min_distance_m", minclear},
                 {"maximum_tool_position_error_m", max_position},
                 {"maximum_tool_rotation_error_rad", max_rotation},
                 {"minimum_arm_joint_limit_margin_rad", margin},
                 {"maximum_1deg_joint_vector_step_rad", max_delta},
                 {"scope",
                  "输入完整场景静态运动学；对侧腕0，闭环夹爪解析位置；记录魔方接触但不判为避障失败"
                  "；不调用 mj_step，不代表动态通过或连续碰撞证明"},
                 {"records", records}};
  result["paths_1deg"] = Json::object();
  for (int h = 0; h < 2; h++) {
    Json a = Json::array();
    for (const auto& q : path[h]) a.push_back(arr(q));
    result["paths_1deg"][h ? "B" : "A"] = a;
  }
  const bool passed = bad == 0 && max_position < 1e-6 && max_rotation < 1e-5 && margin >= 0;
  result["passed"] = passed;
  std::ofstream output(argv[3]);
  output << result.dump(2) << "\n";
  output.close();
  if (!output) throw std::runtime_error("无法写入结果文件");
  result.erase("records");
  result.erase("paths_1deg");
  std::cout << result.dump(2) << std::endl;
  return passed ? 0 : 2;
}

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
