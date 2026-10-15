#include "rx_full_body.hpp"

#include <libxml/parser.h>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <fstream>
#include <functional>
#include <iomanip>

#include "rm/torque_speed.hpp"
namespace rm::cube {
namespace {
using N = xmlNodePtr;
std::string get(N n, const char* k) {
  auto p = xmlGetProp(n, BAD_CAST k);
  std::string s = p ? (char*)p : "";
  xmlFree(p);
  return s;
}
void set(N n, const char* k, const std::string& v) {
  xmlSetProp(n, BAD_CAST k, BAD_CAST v.c_str());
}
N child(N n, const char* tag) {
  for (auto p = n->children; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE && xmlStrEqual(p->name, BAD_CAST tag)) return p;
  return nullptr;
}
void visit(N n, const std::function<void(N)>& f) {
  for (auto p = n; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE) {
      f(p);
      visit(p->children, f);
    }
}
N named(N n, const std::string& name) {
  N found = nullptr;
  visit(n, [&](N p) {
    if (get(p, "name") == name) found = p;
  });
  if (!found) throw std::runtime_error("整机缺少节点: " + name);
  return found;
}
void erase(N n) {
  xmlUnlinkNode(n);
  xmlFreeNode(n);
}
std::string values(const Json& a) {
  std::ostringstream s;
  s << std::setprecision(17);
  for (const auto& v : a) s << v.get<double>() << ' ';
  return s.str();
}
std::string quat(const Mat& r) {
  Eigen::Quaterniond q(r);
  return values(Json::array({q.w(), q.x(), q.y(), q.z()}));
}
Json pose(const std::filesystem::path& bundle) {
  Json j;
  std::ifstream f(bundle / "grasp_pose.json");
  if (!f) throw std::runtime_error("缺少整机抓持姿态 grasp_pose.json");
  f >> j;
  return j;
}
Mat matrix(const mjtNum* p) {
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(p);
}
int id(const mjModel* m, mjtObj type, const std::string& name) {
  int i = mj_name2id(m, type, name.c_str());
  if (i < 0) throw std::runtime_error("整机缺少对象: " + name);
  return i;
}
const std::array<std::string, 6> suffixes{"shoulder_pitch_joint", "shoulder_roll_joint",
                                          "shoulder_yaw_joint",   "elbow_joint",
                                          "wrist_roll_joint",     "wrist_pitch_joint"};
}  // namespace
std::string rx_full_body_scene(const std::string& scene, const std::filesystem::path& bundle) {
  auto cfg = pose(bundle);
  std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)> doc(
      xmlReadMemory(scene.data(), scene.size(), "scene.xml", nullptr, XML_PARSE_NONET), xmlFreeDoc),
      robot(xmlReadFile((bundle / "robot.xml").c_str(), nullptr, XML_PARSE_NONET), xmlFreeDoc);
  if (!doc || !robot) throw std::runtime_error("整机XML读取失败");
  auto root = xmlDocGetRootElement(doc.get()), src = xmlDocGetRootElement(robot.get()),
       world = child(root, "worldbody");
  std::array<N, 2> wrists{named(world, "A_wrist"), named(world, "B_wrist")},
      grips{named(world, "A_l_gripper_link"), named(world, "B_r_gripper_link")};
  // 保存软件动作的零位坐标；这些site不增加关节或物理约束。
  for (int h = 0; h < 2; h++) {
    auto site = xmlNewChild(world, nullptr, BAD_CAST "site", nullptr);
    set(site, "name", std::string(h ? "B" : "A") + "_fullbody_goal");
    std::istringstream in(get(grips[h], "pos"));
    Vec p;
    in >> p.x() >> p.y() >> p.z();
    p.z() += .1;
    set(site, "pos", values(Json::array({p.x(), p.y(), p.z()})));
    std::istringstream qi(get(grips[h], "quat"));
    Eigen::Quaterniond q;
    qi >> q.w() >> q.x() >> q.y() >> q.z();
    Mat r = q.toRotationMatrix() *
            Eigen::AngleAxisd(cfg.at("tool_flip_rad")[h].get<double>(), Vec::UnitX())
                .toRotationMatrix();
    set(site, "quat", quat(r));
    set(site, "size", ".001");
    set(site, "group", "5");
  }
  const bool swapped = cfg.at("a_is_right").get<bool>();
  if (swapped)
    visit(root, [&](N n) {
      for (auto a = n->properties; a; a = a->next) {
        std::string key = (char*)a->name, v = get(n, key.c_str());
        if (v == "A_wrist" || v == "B_wrist" || v == "A_yaw" || v == "B_yaw" ||
            v == "A_yaw_drive" || v == "B_yaw_drive" || v.ends_with("_fullbody_goal"))
          continue;
        if (v.starts_with("A_"))
          v[0] = 'B';
        else if (v.starts_with("B_"))
          v[0] = 'A';
        else
          continue;
        set(n, key.c_str(), v);
      }
    });
  for (int h = 0; h < 2; h++) {
    const bool left = h == 0;
    auto wrist = named(src, left ? "l_wrist_pitch_link" : "r_wrist_pitch_link");
    xmlUnlinkNode(grips[h]);
    xmlAddChild(wrist, xmlDocCopyNode(grips[h], robot.get(), 1));
    xmlFreeNode(grips[h]);
    auto grip = wrist->last;
    set(grip, "pos", left ? ".11235 .02975 0" : ".11235 -.02975 0");
    set(grip, "quat", "1 0 0 0");
  }
  for (auto w : wrists) erase(w);
  for (auto name : {"A_yaw_drive", "B_yaw_drive"}) erase(named(root, name));
  visit(child(src, "asset"), [&](N n) {
    if (xmlStrEqual(n->name, BAD_CAST "mesh"))
      set(n, "file", std::filesystem::canonical(bundle / "meshes" / get(n, "file")).string());
  });
  auto own_default = child(child(src, "default"), "default");
  auto geom_default = xmlNewChild(own_default, nullptr, BAD_CAST "geom", nullptr);
  for (auto [key, value] :
       std::initializer_list<std::pair<const char*, const char*>>{{"contype", "1"},
                                                                  {"conaffinity", "1"},
                                                                  {"condim", "3"},
                                                                  {"margin", "0"},
                                                                  {"gap", "0"},
                                                                  {"friction", "1 .005 .0001"},
                                                                  {"solref", ".02 1"},
                                                                  {"solimp", ".9 .95 .001"}})
    set(geom_default, key, value);
  auto base = named(src, "base_link");
  set(base, "pos", values(cfg.at("base_position_m")));
  set(base, "quat", values(cfg.at("base_quaternion")));
  for (auto tag : {"asset", "default", "contact", "actuator", "worldbody"}) {
    auto from = child(src, tag), to = child(root, tag);
    if (!from) continue;
    if (!to) to = xmlNewChild(root, nullptr, BAD_CAST tag, nullptr);
    for (auto n = from->children; n; n = n->next)
      if (n->type == XML_ELEMENT_NODE) xmlAddChild(to, xmlDocCopyNode(n, doc.get(), 1));
  }
  // 所有机身关节保留；控制目标可含重力前馈，但实际输出仍受原effort限制。
  visit(child(root, "actuator"), [&](N n) {
    if (get(n, "name").ends_with("_joint_drive")) {
      set(n, "kp", "1500");
      set(n, "kv", "35");
      set(n, "ctrllimited", "false");
      xmlUnsetProp(n, BAD_CAST "ctrlrange");
    }
  });
  xmlChar* data = nullptr;
  int size = 0;
  xmlDocDumpFormatMemory(doc.get(), &data, &size, 1);
  std::string out((char*)data, size);
  xmlFree(data);
  return out;
}
RxFullBody::RxFullBody(mjModel* m, mjData* d, const std::filesystem::path& bundle)
    : m_(m),
      reference_(mj_makeData(m), mj_deleteData),
      velocity_(m->nu),
      limits_(m->nu),
      velocity_limits_(m->nu),
      motor_type_(m->nu) {
  const auto cfg = pose(bundle);
  wrist_ranges_ = cfg.at("wrist_ranges").get<std::array<std::array<int, 2>, 2>>();
  std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)> urdf(
      xmlReadFile((bundle / "urdf/rx_assy_v2_runtime_baseline.urdf").c_str(), nullptr,
                  XML_PARSE_NONET),
      xmlFreeDoc);
  if (!urdf) throw std::runtime_error("无法读取原机关节速度限制");
  const bool swapped = cfg.at("a_is_right");
  for (const auto& [name, q] : cfg.at("joint_positions_rad").items()) {
    int j = id(m, mjOBJ_JOINT, name);
    double x = q.get<double>();
    if (!std::isfinite(x) || x < m->jnt_range[2 * j] || x > m->jnt_range[2 * j + 1])
      throw std::runtime_error("整机初始姿态越限");
    d->qpos[m->jnt_qposadr[j]] = x;
  }
  for (int a = 0; a < m->nu; a++) {
    const char* n = mj_id2name(m, mjOBJ_ACTUATOR, a);
    if (n && std::string_view(n).ends_with("_joint_drive")) {
      driven_.push_back(a);
      const std::string name = n;
      if (name.find("hip_pitch") != std::string::npos ||
          name.find("hip_roll") != std::string::npos || name.find("knee") != std::string::npos ||
          name.starts_with("waist_yaw") || name.find("shoulder_pitch") != std::string::npos)
        motor_type_[a] = 1;
      else if (name.find("ankle_") != std::string::npos || name.starts_with("waist_roll") ||
               name.starts_with("waist_pitch"))
        motor_type_[a] = 2;
      else if (name.starts_with("neck_"))
        motor_type_[a] = 3;
      int j = m->actuator_trnid[2 * a];
      auto joint_node = named(xmlDocGetRootElement(urdf.get()), mj_id2name(m, mjOBJ_JOINT, j));
      velocity_limits_[a] = std::stod(get(child(joint_node, "limit"), "velocity"));
      d->ctrl[a] = d->qpos[m->jnt_qposadr[j]];
      limits_[a] = std::min(-m->actuator_forcerange[2 * a], m->actuator_forcerange[2 * a + 1]);
    }
  }
  if (driven_.size() != 29) throw std::runtime_error("整机必须保留29个原生执行器");
  mj_copyData(reference_.get(), m, d);
  for (int h = 0; h < 2; h++) {
    const std::string hand = h ? "B" : "A", side = (h == 0) != swapped ? "l" : "r";
    hands_[h] = id(m, mjOBJ_BODY, hand + "_" + side + "_gripper_link");
    sites_[h] = id(m, mjOBJ_SITE, hand + "_fullbody_goal");
    for (int k = 0; k < 6; k++) joints_[h][k] = id(m, mjOBJ_JOINT, side + "_" + suffixes[k]);
    solve(h, 0);
  }
  for (int a : driven_) {
    int j = m->actuator_trnid[2 * a];
    d->qpos[m->jnt_qposadr[j]] = reference_->qpos[m->jnt_qposadr[j]];
    d->ctrl[a] = d->qpos[m->jnt_qposadr[j]];
  }
  mj_forward(m, d);
}
void RxFullBody::solve(int h, double angle) {
  const Vec axis = h ? Vec(0, -1, 0) : Vec(1, 0, 0);
  const Mat goal =
      Eigen::AngleAxisd(angle, axis).toRotationMatrix() *
      Eigen::Quaterniond(m_->site_quat[4 * sites_[h]], m_->site_quat[4 * sites_[h] + 1],
                         m_->site_quat[4 * sites_[h] + 2], m_->site_quat[4 * sites_[h] + 3])
          .toRotationMatrix();
  const Vec position = Eigen::Map<const Vec>(m_->site_pos + 3 * sites_[h]);
  std::vector<mjtNum> jp(3 * m_->nv), jr(3 * m_->nv);
  for (int it = 0; it < 100; it++) {
    mj_kinematics(m_, reference_.get());
    mj_comPos(m_, reference_.get());
    Eigen::Matrix<double, 6, 1> e;
    e.head<3>() = position - Eigen::Map<const Vec>(reference_->xpos + 3 * hands_[h]);
    Eigen::AngleAxisd aa(goal * matrix(reference_->xmat + 9 * hands_[h]).transpose());
    e.tail<3>() = .15 * aa.angle() * aa.axis();
    if (e.norm() < 1e-7) return;
    mj_jacBody(m_, reference_.get(), jp.data(), jr.data(), hands_[h]);
    Eigen::Matrix<double, 6, 6> J;
    for (int k = 0; k < 6; k++)
      for (int r = 0; r < 3; r++) {
        int v = m_->jnt_dofadr[joints_[h][k]];
        J(r, k) = jp[r * m_->nv + v];
        J(r + 3, k) = .15 * jr[r * m_->nv + v];
      }
    auto delta =
        (J.transpose() *
         (J * J.transpose() + 1e-7 * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(e))
            .eval();
    if (delta.norm() > .1) delta *= .1 / delta.norm();
    for (int k = 0; k < 6; k++) {
      int j = joints_[h][k], q = m_->jnt_qposadr[j];
      reference_->qpos[q] = std::clamp(reference_->qpos[q] + delta[k], m_->jnt_range[2 * j],
                                       m_->jnt_range[2 * j + 1]);
    }
  }
  throw std::runtime_error("RX整臂逆运动学不可达，手=" + std::to_string(h) +
                           " 角度=" + std::to_string(angle));
}
void RxFullBody::reference(int h, double angle, double dt) {
  if (h < 0 || h > 1 || !std::isfinite(angle) || !std::isfinite(dt) || dt <= 0 ||
      angle < wrist_ranges_[h][0] * pi / 2 - 1e-9 || angle > wrist_ranges_[h][1] * pi / 2 + 1e-9)
    throw std::invalid_argument("整臂目标超出验证范围");
  auto old = std::vector<mjtNum>(reference_->qpos, reference_->qpos + m_->nq);
  solve(h, angle);
  angles_[h] = angle;
  for (int a : driven_) {
    int j = m_->actuator_trnid[2 * a], q = m_->jnt_qposadr[j];
    velocity_[a] = (reference_->qpos[q] - old[q]) / dt;
    if (std::abs(velocity_[a]) > velocity_limits_[a])
      throw std::runtime_error("整臂参考速度超过URDF限制");
  }
}
void RxFullBody::control(mjData* d) {
  for (int a : driven_) {
    int j = m_->actuator_trnid[2 * a], v = m_->jnt_dofadr[j], q = m_->jnt_qposadr[j];
    double target = reference_->qpos[q];
    d->ctrl[a] = target + (35 * velocity_[a] + d->qfrc_bias[v]) / 1500;
    const double tn = motor_type_[a] == 1 ? rx_f_torque_limit(d->qvel[v])
                      : motor_type_[a] == 3
                          ? limits_[a]
                          : (motor_type_[a] == 2 ? 2. : 1.) * rx_s_torque_limit(d->qvel[v]);
    const double cap = std::min(limits_[a], tn);
    m_->actuator_forcerange[2 * a] = -cap;
    m_->actuator_forcerange[2 * a + 1] = cap;
    peak_error_ = std::max(peak_error_, std::abs(d->qpos[q] - target));
  }
}
double RxFullBody::error(const mjData* d, int h) const {
  const Vec axis = h ? Vec(0, -1, 0) : Vec(1, 0, 0);
  Eigen::Quaterniond q(m_->site_quat[4 * sites_[h]], m_->site_quat[4 * sites_[h] + 1],
                       m_->site_quat[4 * sites_[h] + 2], m_->site_quat[4 * sites_[h] + 3]);
  Mat wanted = Eigen::AngleAxisd(angles_[h], axis).toRotationMatrix() * q.toRotationMatrix();
  return Eigen::AngleAxisd(wanted * matrix(d->xmat + 9 * hands_[h]).transpose()).angle();
}
double RxFullBody::position_error(const mjData* d, int h) const {
  return (Eigen::Map<const Vec>(d->xpos + 3 * hands_[h]) -
          Eigen::Map<const Vec>(m_->site_pos + 3 * sites_[h]))
      .norm();
}
Json RxFullBody::report(const mjData* d) const {
  Json out = {{"fixed_base", true},
              {"native_tn_enabled", true},
              {"wrist_ranges", wrist_ranges_},
              {"native_joint_actuators", driven_.size()},
              {"added_wrist_joints", 0},
              {"peak_joint_tracking_error_rad", peak_error_}};
  out["contacts"] = Json::array();
  for (int i = 0; i < d->ncon; i++) {
    const auto& c = d->contact[i];
    if (c.dist < -.0001) {
      const char* a = mj_id2name(m_, mjOBJ_BODY, m_->geom_bodyid[c.geom[0]]);
      const char* b = mj_id2name(m_, mjOBJ_BODY, m_->geom_bodyid[c.geom[1]]);
      out["contacts"].push_back(
          {{"body1", a ? a : ""}, {"body2", b ? b : ""}, {"distance_m", c.dist}});
    }
  }
  for (int h = 0; h < 2; h++) {
    Vec p = Eigen::Map<const Vec>(d->xpos + 3 * hands_[h]) -
            Eigen::Map<const Vec>(m_->site_pos + 3 * sites_[h]);
    out[h ? "B" : "A"] = {{"orientation_error_rad", error(d, h)}, {"position_error_m", p.norm()}};
  }
  return out;
}
}  // namespace rm::cube
