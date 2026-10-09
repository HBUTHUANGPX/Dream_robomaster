#pragma once
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <mujoco/mujoco.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <numbers>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

// RX 窄指尖单层探针；不是完整魔方还原验收。
namespace rx_tip {
inline constexpr double pi = std::numbers::pi;
inline constexpr double default_shift = .04189107;
inline constexpr double default_grip = -1.20;
inline constexpr double default_pad_mm = 16;
inline int id(const mjModel* m, mjtObj type, const std::string& name) {
  int value = mj_name2id(m, type, name.c_str());
  if (value < 0) throw std::runtime_error("模型缺少对象: " + name);
  return value;
}
using V = Eigen::Vector3d;
using M = Eigen::Matrix3d;
using J = nlohmann::json;
using N = xmlNodePtr;
inline std::string get(N n, const char* k) {
  if (!n) throw std::runtime_error("XML 节点为空");
  auto p = xmlGetProp(n, BAD_CAST k);
  std::string s = p ? (char*)p : "";
  xmlFree(p);
  return s;
}
inline void set(N n, const char* k, std::string v) {
  if (!n || !xmlSetProp(n, BAD_CAST k, BAD_CAST v.c_str()))
    throw std::runtime_error("XML 属性写入失败");
}
inline N add(N p, const char* t,
             std::initializer_list<std::pair<const char*, std::string>> a = {}) {
  auto n = xmlNewChild(p, nullptr, BAD_CAST t, nullptr);
  if (!n) throw std::runtime_error("XML 节点创建失败");
  for (auto [k, v] : a) set(n, k, v);
  return n;
}
inline N child(N n, const char* t) {
  if (!n) throw std::runtime_error("XML 根节点为空");
  for (auto p = n->children; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE && xmlStrEqual(p->name, BAD_CAST t)) return p;
  throw std::runtime_error(std::string("XML 缺少节点: ") + t);
}
inline void walk(N n, std::function<void(N)> fn) {
  for (auto p = n; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE) {
      fn(p);
      walk(p->children, fn);
    }
}
inline std::string vec(V v) {
  std::ostringstream o;
  o << std::setprecision(14) << v.x() << ' ' << v.y() << ' ' << v.z();
  return o.str();
}
inline std::string quat(M r) {
  Eigen::Quaterniond q(r);
  std::ostringstream o;
  o << std::setprecision(14) << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z();
  return o.str();
}
inline M rx(double t) { return Eigen::AngleAxisd(t, V::UnitX()).toRotationMatrix(); }
inline M rz(double t) { return Eigen::AngleAxisd(t, V::UnitZ()).toRotationMatrix(); }
const V A(-.06784728, .038, 0), B(-.0235475, .03009246, 0), E(-.04332617, .031006855, -.0095),
    C(-.04814728, -.0043544, -.0122515), O(-.04814728, 0, 0);
const double phase = -.8389783259706423, rod = .0357943041426583;
inline double angle(double motor) {
  auto f = [&](double t) {
    return (A + rz(t) * (E - A) - (O + rx(phase - motor) * (C - O))).squaredNorm() - rod * rod;
  };
  double lo = -.6, hi = .8;
  for (int i = 0; i < 70; i++) {
    double mid = (lo + hi) / 2;
    if (f(mid) * f(lo) > 0)
      lo = mid;
    else
      hi = mid;
  }
  return (lo + hi) / 2;
}
inline double gap(double motor) {
  return 2 * (.01389246 + (rz(angle(motor)) * (B - A) - (B - A)).y());
}
inline double motor_for_gap(double g) {
  double lo = -2.25, hi = 0;
  for (int i = 0; i < 60; i++) {
    double m = (lo + hi) / 2;
    if (gap(m) > g)
      lo = m;
    else
      hi = m;
  }
  return (lo + hi) / 2;
}
inline void conf(mjModel* m, mjData* d, std::string side, double motor) {
  auto put = [&](std::string n, double q) {
    int j = id(m, mjOBJ_JOINT, n);
    d->qpos[m->jnt_qposadr[j]] = q;
  };
  double t = angle(motor);
  put(side + "_gripper_motor_joint", motor);
  d->ctrl[id(m, mjOBJ_ACTUATOR, side + "_gripper_motor_joint")] = motor;
  M S = V(1, -1, -1).asDiagonal(), T = side == "l" ? S : M::Identity(), rm = rx(phase - motor);
  for (auto branch : {"positive", "negative"}) {
    M U = std::string(branch) == "positive" ? M::Identity() : S;
    int finger = (T * U * B).y() > 0 ? 2 : 1;
    put(side + "_" + branch + "_outer_joint", t);
    put(side + "_" + branch + "_inner_joint", t);
    put(side + "_finger" + std::to_string(finger) + "_joint", -t);
    V delta = (A + rz(t) * (E - A)) - (O + rm * (C - O));
    M rg = Eigen::Quaterniond::FromTwoVectors(E - C, delta).toRotationMatrix();
    M tr = T * U;
    Eigen::Quaterniond q(tr * rm.transpose() * rg * tr.transpose());
    int j = id(m, mjOBJ_JOINT, side + "_" + branch + "_rod_ball"), adr = m->jnt_qposadr[j];
    d->qpos[adr] = q.w();
    d->qpos[adr + 1] = q.x();
    d->qpos[adr + 2] = q.y();
    d->qpos[adr + 3] = q.z();
  }
}

inline int parse_hand(const std::string& hand) {
  if (hand == "A") return 0;
  if (hand == "B") return 1;
  throw std::invalid_argument("手身份必须为 A 或 B");
}
inline std::string hand_name(int hand) { return hand == 0 ? "A" : "B"; }
inline double finite_number(const std::string& text) {
  size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value))
    throw std::invalid_argument("数值必须完整且有限: " + text);
  return value;
}
inline std::filesystem::path directory(const std::string& text, bool create) {
  if (text.empty()) throw std::invalid_argument("目录路径不能为空");
  auto path = std::filesystem::absolute(text);
  if (create) std::filesystem::create_directories(path);
  if (!std::filesystem::is_directory(path))
    throw std::runtime_error("目录不存在或不是目录: " + path.string());
  return path;
}
inline std::string geom_name(const mjModel* m, int geom) {
  const char* name = mj_id2name(m, mjOBJ_GEOM, geom);
  return name ? name : "<未命名>";
}
inline bool extension(const mjModel* m, int geom) {
  const char* name = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[geom]);
  return name && std::string(name).ends_with("_extension");
}
inline std::array<int, 2> hand_roots(const mjModel* m) {
  return {id(m, mjOBJ_BODY, "l_gripper_link"), id(m, mjOBJ_BODY, "r_gripper_link")};
}
inline void classify(const mjModel* m, const std::array<int, 2>& roots, std::vector<int>& hand,
                     std::vector<int>& piece) {
  hand.assign(m->ngeom, -1);
  piece.assign(m->ngeom, -1);
  for (int g = 0; g < m->ngeom; ++g) {
    const auto name = geom_name(m, g);
    if (name.starts_with("piece_")) {
      size_t consumed = 0;
      const auto suffix = name.substr(6);
      piece[g] = std::stoi(suffix, &consumed);
      if (consumed != suffix.size() || piece[g] < 0 || piece[g] >= 26)
        throw std::runtime_error("魔方块编号非法: " + name);
    }
    for (int b = m->geom_bodyid[g]; b > 0; b = m->body_parentid[b])
      for (int h = 0; h < 2; ++h)
        if (b == roots[h]) hand[g] = h;
  }
}
inline void write_json(const std::filesystem::path& path, const J& data) {
  std::ofstream output(path);
  if (!output) throw std::runtime_error("无法创建输出: " + path.string());
  output << data.dump(2) << '\n';
  output.close();
  if (!output) throw std::runtime_error("输出写入失败: " + path.string());
}
struct Model {
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
  explicit Model(const std::filesystem::path& path) {
    if (!std::filesystem::is_regular_file(path))
      throw std::runtime_error("模型文件不存在: " + path.string());
    std::array<char, 4096> error{};
    model.reset(mj_loadXML(path.c_str(), nullptr, error.data(), error.size()));
    if (!model) throw std::runtime_error("模型加载失败: " + path.string() + ": " + error.data());
    data.reset(mj_makeData(model.get()));
    if (!data) throw std::runtime_error("MuJoCo 数据分配失败");
  }
};
}  // namespace rx_tip
