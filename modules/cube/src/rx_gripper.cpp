#include "rm/rx_gripper.hpp"

#include <libxml/parser.h>

#include <Eigen/Geometry>
#include <array>
#include <functional>
#include <iomanip>
#include <memory>
#include <numbers>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rm::cube {
namespace {
using Node = xmlNodePtr;
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
constexpr char defaults_name[] = "RX_gripper_defaults";
constexpr double shift = .04189107, cube_pitch = .055 / 3;
// 与 rx_tip 闭环探针相同的连杆坐标和二分算法；生产代码不依赖测试文件。
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
double gap(double motor) { return 2 * (.01389246 + (rz(angle(motor)) * (B - A) - (B - A)).y()); }
double motor_for_gap(double target) {
  double lo = -2.25, hi = 0;
  for (int i = 0; i < 60; ++i) {
    double mid = (lo + hi) / 2;
    if (gap(mid) > target)
      lo = mid;
    else
      hi = mid;
  }
  return (lo + hi) / 2;
}
std::string get(Node node, const char* key) {
  auto value = xmlGetProp(node, BAD_CAST key);
  std::string result = value ? reinterpret_cast<const char*>(value) : "";
  xmlFree(value);
  return result;
}
void set(Node node, const char* key, const std::string& value) {
  if (!xmlSetProp(node, BAD_CAST key, BAD_CAST value.c_str()))
    throw std::runtime_error("RX XML 属性写入失败");
}
Node child(Node node, const char* tag) {
  for (auto p = node->children; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE && xmlStrEqual(p->name, BAD_CAST tag)) return p;
  return nullptr;
}
Node add(Node parent, const char* tag,
         std::initializer_list<std::pair<const char*, std::string>> attrs = {}) {
  auto node = xmlNewChild(parent, nullptr, BAD_CAST tag, nullptr);
  if (!node) throw std::runtime_error("RX XML 节点创建失败");
  for (const auto& [key, value] : attrs) set(node, key, value);
  return node;
}
Node section(Node root, const char* tag) {
  auto result = child(root, tag);
  return result ? result : add(root, tag);
}
void visit(Node node, const std::function<void(Node)>& fn) {
  for (auto p = node; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE) {
      fn(p);
      visit(p->children, fn);
    }
}
void copy(Node source, Node target) {
  auto node = xmlDocCopyNode(source, target->doc, 1);
  if (!node) throw std::runtime_error("RX XML 节点复制失败");
  if (!xmlAddChild(target, node)) {
    xmlFreeNode(node);
    throw std::runtime_error("RX XML 节点追加失败");
  }
}
std::string vec(const Vec& value) {
  std::ostringstream out;
  out << std::setprecision(17) << value.x() << ' ' << value.y() << ' ' << value.z();
  return out.str();
}
std::string quat(const Mat& value) {
  Eigen::Quaterniond q(value);
  std::ostringstream out;
  out << std::setprecision(17) << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z();
  return out.str();
}
std::string renamed(const std::string& name) {
  if (name.starts_with("l_")) return "A_" + name;
  if (name.starts_with("r_")) return "B_" + name;
  return "RX_" + name;
}
void private_defaults(Node native) {
  // 隔离生产场景的 geom/equality 默认值；显式源属性和源默认类继续覆盖这些基值。
  auto parent = section(native, "default");
  set(parent, "class", defaults_name);
  auto baseline = [&](const char* tag,
                      std::initializer_list<std::pair<const char*, std::string>> attrs) {
    auto node = section(parent, tag);
    for (const auto& [key, value] : attrs)
      if (get(node, key).empty()) set(node, key, value);
  };
  baseline("geom", {{"contype", "1"},
                    {"conaffinity", "1"},
                    {"condim", "3"},
                    {"friction", "1 .005 .0001"},
                    {"solref", ".02 1"},
                    {"solimp", ".9 .95 .001 .5 2"},
                    {"density", "1000"}});
  baseline("joint", {{"damping", "0"}, {"armature", "0"}, {"frictionloss", "0"}});
  baseline("equality", {{"solref", ".02 1"}, {"solimp", ".9 .95 .001 .5 2"}});
}
int id(const mjModel* model, mjtObj type, const std::string& name) {
  const int result = mj_name2id(model, type, name.c_str());
  if (result < 0) throw std::runtime_error("RX 模型缺少对象: " + name);
  return result;
}
void initialize_hand(mjModel* model, mjData* data, bool left) {
  const std::string prefix = left ? "A_l" : "B_r";
  const double motor = -2.25, t = angle(motor);
  auto put = [&](const std::string& suffix, double value) {
    const int joint = id(model, mjOBJ_JOINT, prefix + suffix);
    data->qpos[model->jnt_qposadr[joint]] = value;
  };
  put("_gripper_motor_joint", motor);
  data->ctrl[id(model, mjOBJ_ACTUATOR, left ? "A_fingers_actuator" : "B_fingers_actuator")] = motor;
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

void append_rx_grippers(xmlNodePtr root, const std::filesystem::path& bundle) {
  if (!root || !root->doc || !xmlStrEqual(root->name, BAD_CAST "mujoco"))
    throw std::invalid_argument("RX 追加入口需要有效的 mujoco 根节点");
  const auto directory = std::filesystem::canonical(bundle);
  std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)> doc(
      xmlReadFile((directory / "free_sweep.xml").c_str(), nullptr, XML_PARSE_NONET), xmlFreeDoc);
  if (!doc) throw std::runtime_error("无法读取 RX free_sweep.xml");
  auto native = xmlDocGetRootElement(doc.get());
  if (!native || !xmlStrEqual(native->name, BAD_CAST "mujoco"))
    throw std::runtime_error("RX 源文件不是 mujoco XML");
  auto world = child(native, "worldbody");
  if (!world) throw std::runtime_error("RX 源文件缺少 worldbody");
  std::vector<Node> remove;
  visit(native, [&](Node node) {
    if (get(node, "name") == "test_pull_slide" || get(node, "name") == "test_pull")
      remove.push_back(node);
  });
  for (auto node : remove) {
    xmlUnlinkNode(node);
    xmlFreeNode(node);
  }

  // 所有资产名与引用同步重命名；匿名 geom 不经过名称过滤，原样保留。
  visit(native, [&](Node node) {
    for (const char* key :
         {"name",   "class",  "childclass", "body",     "body1",   "body2",  "joint",
          "joint1", "joint2", "geom",       "geom1",    "geom2",   "site",   "site1",
          "site2",  "tendon", "mesh",       "material", "texture", "hfield", "target"}) {
      const auto value = get(node, key);
      if (!value.empty()) set(node, key, renamed(value));
    }
    if (xmlStrEqual(node->name, BAD_CAST "mesh") && !get(node, "file").empty()) {
      const auto file = std::filesystem::canonical(directory / get(node, "file"));
      if (!std::filesystem::is_regular_file(file)) throw std::runtime_error("RX mesh 文件无效");
      set(node, "file", file.string());
    }
  });
  private_defaults(native);
  auto actuator = child(native, "actuator");
  if (!actuator) throw std::runtime_error("RX 源文件缺少 actuator");
  int motor_count = 0;
  for (auto node = actuator->children; node; node = node->next) {
    if (node->type != XML_ELEMENT_NODE) continue;
    const auto name = get(node, "name");
    if (name == "A_l_gripper_motor_joint" || name == "B_r_gripper_motor_joint") {
      set(node, "name", name.starts_with("A_") ? "A_fingers_actuator" : "B_fingers_actuator");
      ++motor_count;
    }
    if (get(node, "class").empty()) set(node, "class", defaults_name);
  }
  if (motor_count != 2) throw std::runtime_error("RX 必须包含两个夹爪电机");
  for (const char* tag : {"equality", "tendon"})
    if (auto source = child(native, tag))
      for (auto node = source->children; node; node = node->next)
        if (node->type == XML_ELEMENT_NODE && get(node, "class").empty())
          set(node, "class", defaults_name);
  int connects = 0;
  if (auto source = child(native, "equality"))
    for (auto node = source->children; node; node = node->next)
      if (node->type == XML_ELEMENT_NODE && xmlStrEqual(node->name, BAD_CAST "connect")) ++connects;
  if (connects != 8) throw std::runtime_error("RX 必须保留八个闭环 connect");

  std::array<Node, 2> hands{};
  for (auto node = world->children; node; node = node->next) {
    if (node->type != XML_ELEMENT_NODE) continue;
    const auto name = get(node, "name");
    if (!xmlStrEqual(node->name, BAD_CAST "body") ||
        (name != "A_l_gripper_link" && name != "B_r_gripper_link"))
      throw std::runtime_error("RX worldbody 仅允许两个夹爪根 body");
    const int h = name.starts_with("A_") ? 0 : 1;
    if (hands[h]) throw std::runtime_error("RX 夹爪根 body 重复");
    hands[h] = node;
    if (get(node, "childclass").empty()) set(node, "childclass", defaults_name);
  }
  if (!hands[0] || !hands[1]) throw std::runtime_error("RX 夹爪根 body 不完整");
  int fingertips = 0;
  visit(world->children, [&](Node node) {
    const auto name = get(node, "name");
    if (!xmlStrEqual(node->name, BAD_CAST "body") ||
        (!name.ends_with("_finger1_link") && !name.ends_with("_finger2_link")))
      return;
    ++fingertips;
    const bool first = name.ends_with("_finger1_link");
    const double sign = first ? -1 : 1;
    const std::string hand = name.starts_with("A_") ? "A" : "B";
    auto tip = add(node, "body", {{"name", name + "_extension"}});
    auto append = [&](const std::string& geom_name, double x, double y, const std::string& size,
                      const std::string& rgba) {
      add(tip, "geom",
          {{"name", geom_name},
           {"type", "box"},
           {"pos", vec(Vec(x, sign * y, 0))},
           {"size", size},
           {"mass", ".001"},
           {"contype", "1"},
           {"conaffinity", "15"},
           {"friction", "0 0 0"},
           {"condim", "1"},
           {"rgba", rgba},
           {"solref", ".004 1"},
           {"solimp", ".99 .999 .001 .5 2"},
           {"margin", ".0005"}});
    };
    append(name + "_neck", .027, .01789246, ".009 .003 .004", ".2 .7 .8 1");
    append(hand + (first ? "_right_tip" : "_left_tip"), .040, .01689246, ".008 .003 .008",
           ".1 .15 .2 1");
  });
  if (fingertips != 4) throw std::runtime_error("RX 必须包含四个手指 body");

  std::set<std::string> existing;
  visit(root->children, [&](Node node) {
    for (const char* key : {"name", "class"}) {
      const auto value = get(node, key);
      if (!value.empty()) existing.insert(value);
    }
  });
  visit(native->children, [&](Node node) {
    for (const char* key : {"name", "class"}) {
      const auto value = get(node, key);
      if (!value.empty() && existing.contains(value))
        throw std::runtime_error("RX 导入命名冲突: " + value);
    }
  });
  for (const char* name : {"A_wrist", "B_wrist", "A_yaw", "B_yaw", "A_yaw_drive", "B_yaw_drive"})
    if (existing.contains(name)) throw std::runtime_error("RX 手腕命名冲突: " + std::string(name));
  auto compiler = section(root, "compiler");
  if (!get(compiler, "angle").empty() && get(compiler, "angle") != "radian")
    throw std::runtime_error("RX 父场景必须使用 radian");
  set(compiler, "angle", "radian");
  set(compiler, "inertiafromgeom", "auto");
  copy(child(native, "default"), section(root, "default"));
  for (const char* tag : {"asset", "contact", "tendon", "equality", "actuator"}) {
    if (auto source = child(native, tag)) {
      auto target = section(root, tag);
      for (auto node = source->children; node; node = node->next)
        if (node->type == XML_ELEMENT_NODE) copy(node, target);
    }
  }
  const double half = (.055 - .0007 * 55 / 60) / 2;
  const double cx = -.00189107 + (rz(angle(motor_for_gap(2 * half))) * (B - A) - (B - A)).x();
  Mat ra = Vec(-1, 1, -1).asDiagonal(), rb;
  rb.col(0) = Vec(0, 1, 0);
  rb.col(1) = Vec(1, 0, 0);
  rb.col(2) = Vec(0, 0, -1);
  auto target_world = section(root, "worldbody");
  for (int h = 0; h < 2; ++h) {
    const std::string hand = h == 0 ? "A" : "B";
    const Vec axis = h == 0 ? Vec(1, 0, 0) : Vec(0, -1, 0);
    auto wrist =
        add(target_world, "body",
            {{"name", hand + "_wrist"}, {"pos", "0 0 .22"}, {"childclass", defaults_name}});
    add(wrist, "joint",
        {{"name", hand + "_yaw"},
         {"type", "hinge"},
         {"axis", vec(axis)},
         {"limited", "true"},
         {"range", "-3.2 3.2"},
         {"damping", ".2"},
         {"armature", ".01"}});
    set(hands[h], "pos", vec(axis * (cube_pitch + cx + shift)));
    // 世界轴旋转左乘原安装矩阵，安装平移在轴线上，因此保持不变。
    Mat mounted =
        Eigen::AngleAxisd(std::numbers::pi / 2, axis).toRotationMatrix() * (h == 0 ? ra : rb);
    set(hands[h], "quat", quat(mounted));
    copy(hands[h], wrist);
    add(section(root, "actuator"), "position",
        {{"name", hand + "_yaw_drive"},
         {"joint", hand + "_yaw"},
         {"class", defaults_name},
         {"kp", "20"},
         {"kv", "1"},
         {"forcerange", "-15 15"},
         {"forcelimited", "true"}});
  }
}

void initialize_rx_grippers(mjModel* model, mjData* data) {
  if (!model || !data) throw std::invalid_argument("RX 初始化需要有效 mjModel 和 mjData");
  initialize_hand(model, data, true);
  initialize_hand(model, data, false);
}
}  // namespace rm::cube
