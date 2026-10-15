#include <libxml/parser.h>
#include <libxml/tree.h>
#include <mujoco/mujoco.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Doc = std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)>;
using Model = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
bool is(xmlNode* n, const char* tag) {
  return n && n->type == XML_ELEMENT_NODE && xmlStrEqual(n->name, BAD_CAST tag);
}
std::string attr(xmlNode* n, const char* key) {
  if (!n) throw std::runtime_error("缺少 XML 节点");
  xmlChar* value = xmlGetProp(n, BAD_CAST key);
  std::string result = value ? reinterpret_cast<char*>(value) : "";
  xmlFree(value);
  return result;
}
void set(xmlNode* n, const char* key, const std::string& value) {
  xmlSetProp(n, BAD_CAST key, BAD_CAST value.c_str());
}
xmlNode* child(xmlNode* n, const char* tag) {
  for (auto* c = n->children; c; c = c->next)
    if (is(c, tag)) return c;
  return nullptr;
}
xmlNode* add(xmlNode* n, const char* tag) { return xmlNewChild(n, nullptr, BAD_CAST tag, nullptr); }
template <class F>
void walk(xmlNode* n, F f) {
  for (; n; n = n->next) {
    f(n);
    walk(n->children, f);
  }
}
Doc read(const fs::path& path) {
  Doc d(xmlReadFile(path.c_str(), nullptr, XML_PARSE_NONET | XML_PARSE_NOBLANKS), xmlFreeDoc);
  if (!d) throw std::runtime_error("无法读取 " + path.string());
  return d;
}
void save(const Doc& d, const fs::path& path) {
  if (xmlSaveFormatFileEnc(path.c_str(), d.get(), "UTF-8", 1) < 0)
    throw std::runtime_error("无法写入 " + path.string());
}
Model load(const fs::path& path) {
  char error[2048]{};
  Model m(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  if (!m) throw std::runtime_error(error);
  return m;
}
void require(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
std::string number(double value) {
  std::ostringstream s;
  s << std::setprecision(17) << value;
  return s.str();
}
// STL 解码器限制 20 万面；超限时逐面转存，不简化几何。
std::string mesh_file(const fs::path& asset, const std::string& name) {
  std::ifstream in(asset / "meshes" / name, std::ios::binary);
  require(bool(in), "无法读取网格：" + name);
  std::array<unsigned char, 84> header{};
  in.read(reinterpret_cast<char*>(header.data()), header.size());
  require(bool(in), "STL 文件头不完整");
  const auto u32 = [](const unsigned char* p) {
    return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) | (std::uint32_t(p[2]) << 16) |
           (std::uint32_t(p[3]) << 24);
  };
  const auto count = u32(header.data() + 80);
  require(fs::file_size(asset / "meshes" / name) == 84ull + 50ull * count, "需要二进制 STL");
  if (count <= 200000) return name;
  fs::create_directories(asset / "meshes/generated");
  const std::string relative = "generated/" + fs::path(name).stem().string() + ".msh";
  // 按 float32 原始位模式索引共享顶点，不近似合并，不改变三角面。
  std::map<std::array<std::uint32_t, 3>, std::uint32_t> index;
  std::vector<std::array<std::uint32_t, 3>> vertices;
  std::vector<std::uint32_t> faces;
  std::array<unsigned char, 50> triangle{};
  for (std::uint32_t i = 0; i < count; ++i) {
    in.read(reinterpret_cast<char*>(triangle.data()), triangle.size());
    require(bool(in), "STL 三角面不完整");
    for (int v = 0; v < 3; ++v) {
      std::array<std::uint32_t, 3> xyz{};
      for (int axis = 0; axis < 3; ++axis)
        xyz[axis] = u32(triangle.data() + 12 + 12 * v + 4 * axis);
      auto [it, inserted] = index.emplace(xyz, vertices.size());
      if (inserted) vertices.push_back(xyz);
      faces.push_back(it->second);
    }
  }
  // MuJoCo MSH：4 个 int32 头，float32 顶点，int32 面索引；省略法线和纹理。
  std::ofstream out(asset / "meshes" / relative, std::ios::binary);
  const auto write32 = [&](std::uint32_t value) {
    const std::array<unsigned char, 4> bytes{
        static_cast<unsigned char>(value), static_cast<unsigned char>(value >> 8),
        static_cast<unsigned char>(value >> 16), static_cast<unsigned char>(value >> 24)};
    out.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
  };
  write32(vertices.size());
  write32(0);
  write32(0);
  write32(count);
  for (const auto& v : vertices)
    for (auto value : v) write32(value);
  for (auto value : faces) write32(value);
  require(out.good(), "无法保存生成网格");
  return relative;
}
struct Joint {
  double lower, upper, effort;
};
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 3, "用法：import-rx-full-body 资产目录 临时目录");
    const fs::path asset = fs::absolute(argv[1]);
    const fs::path temp = fs::absolute(argv[2]);
    fs::create_directories(temp);
    auto urdf = read(asset / "urdf/rx_assy_v2_runtime_baseline.urdf");
    auto* root = xmlDocGetRootElement(urdf.get());
    std::set<std::string> removed{"l_gripper_link", "r_gripper_link"};
    bool changed;
    do {
      changed = false;
      for (auto* n = root->children; n; n = n->next) {
        if (is(n, "joint") && removed.contains(attr(child(n, "parent"), "link")))
          changed |= removed.insert(attr(child(n, "child"), "link")).second;
      }
    } while (changed);
    for (auto* n = root->children; n;) {
      auto* next = n->next;
      if ((is(n, "link") && removed.contains(attr(n, "name"))) ||
          (is(n, "joint") && removed.contains(attr(child(n, "child"), "link")))) {
        xmlUnlinkNode(n);
        xmlFreeNode(n);
      }
      n = next;
    }
    std::map<std::string, Joint> joints;
    std::map<std::string, double> masses;
    int visuals = 0, collisions = 0;
    for (auto* n = root->children; n; n = n->next) {
      if (is(n, "joint")) {
        require(attr(n, "type") == "revolute", "发现非转动关节");
        auto* limit = child(n, "limit");
        joints.emplace(attr(n, "name"),
                       Joint{std::stod(attr(limit, "lower")), std::stod(attr(limit, "upper")),
                             std::stod(attr(limit, "effort"))});
      }
      if (is(n, "link")) {
        masses.emplace(attr(n, "name"),
                       std::stod(attr(child(child(n, "inertial"), "mass"), "value")));
        for (auto* c = n->children; c; c = c->next) {
          visuals += is(c, "visual");
          collisions += is(c, "collision");
        }
      }
    }
    require(joints.size() == 29 && masses.size() == 30, "原包结构与预期不符");
    std::map<std::string, std::string> mesh_files;
    walk(root, [&](xmlNode* n) {
      if (is(n, "mesh")) {
        const auto name = fs::path(attr(n, "filename")).filename().string();
        if (!mesh_files.contains(name)) mesh_files[name] = mesh_file(asset, name);
        set(n, "filename", mesh_files.at(name));
      }
    });
    auto* extension = child(root, "mujoco");
    if (!extension) extension = add(root, "mujoco");
    auto* compiler = child(extension, "compiler");
    if (!compiler) compiler = add(extension, "compiler");
    set(compiler, "discardvisual", "false");
    set(compiler, "fusestatic", "false");
    set(compiler, "meshdir", (asset / "meshes").string());
    save(urdf, temp / "rx-pruned.urdf");
    auto imported = load(temp / "rx-pruned.urdf");
    char error[2048]{};
    require(
        mj_saveLastXML((temp / "rx-compiled.xml").c_str(), imported.get(), error, sizeof(error)),
        error);
    auto mjcf = read(temp / "rx-compiled.xml");
    root = xmlDocGetRootElement(mjcf.get());
    set(root, "model", "rx_full_body");
    compiler = child(root, "compiler");
    set(compiler, "meshdir", "meshes/");
    set(compiler, "fusestatic", "false");
    // 所有局部缺省值放入命名类；不向合并场景的主缺省类写值。
    auto* defaults = child(root, "default");
    if (!defaults) defaults = add(root, "default");
    auto* wrapper = xmlNewNode(nullptr, BAD_CAST "default");
    xmlReplaceNode(defaults, wrapper);
    xmlAddChild(wrapper, defaults);
    set(defaults, "class", "rx_full_body");
    auto* joint_default = add(defaults, "joint");
    set(joint_default, "armature", "0.01");
    auto* world = child(root, "worldbody");
    for (auto* n = world->children; n; n = n->next)
      if (is(n, "body")) set(n, "childclass", "rx_full_body");
    walk(child(root, "asset"), [&](xmlNode* n) {
      if (is(n, "mesh")) {
        const auto file = fs::path(attr(n, "file"));
        set(n, "file",
            file.is_absolute() ? fs::relative(file, asset / "meshes").string() : file.string());
      }
    });
    // 序列化默认精度不足以保留原包质量与主惯量，回填完整精度。
    walk(world, [&](xmlNode* n) {
      if (!is(n, "body")) return;
      const int b = mj_name2id(imported.get(), mjOBJ_BODY, attr(n, "name").c_str());
      require(b >= 0, "导出 body 不在编译模型中");
      auto* inertial = child(n, "inertial");
      require(inertial != nullptr, "缺少显式惯性");
      set(inertial, "mass", number(imported->body_mass[b]));
      const auto vector = [](const mjtNum* v, int count) {
        std::string result;
        for (int i = 0; i < count; ++i) result += (i ? " " : "") + number(v[i]);
        return result;
      };
      set(inertial, "pos", vector(imported->body_ipos + 3 * b, 3));
      set(inertial, "quat", vector(imported->body_iquat + 4 * b, 4));
      set(inertial, "diaginertia", vector(imported->body_inertia + 3 * b, 3));
    });
    // 原始装配网格凸包在零位相交；只排除这些固定的局部装配对。
    auto* contact = add(root, "contact");
    for (const auto& pair : std::array<std::pair<const char*, const char*>, 8>{
             {{"base_link", "waist_yaw_link"},
              {"base_link", "waist_roll_link"},
              {"base_link", "waist_pitch_link"},
              {"base_link", "r_hip_pitch_link"},
              {"base_link", "l_hip_pitch_link"},
              {"waist_yaw_link", "waist_pitch_link"},
              {"r_knee_link", "r_ankle_roll_link"},
              {"l_knee_link", "l_ankle_roll_link"}}}) {
      auto* e = add(contact, "exclude");
      set(e, "body1", pair.first);
      set(e, "body2", pair.second);
    }
    auto* actuators = add(root, "actuator");
    for (const auto& [name, j] : joints) {
      auto* a = add(actuators, "position");
      set(a, "name", name + "_drive");
      set(a, "class", "rx_full_body");
      set(a, "joint", name);
      set(a, "kp", number(j.effort * 5));
      set(a, "kv", number(j.effort * 0.2));
      set(a, "ctrllimited", "true");
      set(a, "ctrlrange", number(j.lower) + " " + number(j.upper));
      set(a, "forcelimited", "true");
      set(a, "forcerange", number(-j.effort) + " " + number(j.effort));
    }
    save(mjcf, asset / "robot.xml");
    auto model = load(asset / "robot.xml");
    const auto* m = model.get();
    require(m->njnt == 29 && m->nu == 29 && m->nq == 29 && m->nv == 29, "关节或执行器数量错误");
    require(m->nbody == 31 && m->ngeom == visuals + collisions, "机身或几何数量错误");
    std::ostringstream report;
    report << std::setprecision(12) << "MuJoCo " << mj_versionString()
           << "\n关节：29 hinge\n执行器：29 position\n";
    for (int i = 0; i < m->njnt; ++i) {
      const std::string name = mj_id2name(m, mjOBJ_JOINT, i);
      const auto& j = joints.at(name);
      require(m->jnt_type[i] == mjJNT_HINGE && m->jnt_limited[i], "关节类型错误：" + name);
      require(std::abs(m->jnt_range[2 * i] - j.lower) < 1e-6 &&
                  std::abs(m->jnt_range[2 * i + 1] - j.upper) < 1e-6,
              "关节范围错误：" + name);
      int a = mj_name2id(m, mjOBJ_ACTUATOR, (name + "_drive").c_str());
      require(a >= 0 && m->actuator_trnid[2 * a] == i && m->actuator_forcelimited[a] &&
                  std::abs(m->actuator_forcerange[2 * a] + j.effort) < 1e-9 &&
                  std::abs(m->actuator_forcerange[2 * a + 1] - j.effort) < 1e-9,
              "执行器错误：" + name);
      report << "joint " << name << " [" << j.lower << ", " << j.upper << "] effort=" << j.effort
             << '\n';
    }
    double mass = 0;
    for (int i = 1; i < m->nbody; ++i) {
      const std::string name = mj_id2name(m, mjOBJ_BODY, i);
      require(!removed.contains(name), "仍有原夹爪");
      require(std::abs(m->body_mass[i] - masses.at(name)) < 1e-12, "质量未保留：" + name);
      mass += m->body_mass[i];
      report << "body " << name << " mass=" << m->body_mass[i] << '\n';
    }
    for (const char* name : {"l_wrist_pitch_link", "r_wrist_pitch_link"})
      require(mj_name2id(m, mjOBJ_BODY, name) >= 0, "缺少腕部末端");
    int active = 0;
    for (int i = 0; i < m->ngeom; ++i)
      active += m->geom_contype[i] != 0 || m->geom_conaffinity[i] != 0;
    require(active == collisions, "碰撞几何未完整保留");
    report << "visual=" << visuals << " collision=" << active << "\n总质量（kg）：" << mass << '\n';
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(m), mj_deleteData);
    mj_forward(m, data.get());
    report << "零位接触数：" << data->ncon << '\n';
    for (int i = 0; i < data->ncon; ++i) {
      auto& c = data->contact[i];
      report << "contact " << mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[c.geom[0]]) << " "
             << mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[c.geom[1]]) << " dist=" << c.dist << '\n';
    }
    // 独立复制后从另一路径加载，避免原工作目录掩盖绝对路径依赖。
    const auto portable = temp / "portable";
    fs::create_directories(portable);
    fs::copy_file(asset / "robot.xml", portable / "robot.xml",
                  fs::copy_options::overwrite_existing);
    fs::copy(asset / "meshes", portable / "meshes",
             fs::copy_options::recursive | fs::copy_options::overwrite_existing);
    auto moved = load(portable / "robot.xml");
    require(moved->njnt == 29 && moved->nu == 29, "迁移加载失败");
    for (int i = 0; i < 100; ++i) mj_step(m, data.get());
    for (int i = 0; i < m->nq; ++i) require(std::isfinite(data->qpos[i]), "短步进出现非有限状态");
    for (int i = 0; i < mjNWARNING; ++i)
      require(data->warning[i].number == 0, "短步进出现 MuJoCo 警告");
    report << "迁移加载：通过\n100 步有限值及警告检查：通过（不代表控制器验收）\n";
    std::ofstream out(asset / "validation.txt");
    out << report.str();
    require(out.good(), "无法保存验证报告");
    std::cout << report.str();
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
