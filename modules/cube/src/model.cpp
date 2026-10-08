#include <libxml/parser.h>
#include <libxml/tree.h>

#include <iomanip>
#include <sstream>

#include "rm/cube.hpp"
namespace rm::cube {
namespace {
using Node = xmlNodePtr;
void set(Node n, const char* k, const std::string& v) {
  xmlSetProp(n, BAD_CAST k, BAD_CAST v.c_str());
}
std::string get(Node n, const char* k) {
  auto p = xmlGetProp(n, BAD_CAST k);
  std::string s = p ? reinterpret_cast<char*>(p) : "";
  xmlFree(p);
  return s;
}
Node add(Node p, const char* tag,
         std::initializer_list<std::pair<const char*, std::string>> attrs = {}) {
  auto n = xmlNewChild(p, nullptr, BAD_CAST tag, nullptr);
  for (auto& [k, v] : attrs) set(n, k, v);
  return n;
}
std::string vec(const Vec& v) {
  std::ostringstream s;
  s << std::setprecision(17) << v.x() << ' ' << v.y() << ' ' << v.z();
  return s.str();
}
Node child(Node n, const char* tag) {
  for (auto p = n->children; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE && xmlStrEqual(p->name, BAD_CAST tag)) return p;
  return nullptr;
}
void visit(Node n, const std::function<void(Node)>& f) {
  for (auto p = n; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE) {
      f(p);
      visit(p->children, f);
    }
}
std::string piece_name(int i) {
  return "piece_" + (i < 10 ? std::string("0") : std::string()) + std::to_string(i);
}
const char* colors[3][2] = {{"1 .30 .015 1", ".82 .035 .055 1"},
                            {".015 .63 .27 1", ".025 .19 .85 1"},
                            {"1 .77 .015 1", ".94 .95 .96 1"}};
void geometry(Node body, const Vec& slot, const Vec& offset) {
  add(body, "geom",
      {{"type", "box"},
       {"pos", vec(offset)},
       {"size", ".00965 .00965 .00965"},
       {"mass", ".006"},
       {"rgba", ".025 .029 .035 1"}});
  for (int a = 0; a < 3; a++)
    if (slot[a]) {
      Vec p = offset, z = Vec::Constant(.00825);
      p[a] += slot[a] * .00985;
      z[a] = .00025;
      add(body, "geom",
          {{"type", "box"},
           {"pos", vec(p)},
           {"size", vec(z)},
           {"mass", "0"},
           {"rgba", colors[a][slot[a] > 0]},
           {"contype", "0"},
           {"conaffinity", "0"}});
    }
}
void gripper(Node root, const std::filesystem::path& repo, const std::string& hand, bool fast) {
  auto path = repo / "assets/robotiq_2f85";
  auto doc = xmlReadFile((path / "2f85.xml").c_str(), nullptr, XML_PARSE_NONET);
  if (!doc) throw std::runtime_error("Cannot read Robotiq model");
  auto native = xmlDocGetRootElement(doc);
  auto prefix = hand + "_";
  visit(child(native, "asset")->children, [&](Node n) {
    if (xmlStrEqual(n->name, BAD_CAST "mesh")) {
      auto file = get(n, "file");
      set(n, "name", std::filesystem::path(file).stem().string());
      set(n, "file", (path / "assets" / file).string());
    }
  });
  visit(native, [&](Node n) {
    for (auto key : {"name", "class", "childclass", "body1", "body2", "joint", "joint1", "joint2",
                     "tendon", "mesh", "material"}) {
      auto val = get(n, key);
      if (!val.empty()) set(n, key, prefix + val);
    }
  });
  visit(native, [&](Node n) {
    if (get(n, "class") == prefix + "collision" && xmlStrEqual(n->name, BAD_CAST "default")) {
      auto g = child(n, "geom");
      set(g, "contype", "2");
      set(g, "conaffinity", "3");
    }
    for (auto side : {"right", "left"})
      if (get(n, "name") == prefix + side + "_pad") {
        add(n, "geom",
            {{"name", prefix + side + "_extension"},
             {"type", "box"},
             {"pos", "0 .0014 .041"},
             {"size", ".008 .003 .011"},
             {"mass", ".001"},
             {"rgba", hand == "A" ? ".20 .45 .64 1" : ".85 .42 .12 1"},
             {"contype", "2"},
             {"conaffinity", "3"},
             {"group", "0"}});
        add(n, "geom",
            {{"name", prefix + side + "_tip"},
             {"type", "box"},
             {"pos", "0 .0014 .055"},
             {"size", ".008 .004 .007"},
             {"mass", ".001"},
             {"rgba", ".09 .13 .17 1"},
             {"friction", "1.2 .005 .0001"},
             {"condim", "4"},
             {"priority", "2"},
             {"solref", fast ? ".0003 1" : ".002 1"},
             {"solimp", ".9999 .9999 .001"},
             {"contype", "2"},
             {"conaffinity", "3"},
             {"group", "0"}});
      }
  });
  for (auto section : {"asset", "default", "contact", "tendon", "equality", "actuator"}) {
    auto dest = child(root, section);
    if (!dest) dest = add(root, section);
    auto src = child(native, section);
    if (src)
      for (auto p = src->children; p; p = p->next)
        if (p->type == XML_ELEMENT_NODE) xmlAddChild(dest, xmlDocCopyNode(p, root->doc, 1));
  }
  Vec normal = hand == "A" ? Vec(1, 0, 0) : Vec(0, -1, 0);
  auto world = child(root, "worldbody");
  auto wrist = add(world, "body", {{"name", hand + "_wrist"}, {"pos", "0 0 .22"}});
  add(wrist, "joint",
      {{"name", hand + "_yaw"},
       {"type", "hinge"},
       {"axis", vec(normal)},
       {"range", "-3.2 3.2"},
       {"damping", ".015"},
       {"armature", ".0005"}});
  add(wrist, "geom",
      {{"type", "cylinder"},
       {"pos", vec(normal * .215)},
       {"zaxis", vec(normal)},
       {"size", ".027 .018"},
       {"mass", ".15"},
       {"contype", "2"},
       {"conaffinity", "3"},
       {"rgba", hand == "A" ? ".14 .36 .62 1" : ".84 .35 .10 1"}});
  Mat r;
  r.col(0) = Vec(0, 0, 1);
  r.col(2) = -normal;
  r.col(1) = r.col(2).cross(r.col(0));
  Eigen::Quaterniond q(r);
  std::ostringstream qs;
  qs << std::setprecision(17) << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z();
  auto mount = add(wrist, "body",
                   {{"name", hand + "_mount"}, {"pos", vec(normal * .2055)}, {"quat", qs.str()}});
  for (auto p = child(native, "worldbody")->children; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE) xmlAddChild(mount, xmlDocCopyNode(p, root->doc, 1));
  add(child(root, "actuator"), "position",
      {{"name", hand + "_yaw_drive"},
       {"joint", hand + "_yaw"},
       {"kp", "20"},
       {"kv", "1"},
       {"forcerange", "-15 15"}});
  Vec pedestal = Vec(0, 0, .22) + normal * .26;
  pedestal.z() = .08;
  add(world, "geom",
      {{"name", hand + "_pedestal"},
       {"type", "box"},
       {"pos", vec(pedestal)},
       {"size", ".048 .048 .08"},
       {"rgba", ".16 .20 .26 1"},
       {"contype", "0"},
       {"conaffinity", "0"}});
  xmlFreeDoc(doc);
}
}  // namespace
std::string build_scene(const std::filesystem::path& repo, bool dual, bool fast) {
  auto doc = xmlNewDoc(BAD_CAST "1.0");
  auto root = xmlNewNode(nullptr, BAD_CAST "mujoco");
  xmlDocSetRootElement(doc, root);
  set(root, "model", dual ? "Native dual friction Robotiq cube" : "Native physical cube");
  add(root, "compiler", {{"angle", "radian"}, {"autolimits", "true"}});
  auto option = add(root, "option",
                    {{"timestep", fast ? ".0001" : ".001"},
                     {"integrator", "implicitfast"},
                     {"solver", "Newton"},
                     {"iterations", "80"},
                     {"tolerance", "1e-10"},
                     {"gravity", "0 0 -9.81"}});
  if (dual) {
    set(option, "cone", "elliptic");
    set(option, "impratio", fast ? "100" : "10");
  }
  auto defaults = add(root, "default");
  add(defaults, "geom", {{"contype", "1"}, {"conaffinity", "2"}, {"friction", ".8 .002 .0001"}});
  add(defaults, "equality", {{"solref", ".004 1"}, {"solimp", ".99 .99 .001"}});
  auto visual = add(root, "visual");
  add(visual, "global", {{"offwidth", "1280"}, {"offheight", "720"}});
  add(visual, "quality", {{"shadowsize", "4096"}, {"offsamples", "4"}});
  add(visual, "headlight",
      {{"ambient", ".32 .32 .32"}, {"diffuse", ".5 .5 .5"}, {"specular", ".15 .15 .15"}});
  auto asset = add(root, "asset");
  add(asset, "texture",
      {{"name", "sky"},
       {"type", "skybox"},
       {"builtin", "gradient"},
       {"rgb1", ".035 .045 .07"},
       {"rgb2", ".10 .13 .18"},
       {"width", "512"},
       {"height", "3072"}});
  add(asset, "material",
      {{"name", "floor_mat"},
       {"rgba", ".13 .16 .21 1"},
       {"reflectance", ".15"},
       {"specular", ".25"},
       {"shininess", ".4"}});
  auto world = add(root, "worldbody");
  add(world, "light",
      {{"pos", ".15 -.2 .5"},
       {"dir", "-.25 .3 -1"},
       {"diffuse", ".85 .85 .85"},
       {"castshadow", "true"}});
  add(world, "light",
      {{"pos", "-.2 -.1 .25"},
       {"dir", ".5 .1 -1"},
       {"diffuse", ".45 .48 .55"},
       {"castshadow", "false"}});
  add(world, "geom",
      {{"name", "floor"},
       {"type", "plane"},
       {"size", "1 1 .01"},
       {"material", "floor_mat"},
       {"contype", "2"},
       {"conaffinity", "1"}});
  if (!dual) {
    add(world, "geom",
        {{"name", "plinth"},
         {"type", "cylinder"},
         {"pos", "0 0 .012"},
         {"size", ".055 .012"},
         {"rgba", ".075 .09 .12 1"},
         {"contype", "0"},
         {"conaffinity", "0"}});
    add(world, "geom",
        {{"name", "core_support"},
         {"type", "cylinder"},
         {"pos", "0 0 .051"},
         {"size", ".003 .027"},
         {"rgba", ".22 .25 .3 1"},
         {"contype", "0"},
         {"conaffinity", "0"}});
  }
  Vec origin(0, 0, dual ? .22 : .105);
  auto core = add(world, "body", {{"name", "core"}, {"pos", vec(origin)}});
  if (dual) add(core, "freejoint", {{"name", "cube_free"}});
  add(core, "geom",
      {{"type", "sphere"},
       {"size", ".008"},
       {"rgba", ".04 .05 .07 1"},
       {"contype", "0"},
       {"conaffinity", "0"}});
  auto actuators = add(root, "actuator");
  for (char face : faces) {
    auto [axis, sign] = face_axis(face);
    Vec n = Vec::Unit(axis) * sign;
    std::string f(1, face);
    auto center = add(core, "body", {{"name", "center_" + f}});
    add(center, "joint",
        {{"name", "hinge_" + f},
         {"type", "hinge"},
         {"axis", vec(n)},
         {"damping", ".00002"},
         {"armature", ".000001"}});
    geometry(center, n, n * pitch);
    add(actuators, "position",
        {{"name", "drive_" + f},
         {"joint", "hinge_" + f},
         {"kp", ".3"},
         {"kv", ".003"},
         {"forcerange", "-.15 .15"}});
  }
  auto eq = add(root, "equality");
  int index = 0;
  for (int x = -1; x <= 1; x++)
    for (int y = -1; y <= 1; y++)
      for (int z = -1; z <= 1; z++)
        if ((x != 0) + (y != 0) + (z != 0) >= 2) {
          Vec slot(x, y, z);
          auto name = piece_name(index++);
          auto body = add(world, "body", {{"name", name}, {"pos", vec(origin + pitch * slot)}});
          add(body, "freejoint", {{"name", name + "_free"}});
          geometry(body, slot, Vec::Zero());
          std::vector<std::string> parents = {"core"};
          for (char f : faces) parents.push_back("center_" + std::string(1, f));
          for (auto& parent : parents) {
            auto weld = add(eq, "weld",
                            {{"name", name + "_to_" + parent},
                             {"body1", parent},
                             {"body2", name},
                             {"active", parent == "core" ? "true" : "false"},
                             {"torquescale", dual ? ".3" : ".03"}});
            if (dual) {
              set(weld, "solref", ".002 1");
              set(weld, "solimp", ".9999 .9999 .001");
            }
          }
        }
  if (dual) {
    add(eq, "weld",
        {{"name", "loading_fixture"},
         {"body1", "core"},
         {"solref", ".003 1"},
         {"solimp", ".999 .999 .001"},
         {"torquescale", ".06"}});
    for (char f : faces)
      add(eq, "joint",
          {{"name", "lock_" + std::string(1, f)},
           {"joint1", "hinge_" + std::string(1, f)},
           {"active", "false"},
           {"polycoef", "0 0 0 0 0"},
           {"solref", ".003 1"},
           {"solimp", ".999 .999 .001"}});
    for (auto hand : {"A", "B"}) gripper(root, repo, hand, fast);
  }
  if (fast) {
    visit(eq->children, [&](Node n) {
      if (!get(n, "solref").empty()) set(n, "solref", ".0003 1");
      if (xmlStrEqual(n->name, BAD_CAST "connect")) set(n, "solimp", ".9999 .9999 .001");
    });
    visit(defaults->children, [&](Node n) {
      if (xmlStrEqual(n->name, BAD_CAST "joint") && !get(n, "armature").empty())
        set(n, "armature", std::to_string(std::stod(get(n, "armature")) / 100));
    });
    for (auto h : {"A", "B"})
      for (auto side : {"right", "left"})
        add(eq, "joint",
            {{"name", std::string(h) + "_" + side + "_parallel_guide"},
             {"joint1", std::string(h) + "_" + side + "_coupler_joint"},
             {"polycoef", "0 0 0 0 0"},
             {"solref", ".0003 1"},
             {"solimp", ".9999 .9999 .001"}});
  }
  xmlChar* text = nullptr;
  int size = 0;
  xmlDocDumpFormatMemory(doc, &text, &size, 1);
  std::string result(reinterpret_cast<char*>(text), size);
  xmlFree(text);
  xmlFreeDoc(doc);
  return result;
}
}  // namespace rm::cube
