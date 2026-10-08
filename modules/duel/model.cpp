#include "model.hpp"

#include <libxml/parser.h>
#include <libxml/tree.h>

#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "physics.hpp"
namespace rm::duel {
namespace {
using Node = xmlNode*;
std::string attr(Node n, const char* k) {
  auto v = xmlGetProp(n, BAD_CAST k);
  if (!v) return {};
  std::string s(reinterpret_cast<char*>(v));
  xmlFree(v);
  return s;
}
void set(Node n, const char* k, const std::string& v) {
  xmlSetProp(n, BAD_CAST k, BAD_CAST v.c_str());
}
Node add(Node p, const char* tag,
         std::initializer_list<std::pair<const char*, std::string>> attrs = {}) {
  auto n = xmlNewChild(p, nullptr, BAD_CAST tag, nullptr);
  for (auto& [k, v] : attrs) set(n, k, v);
  return n;
}
bool tag(Node n, const char* s) {
  return n->type == XML_ELEMENT_NODE && xmlStrEqual(n->name, BAD_CAST s);
}
Node child(Node p, const char* s, const std::string& name = "") {
  for (Node n = p->children; n; n = n->next)
    if (tag(n, s) && (name.empty() || attr(n, "name") == name)) return n;
  throw std::runtime_error(std::string("MJCF missing ") + s + " " + name);
}
void visit(Node n, const std::function<void(Node)>& f) {
  if (n->type == XML_ELEMENT_NODE) f(n);
  for (auto c = n->children; c; c = c->next) visit(c, f);
}
std::string xyz(double x, double y, double z) {
  std::ostringstream s;
  s.precision(15);
  s << x << ' ' << y << ' ' << z;
  return s.str();
}
}  // namespace
mjModel* build_model(const std::filesystem::path& root) {
  std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)> doc(
      xmlReadFile((root / "assets/robot.xml").c_str(), nullptr, XML_PARSE_NONET), xmlFreeDoc);
  if (!doc) throw std::runtime_error("cannot read robot.xml");
  auto tree = xmlDocGetRootElement(doc.get());
  set(tree, "model", "RoboMaster RGB vision duel");
  set(child(tree, "compiler"), "meshdir", (root / "assets/meshes").string());
  auto global = child(child(tree, "visual"), "global");
  set(global, "offwidth", "1000");
  set(global, "offheight", "750");
  auto asset = child(tree, "asset");
  add(asset, "texture",
      {{"name", "number_3"},
       {"type", "2d"},
       {"file", (root / "assets/armor_labels/3.png").string()}});
  add(asset, "material",
      {{"name", "number_3"},
       {"texture", "number_3"},
       {"texrepeat", "1 1"},
       {"texuniform", "false"},
       {"rgba", "1 1 1 1"},
       {"emission", ".3"},
       {"specular", "0"}});
  std::string vertices;
  for (double x : {-.0001, .0001})
    for (auto yz : {std::pair{-.055, -.055}, std::pair{.055, -.055}, std::pair{.055, .055},
                    std::pair{-.055, .055}})
      vertices += xyz(x, yz.first, yz.second) + " ";
  add(asset, "mesh",
      {{"name", "number_plate"},
       {"vertex", vertices},
       {"texcoord", "0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0"},
       {"face", "4 5 6 4 6 7 0 2 1 0 3 2 0 1 5 0 5 4 1 2 6 1 6 5 2 3 7 2 7 6 3 0 4 3 4 7"}});
  add(asset, "texture",
      {{"type", "skybox"},
       {"builtin", "gradient"},
       {"rgb1", ".13 .19 .29"},
       {"rgb2", ".025 .035 .06"},
       {"width", "512"},
       {"height", "3072"}});
  set(child(asset, "material", "floor"), "reflectance", "0");
  for (auto team : {"blue", "red"})
    add(asset, "material",
        {{"name", std::string(team) + "_lamp"},
         {"rgba", std::string(team) == "blue" ? ".03 .12 1 1" : "1 .025 .025 1"},
         {"emission", "1"},
         {"specular", "0"}});
  auto world = child(tree, "worldbody");
  for (auto n = world->children; n; n = n->next)
    if (tag(n, "light")) set(n, "castshadow", "false");
  auto robot_template = child(world, "body", "chassis");
  xmlUnlinkNode(robot_template);
  auto drive_template = child(tree, "actuator");
  xmlUnlinkNode(drive_template);
  auto actuators = add(tree, "actuator");
  set(child(world, "geom", "ground"), "size", "6 4 .1");
  set(child(world, "geom", "ground"), "conaffinity", "6");
  for (int axis = 0; axis < 2; ++axis)
    for (int sign : {-1, 1})
      add(world, "geom",
          {{"name", "wall_" + std::to_string(axis) + "_" + std::to_string(sign)},
           {"type", "box"},
           {"pos", axis == 0 ? xyz(sign * 6, 0, .3) : xyz(0, sign * 4, .3)},
           {"size", axis == 0 ? ".06 4 .3" : "6 .06 .3"},
           {"rgba", ".25 .31 .4 1"},
           {"contype", "1"},
           {"conaffinity", "6"}});
  for (std::string team : {"blue", "red"}) {
    auto robot = xmlCopyNode(robot_template, 1);
    visit(robot, [&](Node n) {
      for (auto key : {"name", "joint"}) {
        auto a = attr(n, key);
        if (!a.empty()) set(n, key, team + "_" + a);
      }
      auto name = attr(n, "name");
      if (tag(n, "geom") && name.find("light_") != std::string::npos) {
        xmlUnsetProp(n, BAD_CAST "rgba");
        set(n, "material", team + "_lamp");
      }
      if (name.find("armor_collision_") != std::string::npos) set(n, "group", "3");
    });
    set(robot, "pos", team == "blue" ? "-2 0 .08" : "2 0 .08");
    set(robot, "euler", xyz(0, 0, team == "blue" ? 0 : pi));
    for (std::string side : {"front", "left", "rear", "right"})
      visit(robot, [&](Node n) {
        if (tag(n, "body") && attr(n, "name") == team + "_armor_" + side)
          add(n, "geom",
              {{"name", team + "_number_" + side},
               {"type", "mesh"},
               {"mesh", "number_plate"},
               {"pos", ".0103 0 0"},
               {"material", "number_3"},
               {"contype", "0"},
               {"conaffinity", "0"},
               {"mass", "0"}});
      });
    xmlAddChild(world, robot);
    auto yaw = add(robot, "body", {{"name", team + "_yaw"}, {"pos", "0 0 .29"}});
    add(yaw, "joint",
        {{"name", team + "_yaw_joint"},
         {"axis", "0 0 1"},
         {"damping", ".8"},
         {"armature", ".025"}});
    add(yaw, "geom",
        {{"type", "cylinder"}, {"size", ".065 .035"}, {"mass", ".4"}, {"rgba", ".22 .27 .34 1"}});
    auto pitch = add(yaw, "body", {{"name", team + "_pitch"}, {"pos", "0 0 .045"}});
    add(pitch, "joint",
        {{"name", team + "_pitch_joint"},
         {"axis", "0 -1 0"},
         {"range", "-.45 .5"},
         {"damping", ".4"},
         {"armature", ".012"}});
    add(pitch, "geom",
        {{"type", "box"},
         {"pos", ".025 0 0"},
         {"size", ".075 .043 .028"},
         {"mass", ".35"},
         {"rgba", ".13 .18 .23 1"}});
    for (int j = 0; j < 16; ++j) {
      double a = 2 * pi * j / 16;
      add(pitch, "geom",
          {{"type", "box"},
           {"pos", xyz(.1485, .010 * std::cos(a), .010 * std::sin(a))},
           {"euler", xyz(a, 0, 0)},
           {"size", ".0485 .0005 .002"},
           {"mass", ".002"},
           {"rgba", ".47 .51 .58 1"}});
    }
    add(pitch, "site",
        {{"name", team + "_muzzle"}, {"pos", ".197 0 0"}, {"size", ".003"}, {"rgba", "0 0 0 0"}});
    add(pitch, "geom",
        {{"type", "box"},
         {"pos", ".04 0 .046"},
         {"size", ".022 .025 .017"},
         {"mass", ".05"},
         {"rgba", ".12 .15 .18 1"}});
    add(pitch, "camera",
        {{"name", team + "_camera"},
         {"pos", ".065 0 .046"},
         {"xyaxes", "0 -1 0 0 0 1"},
         {"fovy", "45"}});
    visit(robot, [&](Node n) {
      if (tag(n, "geom") && attr(n, "contype") != "0") {
        set(n, "contype", team == "blue" ? "2" : "4");
        set(n, "conaffinity", team == "blue" ? "5" : "3");
      }
    });
    for (auto n = drive_template->children; n; n = n->next)
      if (n->type == XML_ELEMENT_NODE) {
        auto item = xmlCopyNode(n, 1);
        set(item, "name", team + "_" + attr(item, "name"));
        set(item, "joint", team + "_" + attr(item, "joint"));
        set(item, "ctrlrange", "-85 85");
        xmlAddChild(actuators, item);
      }
    for (std::string axis : {"yaw", "pitch"})
      add(actuators, "position",
          {{"name", team + "_" + axis + "_servo"},
           {"joint", team + "_" + axis + "_joint"},
           {"kp", "65"},
           {"kv", "5"},
           {"ctrlrange", axis == "yaw" ? "-6.2832 6.2832" : "-.45 .5"},
           {"forcerange", "-8 8"}});
  }
  xmlFreeNode(robot_template);
  xmlFreeNode(drive_template);
  xmlChar* buffer = nullptr;
  int length = 0;
  xmlDocDumpMemory(doc.get(), &buffer, &length);
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "duel.xml", buffer, length);
  xmlFree(buffer);
  char error[2048];
  auto model = mj_loadXML("duel.xml", &vfs, error, sizeof(error));
  mj_deleteVFS(&vfs);
  if (!model) throw std::runtime_error(error);
  return model;
}
}  // namespace rm::duel
