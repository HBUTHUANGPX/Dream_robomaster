#include "mechanical_rx.hpp"

#include <libxml/parser.h>

#include <iomanip>
#include <sstream>

#include "rm/rx_gripper.hpp"

namespace rm::cube {
namespace {
using Node = xmlNodePtr;
std::string get(Node n, const char* key) {
  auto p = xmlGetProp(n, BAD_CAST key);
  std::string s = p ? reinterpret_cast<char*>(p) : "";
  xmlFree(p);
  return s;
}
void set(Node n, const char* key, const std::string& value) {
  xmlSetProp(n, BAD_CAST key, BAD_CAST value.c_str());
}
Node child(Node n, const char* tag) {
  for (auto p = n->children; p; p = p->next)
    if (p->type == XML_ELEMENT_NODE && xmlStrEqual(p->name, BAD_CAST tag)) return p;
  return nullptr;
}
Node add(Node n, const char* tag,
         std::initializer_list<std::pair<const char*, std::string>> attrs = {}) {
  auto p = xmlNewChild(n, nullptr, BAD_CAST tag, nullptr);
  for (auto& [k, v] : attrs) set(p, k, v);
  return p;
}
}  // namespace
std::string mechanical_rx_scene(const std::string& xml, const std::filesystem::path& bundle,
                                double bearing_friction, double tip_friction,
                                double tip_contact_time, double cube_mass, bool corner_spheres) {
  std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)> doc(
      xmlReadMemory(xml.data(), xml.size(), "mechanical.xml", nullptr, XML_PARSE_NONET),
      xmlFreeDoc);
  if (!doc) throw std::runtime_error("Cannot parse mechanical scene");
  auto root = xmlDocGetRootElement(doc.get()), world = child(root, "worldbody");
  auto motors = child(root, "actuator");
  xmlUnlinkNode(motors);
  xmlFreeNode(motors);
  auto defaults = child(child(root, "default"), "geom");
  set(defaults, "contype", "16");
  set(defaults, "conaffinity", "31");
  auto core = add(world, "body", {{"name", "mechanical_core"}, {"pos", "0 0 .1"}});
  add(core, "freejoint", {{"name", "mechanical_core_free"}});
  const double scale = cube_mass / .088;
  std::ostringstream mass, inertia;
  mass << std::setprecision(17) << .01 * scale;
  inertia << std::setprecision(17) << .0000005 * scale << " " << .0000005 * scale << " "
          << .0000005 * scale;
  add(core, "inertial", {{"pos", "0 0 0"}, {"mass", mass.str()}, {"diaginertia", inertia.str()}});
  std::vector<std::string> shells;
  for (auto n = world->children; n;) {
    auto next = n->next;
    if (n != core && n->type == XML_ELEMENT_NODE) {
      if (xmlStrEqual(n->name, BAD_CAST "geom")) {
        xmlUnlinkNode(n);
        xmlAddChild(core, n);
        set(n, "pos", "0 0 0");
      } else if (xmlStrEqual(n->name, BAD_CAST "body")) {
        shells.push_back(get(n, "name") + "_shell");
        if (auto hinge = child(n, "joint")) {
          if (get(hinge, "type") == "hinge")
            set(hinge, "frictionloss", std::to_string(bearing_friction));
          xmlUnlinkNode(n);
          xmlAddChild(core, n);
          set(n, "pos", "0 0 0");
        }
      }
    }
    n = next;
  }
  append_rx_grippers(root, bundle);
  std::vector<std::string> tips;
  // 保留原方块的质量与惯性、净空包络；四球只替换接触几何。
  auto replace_tips = [&](auto&& self, Node parent) -> void {
    for (auto n = parent->children; n;) {
      auto next = n->next;
      const auto name = get(n, "name");
      if (n->type == XML_ELEMENT_NODE && xmlStrEqual(n->name, BAD_CAST "geom") &&
          (name == "A_left_tip" || name == "A_right_tip" || name == "B_left_tip" ||
           name == "B_right_tip")) {
        if (!corner_spheres) {
          tips.push_back(name);
        } else {
          set(n, "contype", "0");
          set(n, "conaffinity", "0");
          set(n, "rgba", ".1 .15 .2 .15");
          double x, y, z;
          std::istringstream position(get(n, "pos"));
          if (!(position >> x >> y >> z)) throw std::runtime_error("Invalid RX tip position");
          int index = 0;
          for (double dx : {-.005, .005})
            for (double dz : {-.005, .005}) {
              auto sphere = xmlCopyNode(n, 1);
              xmlAddChild(parent, sphere);
              const auto sphere_name =
                  name.substr(0, name.size() - 4) + "_corner_" + std::to_string(index++) + "_tip";
              std::ostringstream p;
              p << std::setprecision(17) << x + dx << " " << y << " " << z + dz;
              set(sphere, "name", sphere_name);
              set(sphere, "type", "sphere");
              set(sphere, "size", ".003");
              set(sphere, "pos", p.str());
              set(sphere, "mass", "0");
              set(sphere, "contype", "1");
              set(sphere, "conaffinity", "15");
              set(sphere, "rgba", ".1 .65 .85 1");
              tips.push_back(sphere_name);
            }
        }
      } else if (n->type == XML_ELEMENT_NODE) {
        self(self, n);
      }
      n = next;
    }
  };
  replace_tips(replace_tips, world);
  if (tips.size() != (corner_spheres ? 16 : 4))
    throw std::runtime_error("Invalid RX contact tip count");
  for (auto n = world->children; n; n = n->next)
    if (get(n, "name") == "A_wrist" || get(n, "name") == "B_wrist") set(n, "pos", "0 0 .1");
  auto contact = child(root, "contact");
  if (!contact) contact = add(root, "contact");
  for (const auto& tip : tips)
    for (auto& shell : shells)
      add(contact, "pair",
          {{"geom1", tip},
           {"geom2", shell},
           {"condim", "4"},
           {"friction", std::to_string(tip_friction) + " " + std::to_string(tip_friction) +
                            (tip_friction > 0 ? " .002 .0001 .0001" : " 0 0 0")},
           {"solref", std::to_string(tip_contact_time) + " 1"},
           {"solimp", ".99 .999 .001"},
           {"margin", "0"}});
  auto equality = child(root, "equality");
  add(equality, "weld",
      {{"name", "mechanical_loading_fixture"},
       {"body1", "mechanical_core"},
       {"solref", ".004 1"},
       {"solimp", ".9999 .9999 .001"}});
  xmlChar* data = nullptr;
  int size = 0;
  xmlDocDumpMemory(doc.get(), &data, &size);
  std::string result(reinterpret_cast<char*>(data), size);
  xmlFree(data);
  return result;
}
}  // namespace rm::cube
