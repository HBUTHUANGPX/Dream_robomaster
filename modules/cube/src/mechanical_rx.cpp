#include "mechanical_rx.hpp"

#include <libxml/parser.h>

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
                                double bearing_friction, double tip_friction) {
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
  add(core, "inertial",
      {{"pos", "0 0 0"}, {"mass", ".01"}, {"diaginertia", ".0000005 .0000005 .0000005"}});
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
  for (auto n = world->children; n; n = n->next)
    if (get(n, "name") == "A_wrist" || get(n, "name") == "B_wrist") set(n, "pos", "0 0 .1");
  auto contact = child(root, "contact");
  if (!contact) contact = add(root, "contact");
  for (auto h : {"A", "B"})
    for (auto side : {"left", "right"})
      for (auto& shell : shells)
        add(contact, "pair",
            {{"geom1", std::string(h) + "_" + side + "_tip"},
             {"geom2", shell},
             {"condim", "4"},
             {"friction", std::to_string(tip_friction) + " " + std::to_string(tip_friction) +
                              (tip_friction > 0 ? " .002 .0001 .0001" : " 0 0 0")},
             {"solref", ".004 1"},
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
