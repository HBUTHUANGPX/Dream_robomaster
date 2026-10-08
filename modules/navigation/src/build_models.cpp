// Model generation preserves the baseline's original CAD visuals and roller contacts.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "rm/navigation.hpp"
using namespace rm::nav;
namespace {
std::string vector(const V3& v) {
  std::ostringstream s;
  s << std::setprecision(12) << v.x() << ' ' << v.y() << ' ' << v.z();
  return s.str();
}
std::string number(double value) {
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}
struct XML {
  std::ostringstream out;
  void open(const std::string& tag, const std::string& attributes = "") {
    out << '<' << tag << ' ' << attributes << ">\n";
  }
  void close(const std::string& tag) { out << "</" << tag << ">\n"; }
  void add(const std::string& tag, const std::string& attributes) {
    out << '<' << tag << ' ' << attributes << "/>\n";
  }
};
std::string build(const std::filesystem::path& root, bool navigation) {
  XML x;
  x.open("mujoco", "model=\"RoboMaster simplified infantry\"");
  x.add("compiler", "angle=\"radian\" meshdir=\"" + (root / "assets/meshes").string() +
                        "\" autolimits=\"true\"");
  x.add("option", "timestep=\"0.001\" integrator=\"implicitfast\" cone=\"elliptic\" iterations=\"" +
                      std::string(navigation ? "40" : "60") + "\" gravity=\"0 0 -9.81\"");
  x.open("default");
  x.add("geom",
        "friction=\"1.0 0.002 0.0001\" solref=\"0.008 1\" solimp=\"0.95 0.99 0.001\" condim=\"4\"");
  x.close("default");
  x.open("visual");
  x.add("global", "offwidth=\"1280\" offheight=\"960\"");
  x.close("visual");
  x.open("asset");
  for (std::string name : {"armor_am02", "armor_frame_a"})
    x.add("mesh", "name=\"" + name + "\" file=\"" + name + ".obj\"");
  x.add("texture",
        "name=\"tiles\" type=\"2d\" builtin=\"checker\" rgb1=\".16 .19 .23\" rgb2=\".23 .27 .31\" "
        "width=\"512\" height=\"512\"");
  x.add("material", "name=\"floor\" texture=\"tiles\" texrepeat=\"12 12\" reflectance=\".1\"");
  std::vector<Triangle> triangles;
  if (navigation) {
    triangles = load_stl(root / "assets/arena/rmuc2023.stl");
    x.add("mesh",
          "name=\"arena_visual\" file=\"" + (root / "assets/arena/rmuc2023.stl").string() + "\"");
    for (size_t i = 0; i < triangles.size(); ++i) {
      auto t = triangles[i];
      V3 center = (t.v[0] + t.v[1] + t.v[2]) / 3 - .0015 * t.normal;
      std::string vertices;
      for (int side = 0; side < 2; ++side)
        for (auto v : t.v) vertices += vector(v - .003 * side * t.normal - center) + " ";
      x.add("mesh", "name=\"contact_" + std::to_string(i) + "\" vertex=\"" + vertices +
                        "\" face=\"0 1 2 3 5 4 0 3 4 0 4 1 1 4 5 1 5 2 2 5 3 2 3 0\"");
    }
  }
  x.close("asset");
  x.open("worldbody");
  if (navigation) {
    x.add("light",
          "pos=\"7 10 15\" dir=\"0 0 -1\" directional=\"true\" diffuse=\".9 .9 .9\" ambient=\".5 "
          ".5 .5\" castshadow=\"false\"");
    x.add("geom",
          "name=\"arena_visual\" type=\"mesh\" mesh=\"arena_visual\" contype=\"0\" "
          "conaffinity=\"0\" group=\"0\" rgba=\".36 .43 .48 1\"");
    for (size_t i = 0; i < triangles.size(); ++i) {
      auto t = triangles[i];
      V3 center = (t.v[0] + t.v[1] + t.v[2]) / 3 - .0015 * t.normal;
      x.add("geom", "name=\"terrain_" + std::to_string(i) + "\" type=\"mesh\" mesh=\"contact_" +
                        std::to_string(i) + "\" pos=\"" + vector(center) +
                        "\" contype=\"1\" conaffinity=\"2\" group=\"3\" rgba=\"0 0 0 0\" "
                        "friction=\"1.3 .002 .0001\"");
    }
  } else {
    x.add("light",
          "pos=\"1 -2 3\" dir=\"-0.3 0.3 -1\" diffuse=\".9 .9 .9\" ambient=\".35 .35 .35\"");
    x.add("light", "pos=\"-2 1 2\" dir=\"0 0 -1\" diffuse=\".5 .5 .5\"");
    x.add("geom",
          "name=\"ground\" type=\"plane\" size=\"6 6 .1\" material=\"floor\" contype=\"1\" "
          "conaffinity=\"2\"");
  }
  x.open("body",
         "name=\"chassis\" pos=\"" + std::string(navigation ? "7.5 4 .182" : "0 0 .08") + "\"");
  x.add("freejoint", "name=\"root\"");
  auto geom = [&](std::string attributes) {
    x.add("geom", attributes + (navigation ? " group=\"1\"" : ""));
  };
  geom(
      "name=\"lower_deck\" type=\"box\" pos=\"0 0 .045\" size=\".205 .135 .018\" mass=\"8\" "
      "rgba=\".12 .15 .2 1\" contype=\"2\" conaffinity=\"1\"");
  geom(
      "name=\"upper_deck\" type=\"box\" pos=\"0 0 .135\" size=\".12 .12 .012\" mass=\"2\" "
      "rgba=\".28 .33 .39 1\" contype=\"2\" conaffinity=\"1\"");
  geom(
      "name=\"battery\" type=\"box\" pos=\"-.045 0 .085\" size=\".075 .06 .023\" mass=\"2\" "
      "rgba=\".08 .09 .12 1\" contype=\"0\" conaffinity=\"0\"");
  geom(
      "name=\"front_marker\" type=\"box\" pos=\".09 0 .151\" size=\".018 .04 .004\" mass=\".01\" "
      "rgba=\".95 .5 .08 1\" contype=\"0\" conaffinity=\"0\"");
  for (double a : {-.1, .1})
    for (double b : {-.1, .1})
      geom("type=\"cylinder\" pos=\"" + vector(V3(a, b, .09)) +
           "\" size=\".009 .035\" mass=\".08\" rgba=\".45 .49 .54 1\" contype=\"0\" "
           "conaffinity=\"0\"");
  for (int axis = 0; axis < 2; ++axis)
    for (int sign : {-1, 1}) {
      V3 p(0, 0, .022);
      p[axis] = sign * .275;
      geom("name=\"bumper_" + std::to_string(axis) + "_" + std::to_string(sign) +
           "\" type=\"box\" pos=\"" + vector(p) + "\" size=\"" +
           std::string(axis == 1 ? ".285 .01 .009" : ".01 .265 .009") +
           "\" mass=\".25\" rgba=\".32 .37 .43 1\" contype=\"2\" conaffinity=\"1\"");
    }
  const std::array<std::pair<std::string, double>, 4> armors{
      {{"front", 0}, {"left", pi / 2}, {"rear", pi}, {"right", -pi / 2}}};
  for (auto [name, angle] : armors) {
    x.open("body", "name=\"armor_mount_" + name + "\" pos=\"" +
                       vector(V3(.245 * std::cos(angle), .245 * std::sin(angle), .175)) +
                       "\" euler=\"" + vector(V3(0, 0, angle)) + "\"");
    x.open("body", "name=\"armor_" + name + "\" euler=\"" + vector(V3(0, -pi / 12, 0)) + "\"");
    x.add("inertial", "pos=\"0 0 0\" mass=\".359\" diaginertia=\".001 .0005 .0006\"");
    geom(
        "type=\"mesh\" mesh=\"armor_am02\" rgba=\".21 .24 .29 1\" contype=\"0\" conaffinity=\"0\" "
        "mass=\"0\"");
    geom("name=\"armor_collision_" + name +
         "\" type=\"box\" size=\".0095 .0705 .065\" rgba=\"0 0 0 0\" contype=\"2\" "
         "conaffinity=\"1\" mass=\"0\"");
    x.add("site",
          "name=\"armor_face_" + name + "\" pos=\".0095 0 0\" size=\".001\" rgba=\"0 0 0 0\"");
    for (int side : {-1, 1})
      geom("name=\"light_" + name + "_" + std::to_string(side) + "\" type=\"box\" pos=\"" +
           vector(V3(.0097, side * .062, 0)) +
           "\" size=\".0005 .003 .026\" rgba=\".05 .4 1 1\" contype=\"0\" conaffinity=\"0\" "
           "mass=\"0\"");
    x.close("body");
    for (int side : {-1, 1})
      geom("type=\"mesh\" mesh=\"armor_frame_a\" pos=\"" +
           vector(V3(-.009176295, side * .0475, -.002458781)) +
           "\" rgba=\".48 .51 .55 1\" contype=\"0\" conaffinity=\"0\" mass=\".06\"");
    geom(
        "type=\"box\" pos=\"-.09 0 -.045\" size=\".07 .045 .007\" mass=\".15\" rgba=\".28 .32 .38 "
        "1\" contype=\"0\" conaffinity=\"0\"");
    geom(
        "type=\"box\" pos=\"-.034 0 -.034868407\" size=\".026 .062 .003\" mass=\".15\" rgba=\".3 "
        ".34 .4 1\" contype=\"0\" conaffinity=\"0\"");
    for (double a : {-.044894086, -.019894086})
      for (double b : {-.0475, .0475})
        x.add("site", "name=\"m4_" + name + "_" + number(a) + "_" + number(b) + "\" pos=\"" +
                          vector(V3(a, b, -.031868407)) + "\" size=\".0021\" rgba=\".8 .8 .8 1\"");
    x.close("body");
  }
  const char* names[] = {"fl", "fr", "rl", "rr"};
  for (int i = 0; i < 4; ++i) {
    std::string name = names[i];
    double a = i < 2 ? .18 : -.18, b = i % 2 == 0 ? .18 : -.18, hand = i == 0 || i == 3 ? -1 : 1;
    geom("type=\"cylinder\" fromto=\"" + vector(V3(a, b * .6, 0)) + " " + vector(V3(a, b, 0)) +
         "\" size=\".012\" mass=\".12\" rgba=\".38 .42 .47 1\" contype=\"0\" conaffinity=\"0\"");
    geom("type=\"box\" pos=\"" + vector(V3(a, b * .6, .025)) +
         "\" size=\".025 .016 .03\" mass=\".1\" rgba=\".2 .23 .27 1\" contype=\"0\" "
         "conaffinity=\"0\"");
    x.open("body", "name=\"wheel_" + name + "\" pos=\"" + vector(V3(a, b, 0)) + "\"");
    x.add("joint", "name=\"wheel_" + name +
                       "\" type=\"hinge\" axis=\"0 1 0\" damping=\".02\" armature=\".003\"");
    geom(
        "type=\"cylinder\" size=\".049 .022\" quat=\".70710678 .70710678 0 0\" mass=\".45\" "
        "rgba=\".45 .49 .55 1\" contype=\"0\" conaffinity=\"0\"");
    for (int j = 0; j < 12; ++j) {
      double theta = 2 * pi * j / 12;
      std::string suffix = name + "_" + std::to_string(j);
      x.open("body", "name=\"roller_" + suffix + "\" pos=\"" +
                         vector(V3(.062 * std::sin(theta), 0, -.062 * std::cos(theta))) +
                         "\" zaxis=\"" +
                         vector(V3(hand * std::cos(theta), 1, hand * std::sin(theta))) + "\"");
      x.add("joint",
            "name=\"roller_" + suffix +
                "\" type=\"hinge\" axis=\"0 0 1\" damping=\".00002\" armature=\".000001\"");
      geom("name=\"roller_geom_" + suffix +
           "\" type=\"ellipsoid\" size=\".014 .014 .038\" mass=\".025\" rgba=\".055 .06 .07 1\" "
           "contype=\"2\" conaffinity=\"1\"" +
           (navigation ? std::string(" friction=\"1.3 .002 .0001\"") : ""));
      x.close("body");
    }
    x.close("body");
  }
  if (navigation) {
    for (bool inverted : {false, true}) {
      std::string name = inverted ? "mid360_ground" : "mid360_top";
      geom("name=\"" + name + "_mast\" type=\"cylinder\" fromto=\"" +
           std::string(inverted ? "-.19 0 .15 -.19 0 .515" : "0 0 .15 0 0 .460") + "\" size=\"" +
           std::string(inverted ? ".01" : ".012") +
           "\" mass=\".12\" rgba=\".5 .55 .6 1\" contype=\"0\" conaffinity=\"0\"");
      geom("name=\"" + name + (inverted ? "_arm" : "_plate") + "\" type=\"box\" pos=\"" +
           std::string(inverted ? "-.145 0 .507" : "0 0 .463") + "\" size=\"" +
           std::string(inverted ? ".055 .05 .003" : ".05 .05 .003") +
           "\" mass=\".08\" rgba=\".5 .55 .6 1\" contype=\"0\" conaffinity=\"0\"");
      x.open("body", "name=\"" + name + "\" pos=\"" +
                         std::string(inverted ? "-.11 0 .44" : "0 0 .53") + "\" quat=\"" +
                         std::string(inverted ? "0 1 0 0" : "1 0 0 0") + "\"");
      geom("name=\"" + name +
           "_case\" type=\"box\" pos=\"0 0 -.034\" size=\".0325 .0325 .03\" mass=\".265\" "
           "rgba=\".16 .19 .22 1\" contype=\"2\" conaffinity=\"1\"");
      geom(
          "type=\"cylinder\" pos=\"0 0 -.002\" size=\".029 .005\" mass=\".001\" rgba=\".05 .8 .65 "
          "1\" contype=\"0\" conaffinity=\"0\"");
      x.add("site", "name=\"" + name +
                        "_origin\" pos=\"0 0 0\" size=\".002\" rgba=\"0 0 0 0\" group=\"1\"");
      x.close("body");
    }
    x.add("site", "name=\"imu\" pos=\"0 0 .1\" size=\".003\" rgba=\"0 0 0 0\"");
  }
  x.close("body");
  x.close("worldbody");
  x.open("actuator");
  for (std::string name : names)
    x.add("velocity", "name=\"drive_" + name + "\" joint=\"wheel_" + name + "\" kv=\"" +
                          std::string(navigation ? "4" : "2") +
                          "\" ctrlrange=\"-35 35\" forcerange=\"" +
                          std::string(navigation ? "-10 10" : "-5 5") + "\"");
  x.close("actuator");
  if (navigation) {
    x.open("sensor");
    x.add("gyro", "name=\"gyro\" site=\"imu\"");
    x.add("accelerometer", "name=\"accel\" site=\"imu\"");
    x.add("framequat", "name=\"attitude\" objtype=\"site\" objname=\"imu\"");
    x.close("sensor");
  }
  x.close("mujoco");
  return x.out.str();
}
}  // namespace
int main(int argc, char** argv) {
  try {
    auto root = rm::repo_root(argc, argv);
    std::filesystem::path output;
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      if ((a == "--root" || a == "--output") && i + 1 < argc) {
        if (a == "--output") output = argv[i + 1];
        ++i;
      } else
        throw std::invalid_argument(
            "用法：rm_navigation_build_models --root 仓库路径 --output 输出目录");
    }
    if (output.empty()) throw std::invalid_argument("--output required");
    std::filesystem::create_directories(output);
    for (bool navigation : {false, true}) {
      auto path = output / (navigation ? "navigation.xml" : "robot.xml");
      std::ofstream out(path);
      out << build(root, navigation);
      out.close();
      rm::Simulation simulation(path);
      std::cout << path << " nq=" << simulation.model->nq << " nu=" << simulation.model->nu << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
