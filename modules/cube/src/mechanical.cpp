#include "rm/mechanical.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

#include "mechanical_rx.hpp"
#include "rm/cube.hpp"
#include "rm/rx_gripper.hpp"
#include "video.hpp"

namespace {
using namespace rm;
using namespace rm::cube;
constexpr double pitch_m = .055 / 3;
Json settings = {{"shell_gap_m", .00005},
                 {"shell_bevel_m", .00065},
                 {"cap_inner_m", {.01460, .01265, .01085}},
                 {"cap_outer_m", {.0162, .0143, .0125}},
                 {"cap_angle_deg", {43., 21.5, 31.}},
                 {"preload_stiffness_n_m", 500.},
                 {"preload_reference_m", -.0015},
                 {"turn_duration_s", 1.5},
                 {"timestep_s", .0005},
                 {"internal_contact_impedance", .9999},
                 {"alignment_timeout_s", 2.},
                 {"max_torque_nm", .03},
                 {"rx_jaw_torque_nm", .3},
                 {"rx_tip_friction", 1.2},
                 {"rx_bearing_friction_nm", .002},
                 {"rx_detent_torque_nm", .04},
                 {"record_fps", 0.}};
double value(const char* key) { return settings.at(key).get<double>(); }
void load_settings(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot read parameter file");
  Json custom;
  file >> custom;
  if (!custom.is_object()) throw std::invalid_argument("Parameters must be an object");
  for (const auto& [key, v] : custom.items()) {
    if (!settings.contains(key)) throw std::invalid_argument("Unknown parameter: " + key);
    if (settings[key].is_array()) {
      if (!v.is_array() || v.size() != 3)
        throw std::invalid_argument("Expected three cap parameters");
      for (const auto& item : v)
        if (!item.is_number() || !std::isfinite(item.get<double>()) || item.get<double>() <= 0)
          throw std::invalid_argument("Invalid cap parameter");
    } else if (!v.is_number() || !std::isfinite(v.get<double>()) ||
               (key != "preload_reference_m" && v.get<double>() < 0))
      throw std::invalid_argument("Invalid numeric parameter");
    settings[key] = v;
  }
  if (value("turn_duration_s") <= 0 || value("timestep_s") <= 0 || value("timestep_s") > .001 ||
      value("max_torque_nm") <= 0)
    throw std::invalid_argument("Invalid timing or torque");
  if (value("internal_contact_impedance") <= 0 || value("internal_contact_impedance") >= 1)
    throw std::invalid_argument("Contact impedance must be between zero and one");
  for (int i = 0; i < 3; ++i)
    if (settings["cap_inner_m"][i].get<double>() >= settings["cap_outer_m"][i].get<double>() ||
        settings["cap_angle_deg"][i].get<double>() >= 60)
      throw std::invalid_argument("Invalid cap dimensions");
}
struct Part {
  Vec slot;
  std::string name;
};
std::vector<Part> parts;
std::string xyz(const Vec& v) {
  std::ostringstream s;
  s << std::setprecision(12) << v.transpose();
  return s.str();
}
std::string quaternion(const Vec& n) {
  const auto q = Eigen::Quaterniond::FromTwoVectors(Vec::UnitZ(), n);
  std::ostringstream s;
  s << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z();
  return s.str();
}
std::vector<Vec> clipped_box(const Vec& c, const Vec& n) {
  std::vector<Vec> all, result;
  const double half = pitch_m / 2 - value("shell_gap_m") / 2, bevel = value("shell_bevel_m");
  for (int i = 0; i < 8; ++i)
    for (int face = 0; face < 3; ++face) {
      Vec v = c;
      for (int axis = 0; axis < 3; ++axis)
        v[axis] += (i & (1 << axis) ? 1 : -1) * (half - (axis == face ? 0 : bevel));
      all.push_back(v);
      if (n.dot(v) >= .017) result.push_back(v);
    }
  for (size_t i = 0; i < all.size(); ++i)
    for (size_t j = i + 1; j < all.size(); ++j) {
      Vec p = all[i], q = all[j];
      double u = n.dot(p) - .017, v = n.dot(q) - .017;
      if (u * v < 0) result.push_back(p + (q - p) * u / (u - v));
    }
  return result;
}
void mesh(std::ostream& s, const std::string& name, const std::vector<Vec>& vs) {
  s << "<mesh name='" << name << "' vertex='";
  for (const auto& v : vs) s << xyz(v) << ' ';
  s << "'/>\n";
}
std::string scene() {
  parts.clear();
  std::ostringstream assets, bodies, motors;
  std::array<std::vector<std::string>, 3> cap_meshes;
  // 三种卡脚处在不同球面半径。每个小楔体都是凸体，合起来保留内侧空腔。
  const auto inner = settings["cap_inner_m"].get<std::array<double, 3>>();
  const auto outer = settings["cap_outer_m"].get<std::array<double, 3>>();
  auto angle = settings["cap_angle_deg"].get<std::array<double, 3>>();
  for (auto& a : angle) a *= pi / 180;
  const int rings = 3, sectors = 16;
  for (int type = 0; type < 3; ++type) {
    auto point = [&](int r, int s) {
      double p = 2 * pi * s / sectors;
      // 中心卡脚保留对角承托翼，四个轴向缺口给角块颈部让路。
      const double limit =
          type == 0
              ? std::min(angle[type],
                         std::asin(.53 / std::max(std::abs(std::cos(p)), std::abs(std::sin(p)))))
              : angle[type];
      double t = limit * r / rings;
      return Vec(std::sin(t) * std::cos(p), std::sin(t) * std::sin(p), std::cos(t));
    };
    auto wedge = [&](Vec a, Vec b, Vec c) {
      std::string name =
          "cap_" + std::to_string(type) + "_" + std::to_string(cap_meshes[type].size());
      mesh(assets, name,
           {inner[type] * a, inner[type] * b, inner[type] * c, outer[type] * a, outer[type] * b,
            outer[type] * c});
      cap_meshes[type].push_back(name);
    };
    for (int r = 0; r < rings; ++r)
      for (int s = 0; s < sectors; ++s) {
        wedge(point(r, s), point(r + 1, s), point(r + 1, s + 1));
        if (r) wedge(point(r, s), point(r + 1, s + 1), point(r, s + 1));
      }
  }
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        Vec slot(x, y, z);
        int nonzero = (x != 0) + (y != 0) + (z != 0);
        if (!nonzero) continue;
        Vec n = slot.normalized(), c = pitch_m * slot;
        std::string name = "piece_" + std::to_string(parts.size());
        parts.push_back({slot, name});
        mesh(assets, name + "_shell", clipped_box(c, n));
        bodies << "<body name='" << name << "' pos='0 0 .1'>";
        if (nonzero == 1) {
          std::string face;
          for (char f : faces) {
            auto [axis, sign] = face_axis(f);
            if (slot == Vec::Unit(axis) * sign) face = f;
          }
          bodies << "<joint name='hinge_" << face << "' type='hinge' axis='" << xyz(n)
                 << "' damping='.00002' armature='.00000001'/>";
          bodies << "<joint name='preload_" << face << "' type='slide' axis='" << xyz(n)
                 << "' stiffness='" << value("preload_stiffness_n_m") << "' springref='"
                 << value("preload_reference_m")
                 << "' damping='.2' limited='true' "
                    "range='-.0015 .0015'/>";
          motors << "<position name='drive_" << face << "' joint='hinge_" << face
                 << "' kp='.3' kv='.003' forcerange='" << -value("max_torque_nm") << " "
                 << value("max_torque_nm") << "'/>";
        } else
          bodies << "<freejoint/>";
        bodies << "<inertial pos='" << xyz(c)
               << "' mass='.003' diaginertia='.00000016 .00000016 .00000016'/>";
        bodies << "<geom name='" << name << "_shell' type='mesh' mesh='" << name
               << "_shell' rgba='.08 .08 .08 1'/>";
        for (const auto& cap : cap_meshes[nonzero - 1])
          bodies << "<geom type='mesh' mesh='" << cap << "' quat='" << quaternion(n) << "' rgba='"
                 << (nonzero == 1   ? ".2 .55 .9 1"
                     : nonzero == 2 ? ".15 .75 .45 1"
                                    : "1 .5 .12 1")
                 << "' group='3'/>";
        if (nonzero == 2) {
          Vec tangent = Vec::Zero();
          std::vector<int> axes;
          for (int a = 0; a < 3; ++a)
            if (slot[a]) axes.push_back(a);
          tangent[axes[0]] = n[axes[1]];
          tangent[axes[1]] = -n[axes[0]];
          Vec cross = n.cross(tangent);
          auto surface = [&](double a, double b) {
            return (n + std::tan(a * pi / 180) * tangent + std::tan(b * pi / 180) * cross)
                .normalized()
                .eval();
          };
          for (int i = 0; i < 8; ++i) {
            const double a = -20 + 5 * i, b = a + 5;
            std::vector<Vec> vertices;
            for (double r : {.01085, .0117})
              for (auto v : {surface(a, -2.5), surface(b, -2.5), surface(b, 2.5), surface(a, 2.5)})
                vertices.push_back(r * v);
            auto bearing = name + "_bearing_" + std::to_string(i);
            mesh(assets, bearing, vertices);
            bodies << "<geom type='mesh' mesh='" << bearing << "' rgba='.8 .5 .2 1' group='3'/>";
          }
        }
        if (nonzero > 1)
          bodies << "<geom type='capsule' fromto='" << xyz(n * .0112) << " " << xyz(n * (.018))
                 << "' size='.0004' rgba='.6 .6 .6 1' group='3'/>";
        bodies << "<site name='" << name << "_center' pos='" << xyz(c) << "' size='.0001'/>";
        for (int axis = 0; axis < 3; ++axis)
          if (slot[axis]) {
            Vec p = c, size = Vec::Constant(pitch_m / 2 - .0008);
            p[axis] += slot[axis] * (pitch_m / 2 - .00002);
            size[axis] = .00008;
            const char* color = axis == 0   ? (slot[axis] > 0 ? ".8 .08 .04 1" : "1 .35 .02 1")
                                : axis == 1 ? (slot[axis] > 0 ? ".04 .15 .85 1" : ".02 .6 .1 1")
                                            : (slot[axis] > 0 ? ".95 .95 .95 1" : ".95 .85 .02 1");
            bodies << "<geom type='box' pos='" << xyz(p) << "' size='" << xyz(size) << "' rgba='"
                   << color << "' contype='0' conaffinity='0' group='1'/>";
          }
        bodies << "</body>\n";
      }
  std::ostringstream s;
  s << "<mujoco model='Contact retained cube prototype'><compiler angle='radian'/><option "
       "timestep='"
    << value("timestep_s")
    << "' gravity='0 0 -9.81' integrator='implicitfast' solver='Newton' "
       "iterations='100' cone='elliptic' impratio='10'/><size memory='256M'/><default><geom "
       "friction='.03 .0001 .00001' solref='.001 1' solimp='"
    << value("internal_contact_impedance") << ' ' << value("internal_contact_impedance")
    << " .0001' "
       "condim='3'/></default><visual><global offwidth='800' offheight='600'/></visual><asset>"
    << assets.str()
    << "</asset><worldbody><light pos='.1 -.2 .5'/><geom type='sphere' pos='0 0 .1' size='.0107' "
       "rgba='.5 .3 .2 1' group='3'/>"
    << bodies.str() << "</worldbody><actuator>" << motors.str() << "</actuator></mujoco>";
  return s.str();
}
}  // namespace
int rm::cube::mechanical_main(const std::filesystem::path& root, int argc, char** argv) {
  const auto began = std::chrono::steady_clock::now();
  Json report = {{"passed", false},
                 {"model", "experimental_contact_retention"},
                 {"internal_constraint_switches", 0},
                 {"steps", Json::array()}};
  std::filesystem::path out = root / "output/cube-mechanical/current", config, rx_bundle;
  std::string sequence = "R U F";
  bool solve = false, record = false, prepared = false;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      auto next = [&]() {
        if (++i == argc) throw std::invalid_argument("Missing value for " + arg);
        return std::string(argv[i]);
      };
      if (arg == "--root")
        next();
      else if (arg == "--mechanical" || arg == "--headless") {
      } else if (arg == "--output")
        out = next();
      else if (arg == "--scramble" || arg == "--moves")
        sequence = next();
      else if (arg == "--mechanical-config")
        config = next();
      else if (arg == "--rx-bundle")
        rx_bundle = next();
      else if (arg == "--solve")
        solve = true;
      else if (arg == "--record")
        record = true;
      else if (arg == "--help" || arg == "-h") {
        std::cout << "机械魔方原型：./rm cube --mechanical [--scramble \"R U F\"] [--solve] "
                     "[--record] [--output 输出目录] [--mechanical-config "
                     "参数JSON] [--rx-bundle RX目录]"
                     "\n默认使用固定核心试验台；指定RX目录则使用自由核心与摩擦夹爪。"
                     "棱角块靠卡脚保持；RX模式包含被动磁吸定位近似。"
                     "录像为1倍速，需要EGL和FFmpeg。\n";
        return 0;
      } else
        throw std::invalid_argument("Unsupported mechanical option: " + arg);
    }
    if (!config.empty()) load_settings(config);
    if (record) settings["record_fps"] = 30.;
    const auto requested = split_moves(sequence);
    for (const auto& move : requested) parse_move(move);
    if (value("record_fps") > 60 ||
        (value("record_fps") != 0 &&
         (value("record_fps") < 1 || std::floor(value("record_fps")) != value("record_fps"))))
      throw std::invalid_argument("Recording FPS must be an integer in [1,60], or zero");
    report["parameters"] = settings;
    report["video_playback"] = 1;
    std::filesystem::create_directories(out);
    prepared = true;
    std::ofstream scene_file(out / "scene.xml");
    const bool rx = !rx_bundle.empty();
    auto xml = scene();
    scene_file << (rx ? mechanical_rx_scene(xml, rx_bundle, value("rx_bearing_friction_nm"),
                                            value("rx_tip_friction"))
                      : xml);
    scene_file.close();
    if (!scene_file) throw std::runtime_error("Cannot write mechanical scene");
    Simulation sim(out / "scene.xml");
    auto* m = sim.model;
    auto* d = sim.data;
    report["equalities"] = m->neq;
    report["actuators"] = m->nu;
    report["geoms"] = m->ngeom;
    report["fixed_core_test_rig"] = !rx;
    report["gripper"] = rx ? "rx_friction" : "none";
    report["passive_detent"] = rx ? "torque=-amplitude*sin(4*angle)-0.0005*velocity" : "none";
    if (m->neq != (rx ? 9 : 0))
      throw std::runtime_error("Unexpected mechanical equality constraints");
    if (rx) {
      initialize_rx_grippers(m, d);
      for (auto h : {"A", "B"}) {
        int a = mj_name2id(m, mjOBJ_ACTUATOR, (std::string(h) + "_fingers_actuator").c_str());
        m->actuator_forcerange[2 * a] = -value("rx_jaw_torque_nm");
        m->actuator_forcerange[2 * a + 1] = value("rx_jaw_torque_nm");
      }
    }
    const int core = rx ? mj_name2id(m, mjOBJ_BODY, "mechanical_core") : -1;
    auto core_position = [&]() -> Vec {
      return rx ? Vec(Eigen::Map<const Vec>(d->xpos + 3 * core)) : Vec(0, 0, .1);
    };
    auto core_rotation = [&]() -> Mat {
      return rx ? Mat(Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d->xmat +
                                                                                     9 * core))
                : Mat::Identity();
    };
    std::vector<Vec> slots;
    std::vector<Mat> orientations;
    for (const auto& p : parts) {
      slots.push_back(p.slot);
      orientations.push_back(Mat::Identity());
    }
    std::map<char, double> targets;
    auto measure = [&]() {
      mj_forward(m, d);
      double position = 0, angle = 0;
      Json errors = Json::array();
      for (size_t i = 0; i < parts.size(); ++i) {
        int b = mj_name2id(m, mjOBJ_BODY, parts[i].name.c_str()),
            site = mj_name2id(m, mjOBJ_SITE, (parts[i].name + "_center").c_str());
        position = std::max(
            position, (core_rotation().transpose() *
                           (Eigen::Map<const Vec>(d->site_xpos + 3 * site) - core_position()) -
                       pitch_m * slots[i])
                          .norm());
        Mat r = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d->xmat + 9 * b);
        double piece_angle = std::acos(std::clamp(
            ((orientations[i].transpose() * core_rotation().transpose() * r).trace() - 1) / 2, -1.,
            1.));
        errors.push_back(
            {{"piece", parts[i].name},
             {"orientation_error_rad", piece_angle},
             {"expected", xyz(pitch_m * slots[i])},
             {"actual", xyz(core_rotation().transpose() *
                            (Eigen::Map<const Vec>(d->site_xpos + 3 * site) - core_position()))}});
        angle = std::max(angle, piece_angle);
      }
      report["pieces"] = errors;
      return Json{{"position_error_m", position},
                  {"orientation_error_rad", angle},
                  {"contacts", d->ncon},
                  {"time_s", d->time}};
    };
    Recorder writer;
    if (value("record_fps") > 0) writer.open(out / "demo.mp4", 800, 600, int(value("record_fps")));
    std::unique_ptr<Renderer> renderer;
    auto snapshot = [&](const std::string& name) {
      if (!renderer) renderer = std::make_unique<Renderer>(m);
      mjvCamera cam;
      mjv_defaultCamera(&cam);
      cam.lookat[2] = .1;
      cam.distance = name == "internal" ? .07 : rx ? .4 : .15;
      cam.azimuth = 130;
      cam.elevation = -25;
      mjvOption opt;
      mjv_defaultOption(&opt);
      opt.geomgroup[3] = name == "internal";
      if (name == "internal") {
        opt.geomgroup[0] = 0;
        opt.geomgroup[1] = 0;
      }
      cv::Mat rgb = renderer->render(d, cam, &opt), bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      if (name.starts_with("frame_")) {
        cv::putText(bgr, "Contact-retained mechanical prototype", {15, 28},
                    cv::FONT_HERSHEY_SIMPLEX, .55, {230, 230, 230}, 1, cv::LINE_AA);
      }
      if (name.starts_with("frame_")) {
        writer.write(bgr);
        return;
      }
      if (!cv::imwrite((out / (name + ".png")).string(), bgr))
        throw std::runtime_error("Cannot write image");
    };
    int ticks = 0, frames = 0;
    double peak_tip_normal = 0, peak_tip_tangent = 0, peak_turn_tangent = 0;
    double next_frame = 0;
    std::map<std::string, bool> closed{{"A", false}, {"B", false}};
    auto check_rotation_contacts = [&]() {
      for (int i = 0; i < d->ncon; ++i) {
        const auto& c = d->contact[i];
        auto owner = [&](int g) {
          int body = m->geom_bodyid[g];
          const char* n = mj_id2name(m, mjOBJ_BODY, body);
          std::string b = n ? n : "";
          return b.starts_with("A_")                                 ? 1
                 : b.starts_with("B_")                               ? 2
                 : b.starts_with("piece_") || b == "mechanical_core" ? 3
                                                                     : 0;
        };
        int a = owner(c.geom[0]), b = owner(c.geom[1]);
        bool bad = (a == 1 && b == 2) || (a == 2 && b == 1);
        for (int h = 1; h <= 2; ++h)
          if (!closed[h == 1 ? "A" : "B"] && ((a == h && b == 3) || (b == h && a == 3))) bad = true;
        if (bad) {
          mjtNum f[6];
          mj_contactForce(m, d, i, f);
          if (f[0] > .01) throw std::runtime_error("RX forbidden rotation contact");
        }
      }
    };
    auto advance = [&](double t, bool check_contacts = false) {
      for (int k = 0; k < std::lround(t / m->opt.timestep); ++k) {
        if (rx) {
          // 固定周期势能的负梯度，模拟被动磁吸定位；不读取规划目标。
          for (char f : faces) {
            int j = mj_name2id(m, mjOBJ_JOINT, ("hinge_" + std::string(1, f)).c_str());
            int v = m->jnt_dofadr[j];
            d->qfrc_applied[v] =
                -value("rx_detent_torque_nm") * std::sin(4 * d->qpos[m->jnt_qposadr[j]]) -
                .0005 * d->qvel[v];
          }
        }
        mj_step(m, d);
        if (check_contacts) check_rotation_contacts();
        ++ticks;
        if (rx && ticks % 10 == 0) {
          for (int i = 0; i < d->ncon; ++i) {
            const auto& contact = d->contact[i];
            const char* a = mj_id2name(m, mjOBJ_GEOM, contact.geom[0]);
            const char* b = mj_id2name(m, mjOBJ_GEOM, contact.geom[1]);
            if (!a || !b) continue;
            auto tip = [](const char* n) { return std::string_view(n).ends_with("_tip"); };
            auto shell = [](const char* n) { return std::string_view(n).ends_with("_shell"); };
            if (!((tip(a) && shell(b)) || (tip(b) && shell(a)))) continue;
            mjtNum f[6];
            mj_contactForce(m, d, i, f);
            peak_tip_normal = std::max(peak_tip_normal, f[0]);
            peak_tip_tangent = std::max(peak_tip_tangent, std::hypot(f[1], f[2]));
            if (report.contains("current_action") &&
                report["current_action"].value("kind", "") == "yaw")
              peak_turn_tangent = std::max(peak_turn_tangent, std::hypot(f[1], f[2]));
          }
          report["peak_tip_contact_normal_n"] = peak_tip_normal;
          report["peak_tip_contact_tangent_n"] = peak_tip_tangent;
          report["peak_turn_tip_contact_tangent_n"] = peak_turn_tangent;
        }
        if (rx && !d->eq_active[mj_name2id(m, mjOBJ_EQUALITY, "mechanical_loading_fixture")] &&
            (core_position() - Vec(0, 0, .1)).norm() > .005) {
          report["slip_state"] = measure();
          report["slip_state"]["core_position"] = xyz(core_position());
          snapshot("slip");
          throw std::runtime_error("RX lost cube support");
        }
        if (value("record_fps") > 0 && d->time + 1e-9 >= next_frame) {
          mj_forward(m, d);
          std::ostringstream name;
          name << "frame_" << std::setfill('0') << std::setw(6) << frames++;
          snapshot(name.str());
          next_frame += 1 / value("record_fps");
        }
        for (int w = 0; w < mjNWARNING; ++w)
          if (d->warning[w].number) throw std::runtime_error("MuJoCo numerical warning");
        if (ticks % 100 == 0)
          for (const auto& p : parts) {
            int site = mj_name2id(m, mjOBJ_SITE, (p.name + "_center").c_str());
            double r = (core_rotation().transpose() *
                        (Eigen::Map<const Vec>(d->site_xpos + 3 * site) - core_position()))
                           .norm();
            if (std::abs(r - pitch_m * p.slot.norm()) > .005) {
              report["escape_piece"] = p.name;
              report["escape_state"] = measure();
              snapshot("escape");
              throw std::runtime_error("Piece escaped radial retention");
            }
          }
        if (ticks % 1000 == 0)
          std::cout << "time " << d->time << " contacts " << d->ncon << std::endl;
      }
    };
    mj_forward(m, d);
    report["initial"] = measure();
    snapshot("initial");
    snapshot("internal");
    advance(.3);
    report["settled"] = measure();
    std::cout << "settled " << report["settled"] << std::endl;
    if (report["settled"]["position_error_m"].get<double>() > .0005)
      throw std::runtime_error("Assembly lost retention at rest");
    Mat robot_orientation = Mat::Identity();
    auto tip_contacts = [&](const std::string& hand) {
      Json pads = Json::object();
      for (auto side : {"left", "right"}) {
        int tip = mj_name2id(m, mjOBJ_GEOM, (hand + "_" + side + "_tip").c_str());
        double normal = 0;
        for (int i = 0; i < d->ncon; ++i) {
          const auto& c = d->contact[i];
          int other = c.geom[0] == tip ? c.geom[1] : c.geom[1] == tip ? c.geom[0] : -1;
          if (other < 0) continue;
          const char* name = mj_id2name(m, mjOBJ_GEOM, other);
          if (!name || !std::string(name).ends_with("_shell")) continue;
          mjtNum force[6];
          mj_contactForce(m, d, i, force);
          normal += force[0];
        }
        pads[side] = normal;
      }
      return pads;
    };
    auto grip_ready = [&](const std::string& h) {
      auto p = tip_contacts(h);
      return p["left"].get<double>() > .01 && p["right"].get<double>() > .01;
    };
    auto gap = [&](const std::string& h) {
      int a = mj_name2id(m, mjOBJ_GEOM, (h + "_left_tip").c_str()),
          b = mj_name2id(m, mjOBJ_GEOM, (h + "_right_tip").c_str());
      Vec delta = Eigen::Map<const Vec>(d->geom_xpos + 3 * a) -
                  Eigen::Map<const Vec>(d->geom_xpos + 3 * b),
          dir = delta.normalized();
      double radius = 0;
      for (int g : {a, b}) {
        Mat r =
            Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d->geom_xmat + 9 * g);
        radius += (r.transpose() * dir).cwiseAbs().dot(Eigen::Map<const Vec>(m->geom_size + 3 * g));
      }
      return delta.norm() - radius;
    };
    auto command = [&](const std::string& name, double target, double duration) {
      int a = mj_name2id(m, mjOBJ_ACTUATOR, name.c_str());
      if (a < 0) throw std::runtime_error("Missing RX actuator: " + name);
      double start = d->ctrl[a];
      bool turning = name.ends_with("_yaw_drive");
      int count = std::max(1L, std::lround(duration / m->opt.timestep));
      for (int k = 1; k <= count; ++k) {
        double u = double(k) / count;
        d->ctrl[a] = start + (target - start) * u * u * u * (10 + u * (-15 + 6 * u));
        advance(m->opt.timestep, turning);
      }
      advance(.15, turning);
    };
    auto jaw = [&](const std::string& h, bool close) {
      command(h + "_fingers_actuator", close ? -1.20 : -2.25, .5);
      report["jaw_checks"].push_back({{"hand", h},
                                      {"closed", close},
                                      {"gap_m", gap(h)},
                                      {"forces_n", tip_contacts(h)},
                                      {"time_s", d->time}});
      closed[h] = close;
      if (close ? !grip_ready(h) : gap(h) < .055 * std::sqrt(2.) + .001)
        throw std::runtime_error("RX jaw contact/clearance failed: " + h);
    };
    auto update_expected = [&](const std::string& move) {
      auto [f, c] = parse_move(move);
      auto [axis, sign] = face_axis(f);
      Mat rot = rotation(axis, -sign * c * pi / 2).array().round();
      for (size_t i = 0; i < parts.size(); ++i)
        if (slots[i][axis] == sign) {
          slots[i] = rot * slots[i];
          orientations[i] = rot * orientations[i];
        }
    };
    auto check_pose = [&](const std::string& label, bool checkpoint = true) {
      auto read_pose = [&]() {
        auto state = measure();
        state["core_position"] = xyz(core_position());
        state["core_orientation_error_rad"] = std::acos(std::clamp(
            ((robot_orientation.transpose() * core_rotation()).trace() - 1) / 2, -1., 1.));
        return state;
      };
      auto aligned = [&](const Json& state) {
        return state["position_error_m"].get<double>() <= .0005 &&
               state["orientation_error_rad"].get<double>() <= .02 &&
               state["core_orientation_error_rad"].get<double>() <= .02 &&
               (core_position() - Vec(0, 0, .1)).norm() <= .001;
      };
      auto state = read_pose();
      bool timed_out = false;
      if (checkpoint && !aligned(state) && value("alignment_timeout_s") > 0) {
        Json wait = {{"move", label}, {"initial", state}, {"samples", Json::array()}};
        std::cout << "waiting for alignment: " << label << ' ' << state << std::endl;
        double started = d->time;
        int stable = 0;
        while (d->time - started + .05 <= value("alignment_timeout_s") + 1e-8 && stable < 3) {
          advance(.05, true);
          state = read_pose();
          wait["samples"].push_back(state);
          stable = aligned(state) ? stable + 1 : 0;
        }
        wait["duration_s"] = d->time - started;
        wait["passed"] = stable == 3;
        report["alignment_waits"].push_back(wait);
        timed_out = stable < 3;
      }
      state["move"] = label;
      double core_angle = state["core_orientation_error_rad"];
      report["steps"].push_back(state);
      std::cout << state << std::endl;
      snapshot("step_" + std::to_string(report["steps"].size()));
      // 单手换向后还未重新夹紧；记录瞬态偏差，转层前仍执行完整验收。
      if (!checkpoint) return;
      if (timed_out) throw std::runtime_error("RX alignment timeout: " + label);
      if (state["position_error_m"].get<double>() > .0005 ||
          state["orientation_error_rad"].get<double>() > .02)
        throw std::runtime_error("RX mechanical pose mismatch: " + label);
      if ((core_position() - Vec(0, 0, .1)).norm() > .001 || core_angle > .02)
        throw std::runtime_error("RX core slipped: " + label);
      for (auto h : {"A", "B"})
        if (closed[h] && !grip_ready(h))
          throw std::runtime_error("RX grip lost: " + std::string(h));
    };
    if (rx) {
      jaw("A", true);
      jaw("B", true);
      d->eq_active[mj_name2id(m, mjOBJ_EQUALITY, "mechanical_loading_fixture")] = 0;
      report["loading_fixture_releases"] = 1;
      advance(.3);
      check_pose("grasp");
    }
    auto execute_moves = [&](const std::vector<std::string>& moves) {
      if (rx) {
        auto plan = compile_moves(moves, robot_orientation, true);
        for (const auto& a : plan) {
          report["current_action"] = a.json();
          if (a.kind == "jaw")
            jaw(a.hand, a.target != 0);
          else if (a.kind == "grasp" && !grip_ready(a.hand))
            throw std::runtime_error("RX lost fingertip contact");
          else if (a.kind == "yaw") {
            auto states = out / "diagnostic_states";
            std::filesystem::create_directories(states);
            auto save_state = [&](const std::string& prefix) {
              std::vector<mjtNum> state(mj_stateSize(m, mjSTATE_INTEGRATION));
              mj_getState(m, d, state.data(), mjSTATE_INTEGRATION);
              std::ofstream saved(states /
                                  (prefix + std::to_string(report["actions"].size()) + ".json"));
              saved << Json{{"mode", "diagnostic_snapshot"},
                            {"state_spec", int(mjSTATE_INTEGRATION)},
                            {"state", state},
                            {"parameters", settings},
                            {"action", a.json()},
                            {"robot_orientation", matrix_json(robot_orientation)}}
                           .dump();
              saved.close();
              if (!saved) throw std::runtime_error("Cannot write diagnostic state");
            };
            save_state("before_");
            double checked_at = d->time;
            if (a.mode == "face") check_pose("before " + a.move);
            if (d->time > checked_at) save_state("aligned_");
            int actuator = mj_name2id(m, mjOBJ_ACTUATOR, (a.hand + "_yaw_drive").c_str());
            double start = d->ctrl[actuator];
            command(a.hand + "_yaw_drive", a.target, value("turn_duration_s"));
            int joint = mj_name2id(m, mjOBJ_JOINT, (a.hand + "_yaw").c_str());
            if (std::abs(d->qpos[m->jnt_qposadr[joint]] - a.target) > .006)
              throw std::runtime_error("RX wrist tracking failed");
            if (a.mode == "whole") {
              robot_orientation = wrist_rotation(a.hand, a.target - start) * robot_orientation;
              check_pose("whole " + a.hand, false);
            }
          } else if (a.kind == "layer_lock") {
            update_expected(a.move);
            check_pose(a.move);
          } else if (a.kind == "checkpoint")
            check_pose(a.move + " checkpoint");
          report["actions"].push_back(a.json());
        }
        return;
      }
      for (const auto& move : moves) {
        report["current_move"] = move;
        auto [f, c] = parse_move(move);
        auto [axis, sign] = face_axis(f);
        int a = mj_name2id(m, mjOBJ_ACTUATOR, ("drive_" + std::string(1, f)).c_str());
        const double start = targets[f], end = start - c * pi / 2;
        int count = std::max(1L, std::lround(value("turn_duration_s") / m->opt.timestep));
        for (int k = 1; k <= count; ++k) {
          double u = double(k) / count;
          d->ctrl[a] = start + (end - start) * u * u * u * (10 + u * (-15 + 6 * u));
          advance(m->opt.timestep);
        }
        targets[f] = end;
        d->ctrl[a] = end;
        advance(.3);
        Mat rot = rotation(axis, -sign * c * pi / 2).array().round();
        for (size_t i = 0; i < parts.size(); ++i)
          if (slots[i][axis] == sign) {
            slots[i] = rot * slots[i];
            orientations[i] = rot * orientations[i];
          }
        auto state = measure();
        state["move"] = move;
        report["steps"].push_back(state);
        std::cout << state << std::endl;
        snapshot("step_" + std::to_string(report["steps"].size()));
        if (state["position_error_m"].get<double>() > .0005 ||
            state["orientation_error_rad"].get<double>() > .02)
          throw std::runtime_error("Contact-driven layer failed physical alignment");
      }
    };
    auto physical_facelets = [&]() {
      mj_forward(m, d);
      std::vector<Vec> initial, physical_slots;
      std::vector<Mat> physical_rotations;
      for (const auto& p : parts) {
        if (p.slot.squaredNorm() < 2) continue;
        int b = mj_name2id(m, mjOBJ_BODY, p.name.c_str()),
            site = mj_name2id(m, mjOBJ_SITE, (p.name + "_center").c_str());
        Vec position = (core_rotation().transpose() *
                        (Eigen::Map<const Vec>(d->site_xpos + 3 * site) - core_position())) /
                       pitch_m;
        Vec rounded = position.array().round();
        Mat rotation =
                core_rotation().transpose() *
                Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d->xmat + 9 * b),
            rounded_rotation = rotation.array().round();
        if ((position - rounded).cwiseAbs().maxCoeff() > .02 ||
            (rotation - rounded_rotation).cwiseAbs().maxCoeff() > .02)
          throw std::runtime_error("Physical pieces not aligned for facelet reading");
        initial.push_back(p.slot);
        physical_slots.push_back(rounded);
        physical_rotations.push_back(rounded_rotation);
      }
      return encode_facelets(initial, physical_slots, physical_rotations);
    };
    report["scramble"] = requested;
    execute_moves(requested);
    report["scrambled_facelets"] = physical_facelets();
    if (solve) {
      auto solution = solve_facelets(report["scrambled_facelets"].get<std::string>(), root);
      report["solution"] = solution;
      execute_moves(solution);
    }
    if (value("record_fps") > 0) advance(.5, rx);
    if (rx) check_pose("final");
    report["final_facelets"] = physical_facelets();
    report["solved"] = report["final_facelets"] == solved;
    if (solve && !report["solved"].get<bool>())
      throw std::runtime_error("Mechanical cube did not finish solved");
    report["final_pose"] = measure();
    if (report["final_pose"]["position_error_m"].get<double>() > .0005 ||
        report["final_pose"]["orientation_error_rad"].get<double>() > .02)
      throw std::runtime_error("Mechanical cube drifted after execution");
    writer.release();
    report["frames"] = frames;
    snapshot("final");
    report["passed"] = true;
  } catch (const std::exception& e) {
    report["passed"] = false;
    report["error"] = e.what();
    std::cerr << e.what() << std::endl;
    if (!prepared) return 1;
  }
  report["wall_time_s"] =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
  std::ofstream result(out / "report.json");
  result << report.dump(2) << '\n';
  result.flush();
  if (!result) {
    std::cerr << "Cannot write mechanical report\n";
    return 1;
  }
  return report["passed"].get<bool>() ? 0 : 1;
}
