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
#include "rm/cube_magnets.hpp"
#include "rm/mechanical_plan.hpp"
#include "rm/mechanical_sdf.hpp"
#include "rm/rx_gripper.hpp"
#include "rm/rx_wrist_tn.hpp"
#include "rm/wrist_motion.hpp"
#include "rx_full_body.hpp"
#include "video.hpp"

namespace {
using namespace rm;
using namespace rm::cube;
constexpr double pitch_m = .055 / 3;
Json settings = {{"cube_mass_kg", .088},
                 {"shell_gap_m", .00005},
                 {"shell_bevel_m", .00065},
                 {"cap_inner_m", {.01460, .01265, .01085}},
                 {"cap_outer_m", {.0162, .0143, .0125}},
                 {"cap_angle_deg", {43., 21.5, 31.}},
                 {"cap_merge_quads", 0},
                 {"cap_sectors", 16},
                 {"cap_rings", {3, 3, 3}},
                 {"cap_collision_mode", 0},
                 {"retention_mode", 0},
                 {"ball_damping_nm_s", .00002},
                 {"cube_magnet_peak_force_n", 0.},
                 {"bearing_collision_mode", 0},
                 {"preload_stiffness_n_m", 500.},
                 {"preload_reference_m", -.0015},
                 {"turn_duration_s", 1.5},
                 {"timestep_s", .0005},
                 {"internal_contact_impedance", .9999},
                 {"internal_contact_dimension", 3},
                 {"friction_impedance_ratio", 10.},
                 {"friction_noslip_iterations", 0},
                 {"alignment_timeout_s", 2.},
                 {"alignment_regrasp_attempts", 2},
                 {"robot_search_ms", 1000.},
                 {"rx_alignment_position_m", .0005},
                 {"rx_alignment_angle_rad", .02},
                 {"rx_core_position_m", .001},
                 {"max_torque_nm", .03},
                 {"rx_jaw_torque_nm", .3},
                 {"rx_wrist_tn", 0},
                 {"rx_full_body_centering_gain_s", 0.},
                 {"rx_full_body_realign_open", 0},
                 {"rx_wrist_speed_rad_s", 0.},
                 {"rx_wrist_accel_rad_s2", 80.},
                 {"rx_wrist_empty_multiplier", 1.5},
                 {"rx_wrist_whole_multiplier", 1.},
                 {"rx_wrist_settle_s", .15},
                 {"rx_jaw_duration_s", .5},
                 {"rx_jaw_settle_s", .15},
                 {"rx_wrist_kp", 20.},
                 {"rx_wrist_kv", 1.},
                 {"rx_wrist_zero_jerk", 0},
                 {"rx_wrist_cruise_fraction", 0.},
                 {"rx_wrist_brake_half_turn", 0},
                 {"rx_tip_friction", 1.2},
                 {"tip_torsional_friction_m", .002},
                 {"rx_tip_corner_spheres", 0},
                 {"rx_tip_sphere_radius_m", .003},
                 {"rx_tip_sphere_offset_m", .005},
                 {"rx_tip_contact_time_s", .004},
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
    if (key == "alignment_regrasp_attempts" &&
        (!v.is_number_integer() || v.get<double>() < 0 || v.get<double>() > 2))
      throw std::invalid_argument("alignment_regrasp_attempts must be an integer in [0,2]");
    if (key == "friction_noslip_iterations" &&
        (!v.is_number_integer() || v.get<double>() < 0 || v.get<double>() > 100))
      throw std::invalid_argument("friction_noslip_iterations must be an integer in [0,100]");
    if (key == "cap_merge_quads" &&
        (!v.is_number_integer() || (v.get<double>() != 0 && v.get<double>() != 1)))
      throw std::invalid_argument("cap_merge_quads must be zero or one");
    if (key == "cap_sectors" && (!v.is_number_integer() || v.get<double>() < 8 ||
                                 v.get<double>() > 32 || std::fmod(v.get<double>(), 4) != 0))
      throw std::invalid_argument("cap_sectors must be a multiple of four in [8,32]");
    if (key == "cap_rings")
      for (const auto& item : v)
        if (!item.is_number_integer() || item.get<double>() < 1 || item.get<double>() > 4)
          throw std::invalid_argument("cap_rings entries must be integers in [1,4]");
    if (key == "rx_wrist_cruise_fraction" && v.get<double>() > .9)
      throw std::invalid_argument("rx_wrist_cruise_fraction must be in [0,0.9]");
    if (key == "rx_full_body_realign_open" && v.get<double>() != 0 && v.get<double>() != 1)
      throw std::invalid_argument("rx_full_body_realign_open must be 0 or 1");
    if (key == "rx_wrist_tn" && v.get<double>() != 0 && v.get<double>() != 1)
      throw std::invalid_argument("rx_wrist_tn must be 0 or 1");
    if (key == "rx_tip_corner_spheres" && v.get<double>() != 0 && v.get<double>() != 1)
      throw std::invalid_argument("rx_tip_corner_spheres must be 0 or 1");
    if ((key == "rx_wrist_zero_jerk" || key == "rx_wrist_brake_half_turn") &&
        v.get<double>() != 0 && v.get<double>() != 1)
      throw std::invalid_argument(key + " must be 0 or 1");
    if (key == "retention_mode" && (v.get<double>() != 0 && v.get<double>() != 1))
      throw std::invalid_argument("retention_mode must be 0 or 1");
    if (key == "cap_collision_mode" &&
        (!v.is_number_integer() || v.get<double>() < 0 || v.get<double>() > 2))
      throw std::invalid_argument("cap_collision_mode must be an integer in [0,2]");
    if (key == "bearing_collision_mode" &&
        (!v.is_number_integer() || v.get<double>() < 0 || v.get<double>() > 2))
      throw std::invalid_argument("bearing_collision_mode must be an integer in [0,2]");
    if (key == "internal_contact_dimension" &&
        (!v.is_number_integer() || (v.get<double>() != 1 && v.get<double>() != 3)))
      throw std::invalid_argument("internal_contact_dimension must be one or three");
    settings[key] = v;
  }
  if (value("cube_mass_kg") < .01 || value("cube_mass_kg") > 1 ||
      value("rx_wrist_accel_rad_s2") <= 0 || value("rx_wrist_empty_multiplier") < 1 ||
      value("rx_wrist_empty_multiplier") > 4 || value("rx_wrist_whole_multiplier") <= 0 ||
      value("rx_wrist_whole_multiplier") > 1 || value("rx_wrist_kp") <= 0 ||
      value("rx_wrist_settle_s") > 2 || value("rx_wrist_speed_rad_s") > 30 ||
      value("rx_jaw_duration_s") <= 0 || value("rx_jaw_duration_s") > 2 ||
      value("rx_jaw_settle_s") > 2)
    throw std::invalid_argument("Invalid cube mass or wrist motion settings");
  if (value("turn_duration_s") <= 0 || value("timestep_s") <= 0 || value("timestep_s") > .001 ||
      value("max_torque_nm") <= 0)
    throw std::invalid_argument("Invalid timing or torque");
  if (value("internal_contact_impedance") <= 0 || value("internal_contact_impedance") >= 1)
    throw std::invalid_argument("Contact impedance must be between zero and one");
  if (value("tip_torsional_friction_m") > .01)
    throw std::invalid_argument("Tip torsional friction must be in [0,.01] m");
  if (value("rx_tip_contact_time_s") < .001 || value("rx_tip_contact_time_s") > .1)
    throw std::invalid_argument("RX tip contact time must be in [.001,.1] seconds");
  if (value("rx_tip_sphere_radius_m") < .0005 || value("rx_tip_sphere_radius_m") > .003 ||
      value("rx_tip_sphere_offset_m") < value("rx_tip_sphere_radius_m") ||
      value("rx_tip_sphere_offset_m") + value("rx_tip_sphere_radius_m") > .008 + 1e-12)
    throw std::invalid_argument("RX corner spheres must fit the pad and not overlap");
  if (value("cube_magnet_peak_force_n") > 2)
    throw std::invalid_argument("Cube magnet force must be in [0,2] N");
  if (value("ball_damping_nm_s") > .01)
    throw std::invalid_argument("Ball damping must be in [0,.01] Nm s/rad");
  if (value("friction_impedance_ratio") < 1 || value("friction_impedance_ratio") > 1000)
    throw std::invalid_argument("Friction impedance ratio must be in [1,1000]");
  if (value("robot_search_ms") > 60000)
    throw std::invalid_argument("robot_search_ms must be in [0,60000]");
  if (value("rx_alignment_position_m") <= 0 || value("rx_alignment_position_m") >= .005 ||
      value("rx_alignment_angle_rad") <= 0 || value("rx_alignment_angle_rad") >= pi / 4 ||
      value("rx_core_position_m") <= 0 || value("rx_core_position_m") >= .005)
    throw std::invalid_argument("Invalid RX alignment tolerance");
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
  const bool guided = value("retention_mode") == 1;
  const bool torpedo = value("bearing_collision_mode") == 2;
  const double core_radius = torpedo ? .009 : .0107;
  std::ostringstream assets, bodies, motors, plugins;
  bodies << std::setprecision(17);
  std::array<std::vector<std::string>, 3> cap_meshes;
  std::array<std::vector<std::pair<Vec, Vec>>, 3> cap_rims, cap_spokes;
  // 三种卡脚处在不同球面半径。每个小楔体都是凸体，合起来保留内侧空腔。
  const auto inner = settings["cap_inner_m"].get<std::array<double, 3>>();
  const auto outer = settings["cap_outer_m"].get<std::array<double, 3>>();
  auto angle = settings["cap_angle_deg"].get<std::array<double, 3>>();
  for (auto& a : angle) a *= pi / 180;
  const int sectors = settings["cap_sectors"].get<int>();
  for (int type = 0; !guided && type < 3; ++type) {
    const int rings = settings["cap_rings"][type].get<int>();
    if (!guided && value("cap_collision_mode") == 2) {
      const std::string name = "cap_sdf_" + std::to_string(type);
      plugins << "<instance name='" << name << "'><config key='inner' value='" << inner[type]
              << "'/><config key='outer' value='" << outer[type]
              << "'/><config key='angle_deg' value='" << angle[type] * 180 / pi
              << "'/><config key='notch' value='" << (type == 0 ? 1 : 0) << "'/></instance>";
      assets << "<mesh name='" << name << "'><plugin instance='" << name << "'/></mesh>";
      continue;
    }
    if (!guided && value("cap_collision_mode") == 1) {
      const double mid = (inner[type] + outer[type]) / 2;
      const double tube = (outer[type] - inner[type]) / 2;
      auto rim = [&](double phi, double fraction = 1.) -> Vec {
        const double limit =
            type == 0 ? std::min(angle[type], std::asin(.53 / std::max(std::abs(std::cos(phi)),
                                                                       std::abs(std::sin(phi)))))
                      : angle[type];
        const double theta = std::max(.001, limit - std::asin(tube / mid)) * fraction;
        return mid * Vec(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi),
                         std::cos(theta));
      };
      for (int s = 0; s < sectors; ++s)
        cap_rims[type].emplace_back(rim(2 * pi * s / sectors), rim(2 * pi * (s + 1) / sectors));
      // 细辐条连接颈部和承托环；分成两段，避免直弦侵入球芯。
      for (int s = 0; s < 4; ++s) {
        const double phi = pi / 4 + s * pi / 2;
        cap_spokes[type].emplace_back(rim(phi, 0), rim(phi, .5));
        cap_spokes[type].emplace_back(rim(phi, .5), rim(phi));
      }
      auto clear_core = [&](const auto& segments, double radius) {
        for (const auto& [a, b] : segments) {
          const Vec delta = b - a;
          const double t = std::clamp(-a.dot(delta) / delta.squaredNorm(), 0., 1.);
          if ((a + t * delta).norm() - radius < core_radius)
            throw std::invalid_argument("Capsule cap intersects spherical core");
        }
      };
      clear_core(cap_rims[type], tube);
      clear_core(cap_spokes[type], .00035);
      continue;
    }
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
    auto wedge = [&](const std::vector<Vec>& points) {
      std::string name =
          "cap_" + std::to_string(type) + "_" + std::to_string(cap_meshes[type].size());
      std::vector<Vec> vertices;
      for (double radius : {inner[type], outer[type]})
        for (const auto& p : points) vertices.push_back(radius * p);
      mesh(assets, name, vertices);
      cap_meshes[type].push_back(name);
    };
    for (int r = 0; r < rings; ++r)
      for (int s = 0; s < sectors; ++s) {
        const Vec a = point(r, s), b = point(r + 1, s), c = point(r + 1, s + 1),
                  d = point(r, s + 1);
        // 同平面四边形的两个楔体可合并，非共面单元保留原三角分割，避免填平内腔。
        const Vec normal = (b - a).cross(c - a).normalized();
        if (r && value("cap_merge_quads") != 0 && std::abs(normal.dot(d - a)) < 1e-12)
          wedge({a, b, c, d});
        else {
          wedge({a, b, c});
          if (r) wedge({a, c, d});
        }
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
          if (!guided)
            bodies << "<joint name='preload_" << face << "' type='slide' axis='" << xyz(n)
                   << "' stiffness='" << value("preload_stiffness_n_m") << "' springref='"
                   << value("preload_reference_m")
                   << "' damping='.2' limited='true' "
                      "range='-.0015 .0015'/>";
          motors << "<position name='drive_" << face << "' joint='hinge_" << face
                 << "' kp='.3' kv='.003' forcerange='" << -value("max_torque_nm") << " "
                 << value("max_torque_nm") << "'/>";
        } else if (guided)
          bodies << "<joint name='guide_" << name << "' type='ball' damping='"
                 << value("ball_damping_nm_s") << "' armature='.00000001'/>";
        else
          bodies << "<freejoint/>";
        // 棱角接缝两侧各埋入1.5毫米；位置随块运动，不依赖当前逻辑排列。
        if (nonzero >= 2) {
          auto magnet_site = [&](int axis, int sign) {
            Vec p = c;
            p[axis] += sign * (pitch_m / 2 - .0015);
            bodies << "<site name='cube_" << (nonzero == 3 ? "corner" : "edge") << "_magnet_"
                   << name << "_" << axis << "_" << (sign > 0 ? "p" : "n") << "' pos='" << xyz(p)
                   << "' size='.001' rgba='1 .7 .1 1' group='3'/>";
          };
          for (int axis = 0; axis < 3; ++axis) {
            if (nonzero == 3)
              magnet_site(axis, -static_cast<int>(slot[axis]));
            else if (slot[axis] == 0) {
              magnet_site(axis, -1);
              magnet_site(axis, 1);
            }
          }
        }
        bodies << "<inertial pos='" << xyz(c) << "' mass='" << .003 * value("cube_mass_kg") / .088
               << "' diaginertia='" << xyz(Vec::Constant(.00000016 * value("cube_mass_kg") / .088))
               << "'/>";
        bodies << "<geom name='" << name << "_shell' type='mesh' mesh='" << name
               << "_shell' rgba='.08 .08 .08 1'/>";
        for (const auto& cap : cap_meshes[nonzero - 1])
          bodies << "<geom type='mesh' mesh='" << cap << "' quat='" << quaternion(n) << "' rgba='"
                 << (nonzero == 1   ? ".2 .55 .9 1"
                     : nonzero == 2 ? ".15 .75 .45 1"
                                    : "1 .5 .12 1")
                 << "' group='3'/>";
        if (!guided && value("cap_collision_mode") == 1) {
          const auto rotation = Eigen::Quaterniond::FromTwoVectors(Vec::UnitZ(), n);
          auto emit = [&](const auto& segments, double radius) {
            for (const auto& [a, b] : segments)
              bodies << "<geom type='capsule' fromto='" << xyz(rotation * a) << " "
                     << xyz(rotation * b) << "' size='" << radius
                     << "' rgba='.2 .7 .5 1' group='3'/>";
          };
          emit(cap_rims[nonzero - 1], (outer[nonzero - 1] - inner[nonzero - 1]) / 2);
          emit(cap_spokes[nonzero - 1], .00035);
        }
        if (!guided && value("cap_collision_mode") == 2) {
          const std::string cap = "cap_sdf_" + std::to_string(nonzero - 1);
          bodies << "<geom type='sdf' mesh='" << cap << "' quat='" << quaternion(n)
                 << "' rgba='.3 .6 .8 1' group='3'><plugin instance='" << cap << "'/></geom>";
        }
        if (!guided && nonzero == 2) {
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
          if (torpedo) {
            // 两侧凸缘进入角足下方，横梁与颈部连接。原球芯缩小以保留内腔。
            auto rail = [&](double a, double b) -> Vec { return .0098 * surface(a, b); };
            auto capsule = [&](const Vec& p, const Vec& q, double radius) {
              const Vec delta = q - p;
              const double t = std::clamp(-p.dot(delta) / delta.squaredNorm(), 0., 1.);
              if ((p + t * delta).norm() - radius < core_radius)
                throw std::invalid_argument("Torpedo bearing intersects spherical core");
              bodies << "<geom type='capsule' fromto='" << xyz(p) << " " << xyz(q) << "' size='"
                     << radius << "' rgba='.8 .5 .2 1' group='3'/>";
            };
            for (double b : {-8., 8.})
              for (int a = -20; a < 20; a += 5) capsule(rail(a, b), rail(a + 5, b), .00065);
            for (double a : {-20., 20.}) capsule(rail(a, -8), rail(a, 8), .00065);
            capsule(rail(0, -8), rail(0, 8), .00035);
          }
          for (int i = 0; !torpedo && i < 8; ++i) {
            const double a = -20 + 5 * i, b = a + 5;
            if (value("bearing_collision_mode") == 1) {
              // 用原凸片两端截面的中心作轴线，圆截面消除薄片侧棱的楔入。
              // 横截面收窄，半球端帽越过原端面；不能视为与原薄片完全等价。
              const Vec p = .011275 * (surface(a, -2.5) + surface(a, 2.5)) / 2;
              const Vec q = .011275 * (surface(b, -2.5) + surface(b, 2.5)) / 2;
              bodies << "<geom type='capsule' fromto='" << xyz(p) << " " << xyz(q)
                     << "' size='.000425' rgba='.8 .5 .2 1' group='3'/>";
              continue;
            }
            std::vector<Vec> vertices;
            for (double r : {.01085, .0117})
              for (auto v : {surface(a, -2.5), surface(b, -2.5), surface(b, 2.5), surface(a, 2.5)})
                vertices.push_back(r * v);
            auto bearing = name + "_bearing_" + std::to_string(i);
            mesh(assets, bearing, vertices);
            bodies << "<geom type='mesh' mesh='" << bearing << "' rgba='.8 .5 .2 1' group='3'/>";
          }
        }
        if (!guided && nonzero > 1)
          bodies << "<geom type='capsule' fromto='"
                 << xyz(n * (torpedo && nonzero == 2 ? .00965 : .0112)) << " " << xyz(n * .018)
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
  s << "<mujoco model='"
    << (guided ? "Joint guided cube prototype" : "Contact retained cube prototype") << "'>";
  if (!guided && value("cap_collision_mode") == 2)
    s << "<extension><plugin plugin='rm.cube.cap'>" << plugins.str() << "</plugin></extension>";
  s << "<compiler angle='radian'/><option "
       "timestep='"
    << value("timestep_s")
    << "' gravity='0 0 -9.81' integrator='implicitfast' solver='Newton' "
       "iterations='100' cone='elliptic' impratio='"
    << value("friction_impedance_ratio") << "' noslip_iterations='"
    << settings["friction_noslip_iterations"].get<int>()
    << "'/><size memory='256M'/><default><geom "
       "friction='.03 .0001 .00001' solref='.001 1' solimp='"
    << value("internal_contact_impedance") << ' ' << value("internal_contact_impedance")
    << " .0001' condim='" << settings["internal_contact_dimension"].get<int>()
    << "'/></default><visual><global offwidth='800' offheight='600'/></visual><asset>"
    << assets.str()
    << "</asset><worldbody><light pos='.1 -.2 .5'/><geom type='sphere' pos='0 0 .1' size='"
    << core_radius << (guided ? "' contype='0' conaffinity='0" : "")
    << "' rgba='.5 .3 .2 1' group='3'/>" << bodies.str() << "</worldbody><actuator>" << motors.str()
    << "</actuator></mujoco>";
  return s.str();
}
}  // namespace
int rm::cube::mechanical_main(const std::filesystem::path& root, int argc, char** argv) {
  const auto began = std::chrono::steady_clock::now();
  Json report = {{"passed", false},
                 {"model", "experimental_contact_retention"},
                 {"internal_constraint_switches", 0},
                 {"steps", Json::array()}};
  std::filesystem::path out = root / "output/cube-mechanical/current", config, rx_bundle,
                        full_body_bundle;
  std::string sequence = "R U F";
  bool solve = false, record = false, prepared = false, plan_only = false;
  std::optional<double> search_ms_override;
  try {
    register_mechanical_sdf();
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
      else if (arg == "--rx-full-body")
        full_body_bundle = next();
      else if (arg == "--solve")
        solve = true;
      else if (arg == "--record")
        record = true;
      else if (arg == "--plan-only")
        plan_only = true;
      else if (arg == "--search-ms") {
        const auto text = next();
        size_t used = 0;
        search_ms_override = std::stod(text, &used);
        if (used != text.size()) throw std::invalid_argument("Invalid search budget");
      } else if (arg == "--help" || arg == "-h") {
        std::cout << "机械魔方原型：./rm cube --mechanical [--scramble \"R U F\"] [--solve] "
                     "[--record] [--output 输出目录] [--mechanical-config "
                     "参数JSON] [--rx-bundle RX目录] [--rx-full-body 整机目录] [--search-ms 毫秒] "
                     "[--plan-only]"
                     "\n默认使用固定核心试验台；指定RX目录则使用自由核心与摩擦夹爪。"
                     "棱角块靠卡脚保持；RX模式包含被动磁吸定位近似。"
                     "录像为1倍速，需要EGL和FFmpeg。\n";
        return 0;
      } else
        throw std::invalid_argument("Unsupported mechanical option: " + arg);
    }
    if (!config.empty()) load_settings(config);
    if (!full_body_bundle.empty() && (rx_bundle.empty() || value("rx_wrist_speed_rad_s") <= 0))
      throw std::invalid_argument(
          "RX整机需要 --rx-bundle，且 rx_wrist_speed_rad_s 必须大于0；请使用整机配置");
    if (search_ms_override) settings["robot_search_ms"] = *search_ms_override;
    if (!std::isfinite(value("robot_search_ms")) || value("robot_search_ms") < 0 ||
        value("robot_search_ms") > 60000)
      throw std::invalid_argument("robot_search_ms must be in [0,60000]");
    if (record) settings["record_fps"] = 30.;
    std::optional<PrimitiveCosts> motion_costs;
    if (value("rx_wrist_speed_rad_s") > 0 || value("rx_wrist_settle_s") != .15 ||
        value("rx_jaw_duration_s") != .5 || value("rx_jaw_settle_s") != .15) {
      motion_costs.emplace();
      for (int a = 0; a < 12; ++a)
        for (int mode = 0; mode < 3; ++mode) {
          const double factor = mode == 2   ? value("rx_wrist_empty_multiplier")
                                : mode == 0 ? value("rx_wrist_whole_multiplier")
                                            : 1.;
          const double dt = value("timestep_s");
          const double turn =
              value("rx_wrist_speed_rad_s") > 0
                  ? wrist_duration((a % 6 < 2 ? 1 : 2) * pi / 2,
                                   factor * value("rx_wrist_speed_rad_s"),
                                   factor * value("rx_wrist_accel_rad_s2"), dt,
                                   value("rx_wrist_zero_jerk") != 0,
                                   mode != 2 && value("rx_wrist_brake_half_turn") != 0,
                                   value("rx_wrist_cruise_fraction"))
                  : std::max(1L, std::lround(value("turn_duration_s") / dt)) * dt;
          motion_costs->duration[a][mode] =
              a % 6 >= 4 ? (std::max(1L, std::lround(value("rx_jaw_duration_s") / dt)) +
                            std::lround(value("rx_jaw_settle_s") / dt)) *
                               dt
                         : turn + std::lround(value("rx_wrist_settle_s") / dt) * dt;
        }
    }

    std::array<std::array<int, 2>, 2> wrist_ranges{{{-2, 2}, {-2, 2}}};
    if (!full_body_bundle.empty()) {
      std::ifstream input(full_body_bundle / "grasp_pose.json");
      Json pose;
      if (!input) throw std::runtime_error("缺少整机抓持姿态");
      input >> pose;
      wrist_ranges = pose.at("wrist_ranges").get<decltype(wrist_ranges)>();
    }
    const auto requested = split_moves(sequence);
    for (const auto& move : requested) parse_move(move);
    if (plan_only) {
      if (solve || record) throw std::invalid_argument("Plan-only cannot solve or record physics");
      auto plan = plan_mechanical_moves(requested, Mat::Identity(), value("turn_duration_s"),
                                        value("robot_search_ms"),
                                        motion_costs ? &*motion_costs : nullptr, wrist_ranges);
      std::filesystem::create_directories(out);
      std::ofstream file(out / "plan.json");
      file << Json{{"planning_only", true},
                   {"moves", requested},
                   {"parameters", settings},
                   {"optimization", plan.report},
                   {"actions", plan_json(plan.actions)}}
                  .dump(2);
      file.close();
      if (!file) throw std::runtime_error("Cannot write mechanical plan");
      std::cout << plan.report << std::endl;
      return 0;
    }
    if (value("record_fps") > 60 ||
        (value("record_fps") != 0 &&
         (value("record_fps") < 1 || std::floor(value("record_fps")) != value("record_fps"))))
      throw std::invalid_argument("Recording FPS must be an integer in [1,60], or zero");
    report["parameters"] = settings;
    if (value("retention_mode") == 1) report["model"] = "experimental_joint_guided";
    report["video_playback"] = 1;
    std::filesystem::create_directories(out);
    prepared = true;
    std::ofstream scene_file(out / "scene.xml");
    const bool rx = !rx_bundle.empty();
    auto xml = scene();
    std::string final_scene =
        (rx ? mechanical_rx_scene(xml, rx_bundle, value("rx_bearing_friction_nm"),
                                  value("rx_tip_friction"), value("rx_tip_contact_time_s"),
                                  value("cube_mass_kg"), value("rx_tip_corner_spheres") != 0,
                                  value("rx_tip_sphere_radius_m"), value("rx_tip_sphere_offset_m"),
                                  value("tip_torsional_friction_m"))
            : xml);
    if (!full_body_bundle.empty()) {
      if (!rx) throw std::invalid_argument("整机模式需要 --rx-bundle");
      final_scene = rx_full_body_scene(final_scene, full_body_bundle);
    }
    scene_file << final_scene;
    scene_file.close();
    if (!scene_file) throw std::runtime_error("Cannot write mechanical scene");
    Simulation sim(out / "scene.xml");
    auto* m = sim.model;
    auto* d = sim.data;
    CubeMagnets magnets(m);
    if (value("cube_magnet_peak_force_n") > 0 &&
        (magnets.corner_count() != 24 || magnets.edge_count() != 24))
      throw std::runtime_error("Cube magnets require 24 corner and 24 edge sites");
    report["equalities"] = m->neq;
    report["actuators"] = m->nu;
    report["geoms"] = m->ngeom;
    report["velocity_dofs"] = m->nv;
    int cube_shells = 0, ball_joints = 0;
    for (int j = 0; j < m->njnt; ++j) ball_joints += m->jnt_type[j] == mjJNT_BALL;
    for (int g = 0; g < m->ngeom; ++g) {
      const char* name = mj_id2name(m, mjOBJ_GEOM, g);
      if (name && std::string_view(name).starts_with("piece_") &&
          std::string_view(name).ends_with("_shell"))
        ++cube_shells;
    }
    report["ball_joints"] = ball_joints;
    report["cube_shells"] = cube_shells;
    report["fixed_core_test_rig"] = !rx;
    report["gripper"] = rx ? "rx_friction" : "none";
    report["cube_magnets"] = {{"peak_pair_force_n", value("cube_magnet_peak_force_n")},
                              {"range_m", .006},
                              {"model", "experimental_compact_radial_potential"}};
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
    std::unique_ptr<RxFullBody> full_body;
    if (!full_body_bundle.empty())
      full_body = std::make_unique<RxFullBody>(m, d, full_body_bundle,
                                               value("rx_full_body_centering_gain_s"),
                                               value("rx_full_body_realign_open") != 0);
    for (const char* hand : {"A", "B"}) {
      if (!rx || full_body) break;
      int a = mj_name2id(m, mjOBJ_ACTUATOR, (std::string(hand) + "_yaw_drive").c_str());
      m->actuator_gainprm[10 * a] = value("rx_wrist_kp");
      m->actuator_biasprm[10 * a + 1] = -value("rx_wrist_kp");
      m->actuator_biasprm[10 * a + 2] = -value("rx_wrist_kv");
    }
    std::optional<RxWristTn> wrist_tn;
    if (value("rx_wrist_tn") == 1 && !full_body) {
      if (!rx) throw std::invalid_argument("Wrist TN requires RX grippers");
      wrist_tn.emplace(m);
    }
    report["wrist_tn_enabled"] = wrist_tn.has_value();
    if (wrist_tn) {
      report["wrist_tn_profile"] = "RX-P2 S conservative 0.05 rpm; virtual wrist selection";
      report["wrist_tn"] = Json::array();
      for (const auto& a : wrist_tn->axes)
        report["wrist_tn"].push_back({{"actuator", mj_id2name(m, mjOBJ_ACTUATOR, a.actuator)},
                                      {"peak_speed_rad_s", 0.},
                                      {"peak_torque_nm", 0.},
                                      {"max_excess_nm", 0.},
                                      {"reduced_steps", 0},
                                      {"static_limit_nm", a.static_limit}});
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
      double position = 0, angle = 0, facelet_component = 0;
      Json errors = Json::array();
      for (size_t i = 0; i < parts.size(); ++i) {
        int b = mj_name2id(m, mjOBJ_BODY, parts[i].name.c_str()),
            site = mj_name2id(m, mjOBJ_SITE, (parts[i].name + "_center").c_str());
        Vec error = core_rotation().transpose() *
                        (Eigen::Map<const Vec>(d->site_xpos + 3 * site) - core_position()) -
                    pitch_m * slots[i];
        position = std::max(position, error.norm());
        if (parts[i].slot.squaredNorm() >= 2)
          facelet_component = std::max(facelet_component, error.cwiseAbs().maxCoeff());
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
                  {"facelet_position_component_error_m", facelet_component},
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
      if (full_body && name != "internal") {
        cam.lookat[2] = -.15;
        cam.distance = 1.8;
        cam.azimuth = -50;
      }
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
        cv::putText(bgr,
                    value("retention_mode") == 1 ? "Joint-guided cube / friction grippers"
                                                 : "Contact-retained mechanical prototype",
                    {15, 28}, cv::FONT_HERSHEY_SIMPLEX, .55, {230, 230, 230}, 1, cv::LINE_AA);
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
    // 几何归属不随积分变化，避免在每个接触、每个物理步重复分配名称字符串。
    std::vector<int> contact_owner(m->ngeom);
    for (int g = 0; g < m->ngeom; ++g) {
      const char* name = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
      const std::string_view body = name ? name : "";
      contact_owner[g] = body.starts_with("A_")                                    ? 1
                         : body.starts_with("B_")                                  ? 2
                         : body.starts_with("piece_") || body == "mechanical_core" ? 3
                                                                                   : 0;
    }
    auto check_rotation_contacts = [&]() {
      const std::array<bool, 2> open{!closed["A"], !closed["B"]};
      for (int i = 0; i < d->ncon; ++i) {
        const auto& c = d->contact[i];
        int a = contact_owner[c.geom[0]], b = contact_owner[c.geom[1]];
        bool bad = (a == 1 && b == 2) || (a == 2 && b == 1);
        for (int h = 1; h <= 2; ++h)
          if (open[h - 1] && ((a == h && b == 3) || (b == h && a == 3))) bad = true;
        if (bad) {
          mjtNum f[6];
          mj_contactForce(m, d, i, f);
          if (f[0] > .01) throw std::runtime_error("RX forbidden rotation contact");
        }
      }
    };
    auto advance = [&](double t, bool check_contacts = false) {
      for (int k = 0; k < std::lround(t / m->opt.timestep); ++k) {
        if (value("cube_magnet_peak_force_n") > 0) mju_zero(d->qfrc_applied, m->nv);
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
        if (wrist_tn) wrist_tn->before_step(m, d);
        if (full_body || value("cube_magnet_peak_force_n") > 0) {
          // 场景固定使用 implicitfast；分步接口复用运动学，避免磁吸再计算一次。
          mj_step1(m, d);
          if (full_body) full_body->control(d, {closed["A"], closed["B"]});
          magnets.apply_prepared(m, d, value("cube_magnet_peak_force_n"));
          mj_step2(m, d);
        } else {
          mj_step(m, d);
        }
        if (wrist_tn) {
          wrist_tn->after_step(d);
          // 保留逐步峰值记录，复用报告对象，避免反复构造数组和字段。
          for (size_t h = 0; h < wrist_tn->axes.size(); ++h) {
            const auto& a = wrist_tn->axes[h];
            auto& row = report["wrist_tn"][h];
            row["peak_speed_rad_s"] = a.peak_speed_rad_s;
            row["peak_torque_nm"] = a.peak_torque_nm;
            row["max_excess_nm"] = a.max_excess_nm;
            row["reduced_steps"] = a.reduced_steps;
          }
        }
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

    double cube_mass = 0;
    Vec com = Vec::Zero();
    std::vector<int> cube_bodies;
    for (int b = 1; b < m->nbody; ++b) {
      const char* name = mj_id2name(m, mjOBJ_BODY, b);
      if (name && (std::string_view(name).starts_with("piece_") ||
                   std::string_view(name) == "mechanical_core")) {
        cube_bodies.push_back(b);
        cube_mass += m->body_mass[b];
        com += m->body_mass[b] * Eigen::Map<const Vec>(d->xipos + 3 * b);
      }
    }
    com /= cube_mass;
    Mat inertia = Mat::Zero();
    for (int b : cube_bodies) {
      Vec r = Eigen::Map<const Vec>(d->xipos + 3 * b) - com;
      Mat rot = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d->ximat + 9 * b);
      inertia +=
          rot * Eigen::Map<const Vec>(m->body_inertia + 3 * b).asDiagonal() * rot.transpose() +
          m->body_mass[b] * (r.squaredNorm() * Mat::Identity() - r * r.transpose());
    }
    report["cube_mass_properties"] = {
        {"dynamic_mass_kg", cube_mass},
        {"center_of_mass_world_m", xyz(com)},
        {"inertia_about_com_kg_m2", matrix_json(inertia)},
        {"distribution", "estimated, uniformly scaled original piece/core allocation"}};
    report["initial"] = measure();
    if (full_body) report["full_body_initial"] = full_body->report(d);
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
        const std::string prefix = hand + "_" + side + "_";
        auto is_tip = [&](int geom) {
          const char* name = mj_id2name(m, mjOBJ_GEOM, geom);
          return name && std::string_view(name).starts_with(prefix) &&
                 std::string_view(name).ends_with("_tip");
        };
        double normal = 0;
        for (int i = 0; i < d->ncon; ++i) {
          const auto& c = d->contact[i];
          int other = is_tip(c.geom[0]) ? c.geom[1] : is_tip(c.geom[1]) ? c.geom[0] : -1;
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
      if (full_body && name.ends_with("_yaw_drive")) {
        const int h = name.starts_with("A") ? 0 : 1;
        const double start = full_body->angle(h), began = d->time;
        const std::string mode = report["current_action"].value("mode", "");
        const double factor = mode == "empty"   ? value("rx_wrist_empty_multiplier")
                              : mode == "whole" ? value("rx_wrist_whole_multiplier")
                                                : 1.;
        duration = wrist_duration(target - start, factor * value("rx_wrist_speed_rad_s"),
                                  factor * value("rx_wrist_accel_rad_s2"), m->opt.timestep,
                                  value("rx_wrist_zero_jerk") != 0,
                                  mode != "empty" && value("rx_wrist_brake_half_turn") != 0,
                                  value("rx_wrist_cruise_fraction"));
        const int count = std::lround(duration / m->opt.timestep);
        for (int k = 1; k <= count; k++) {
          auto ref = wrist_sample(start, target, duration, k * m->opt.timestep,
                                  value("rx_wrist_zero_jerk") != 0,
                                  mode != "empty" && value("rx_wrist_brake_half_turn") != 0,
                                  value("rx_wrist_cruise_fraction"));
          full_body->reference(h, ref.position, m->opt.timestep);
          advance(m->opt.timestep, true);
        }
        full_body->reference(h, target, m->opt.timestep);
        report["full_body"] = full_body->report(d);
        if (full_body->position_error(d, h) > .001)
          throw std::runtime_error("RX整臂末端位置跟踪失败");
        report["wrist_motions"].push_back({{"logical_axis", h == 0 ? "A" : "B"},
                                           {"mode", mode},
                                           {"start_time_s", began},
                                           {"duration_s", duration},
                                           {"target", target},
                                           {"end_tracking_error_rad", full_body->error(d, h)},
                                           {"native_arm_execution", true}});
        return;
      }
      int a = mj_name2id(m, mjOBJ_ACTUATOR, name.c_str());
      if (a < 0) throw std::runtime_error("Missing RX actuator: " + name);
      double start = d->ctrl[a];
      bool turning = name.ends_with("_yaw_drive");
      bool controlled = turning && value("rx_wrist_speed_rad_s") > 0;
      const bool empty = report.contains("current_action") &&
                         report["current_action"].value("mode", "") == "empty";
      const double factor = empty ? value("rx_wrist_empty_multiplier")
                            : report.contains("current_action") &&
                                    report["current_action"].value("mode", "") == "whole"
                                ? value("rx_wrist_whole_multiplier")
                                : 1.;
      if (controlled)
        duration = wrist_duration(target - start, factor * value("rx_wrist_speed_rad_s"),
                                  factor * value("rx_wrist_accel_rad_s2"), m->opt.timestep,
                                  value("rx_wrist_zero_jerk") != 0,
                                  !empty && value("rx_wrist_brake_half_turn") != 0,
                                  value("rx_wrist_cruise_fraction"));
      int count = std::max(1L, std::lround(duration / m->opt.timestep));
      duration = count * m->opt.timestep;
      int v = turning ? m->jnt_dofadr[m->actuator_trnid[2 * a]] : -1;
      double inertia = 0;
      if (controlled) {
        mj_forward(m, d);
        std::vector<mjtNum> mass(m->nv * m->nv);
        mj_fullM(m, d, mass.data());
        inertia = mass[v * m->nv + v];
      }
      double peak_error = 0, peak_speed = 0, peak_torque = 0, began = d->time;
      for (int k = 1; k <= count; ++k) {
        auto ref = wrist_sample(start, target, duration, k * m->opt.timestep,
                                turning && value("rx_wrist_zero_jerk") != 0,
                                controlled && !empty && value("rx_wrist_brake_half_turn") != 0,
                                controlled ? value("rx_wrist_cruise_fraction") : 0.);
        d->ctrl[a] = ref.position;
        if (controlled)
          d->ctrl[a] += ((value("rx_wrist_kv") + m->dof_damping[v]) * ref.velocity +
                         inertia * ref.acceleration) /
                        value("rx_wrist_kp");
        advance(m->opt.timestep, turning);
        if (turning) {
          peak_error =
              std::max(peak_error,
                       std::abs(d->qpos[m->jnt_qposadr[m->actuator_trnid[2 * a]]] - ref.position));
          peak_speed = std::max(peak_speed, std::abs(d->qvel[v]));
          peak_torque = std::max(peak_torque, std::abs(d->actuator_force[a]));
        }
      }
      d->ctrl[a] = target;
      advance(turning ? value("rx_wrist_settle_s") : value("rx_jaw_settle_s"), turning);
      if (turning)
        report["wrist_motions"].push_back(
            {{"actuator", name},
             {"mode", report["current_action"].value("mode", "")},
             {"start_time_s", began},
             {"duration_s", duration},
             {"midpoint_brake", controlled && !empty && value("rx_wrist_brake_half_turn") != 0 &&
                                    std::abs(std::abs(target - start) - pi) < 1e-9},
             {"settle_s", value("rx_wrist_settle_s")},
             {"peak_tracking_error_rad", peak_error},
             {"peak_speed_rad_s", peak_speed},
             {"peak_torque_nm", peak_torque},
             {"end_tracking_error_rad", d->qpos[m->jnt_qposadr[m->actuator_trnid[2 * a]]] - target},
             {"end_speed_rad_s", d->qvel[v]},
             {"feedforward_inertia_kg_m2", inertia},
             {"target", target}});
    };
    auto jaw = [&](const std::string& h, bool close) {
      command(h + "_fingers_actuator", close ? -1.20 : -2.25, value("rx_jaw_duration_s"));
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
    auto check_pose = [&](const std::string& label, bool checkpoint = true, bool readout = false) {
      const double position_limit = readout ? .0005 : value("rx_alignment_position_m");
      const double angle_limit = readout ? .02 : value("rx_alignment_angle_rad");
      const double core_limit = readout ? .001 : value("rx_core_position_m");
      const bool had_action = report.contains("current_action");
      const Json pending_action = report.value("current_action", Json::object());
      auto read_pose = [&]() {
        auto state = measure();
        state["core_position"] = xyz(core_position());
        state["core_orientation_error_rad"] = std::acos(std::clamp(
            ((robot_orientation.transpose() * core_rotation()).trace() - 1) / 2, -1., 1.));
        return state;
      };
      auto aligned = [&](const Json& state) {
        return (!readout ||
                state["facelet_position_component_error_m"].get<double>() <= pitch_m * .02) &&
               state["position_error_m"].get<double>() <= position_limit &&
               state["orientation_error_rad"].get<double>() <= angle_limit &&
               state["core_orientation_error_rad"].get<double>() <= angle_limit &&
               (core_position() - Vec(0, 0, .1)).norm() <= core_limit;
      };
      auto state = read_pose();
      auto wait_alignment = [&]() {
        report["current_action"] = {{"kind", "alignment_wait"}, {"move", label}};
        report["alignment_waits"].push_back(
            {{"move", label}, {"initial", state}, {"samples", Json::array()}});
        auto& wait = report["alignment_waits"].back();
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
        return stable == 3;
      };
      bool timed_out = false;
      if (checkpoint && !aligned(state) && value("alignment_timeout_s") > 0) {
        timed_out = !wait_alignment();
        const std::string first = pending_action.value("hand", "") == "A" ? "B" : "A";
        for (int attempt = 0;
             timed_out && attempt < settings["alignment_regrasp_attempts"].get<int>(); ++attempt) {
          const std::string hand = attempt == 0 ? first : first == "A" ? "B" : "A";
          const std::string support = hand == "A" ? "B" : "A";
          if (!closed["A"] || !closed["B"] || !grip_ready(support) ||
              state["position_error_m"].get<double>() > .0015 ||
              state["orientation_error_rad"].get<double>() > .05 ||
              state["core_orientation_error_rad"].get<double>() > .05 ||
              (core_position() - Vec(0, 0, .1)).norm() > .001)
            break;
          report["alignment_regrasps"].push_back({{"move", label},
                                                  {"hand", hand},
                                                  {"start_time_s", d->time},
                                                  {"initial", state},
                                                  {"actions", Json::array()},
                                                  {"passed", false}});
          auto& recovery = report["alignment_regrasps"].back();
          try {
            for (bool close : {false, true}) {
              report["current_action"] = {{"kind", "jaw"},
                                          {"hand", hand},
                                          {"target", close ? 115 : 0},
                                          {"mode", "alignment_regrasp"},
                                          {"move", label}};
              recovery["actions"].push_back({{"action", report["current_action"]},
                                             {"start_time_s", d->time},
                                             {"initial", read_pose()},
                                             {"completed", false}});
              auto& action = recovery["actions"].back();
              try {
                if (!grip_ready(support)) throw std::runtime_error("RX grip lost: " + support);
                jaw(hand, close);
                if (!grip_ready(support)) throw std::runtime_error("RX grip lost: " + support);
                action["completed"] = true;
              } catch (...) {
                action["end_time_s"] = d->time;
                action["final"] = read_pose();
                throw;
              }
              action["end_time_s"] = d->time;
              action["final"] = read_pose();
            }
            state = read_pose();
            timed_out = !wait_alignment();
            recovery["passed"] = !timed_out && grip_ready("A") && grip_ready("B");
          } catch (...) {
            recovery["end_time_s"] = d->time;
            recovery["final"] = read_pose();
            throw;
          }
          recovery["end_time_s"] = d->time;
          recovery["final"] = state;
        }
      }
      state["move"] = label;
      double core_angle = state["core_orientation_error_rad"];
      report["steps"].push_back(state);
      std::cout << state << std::endl;
      snapshot("step_" + std::to_string(report["steps"].size()));
      // 单手换向后还未重新夹紧；记录瞬态偏差，转层前仍执行完整验收。
      if (!checkpoint) return;
      if (timed_out) throw std::runtime_error("RX alignment timeout: " + label);
      if (state["position_error_m"].get<double>() > position_limit ||
          state["orientation_error_rad"].get<double>() > angle_limit)
        throw std::runtime_error("RX mechanical pose mismatch: " + label);
      if ((core_position() - Vec(0, 0, .1)).norm() > core_limit || core_angle > angle_limit)
        throw std::runtime_error("RX core slipped: " + label);
      if (!aligned(state))
        throw std::runtime_error("RX physical readout alignment failed: " + label);
      for (auto h : {"A", "B"})
        if (closed[h] && !grip_ready(h))
          throw std::runtime_error("RX grip lost: " + std::string(h));
      if (had_action)
        report["current_action"] = pending_action;
      else
        report.erase("current_action");
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
        auto selected = plan_mechanical_moves(
            moves, robot_orientation, value("turn_duration_s"), value("robot_search_ms"),
            motion_costs ? &*motion_costs : nullptr, wrist_ranges);
        report["planning"].push_back(selected.report);
        auto& plan = selected.actions;
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
            double start = full_body ? full_body->angle(a.hand == "A" ? 0 : 1) : d->ctrl[actuator];
            command(a.hand + "_yaw_drive", a.target, value("turn_duration_s"));
            int joint = mj_name2id(m, mjOBJ_JOINT, (a.hand + "_yaw").c_str());
            if ((full_body ? full_body->error(d, a.hand == "A" ? 0 : 1)
                           : std::abs(d->qpos[m->jnt_qposadr[joint]] - a.target)) > .006)
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
    if (rx) check_pose("scramble readout", true, true);
    report["scrambled_facelets"] = physical_facelets();
    if (solve) {
      auto solution = solve_facelets(report["scrambled_facelets"].get<std::string>(), root);
      report["solution"] = solution;
      execute_moves(solution);
    }
    // 录像收尾不能替代动作结束时的验收。
    if (rx && value("record_fps") > 0) check_pose("before recording tail", true, true);
    report["execution_time_s"] = d->time;
    if (value("record_fps") > 0) advance(.5, rx);
    if (rx) check_pose("final", true, true);
    report["final_facelets"] = physical_facelets();
    report["solved"] = report["final_facelets"] == solved;
    if (solve && !report["solved"].get<bool>())
      throw std::runtime_error("Mechanical cube did not finish solved");
    if (full_body) report["full_body"] = full_body->report(d);
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
