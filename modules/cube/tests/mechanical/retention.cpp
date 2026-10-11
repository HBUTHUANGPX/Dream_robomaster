#include <cmath>
#include <fstream>
#include <iostream>
#include <string_view>

#include "rm/cube.hpp"
#include "rm/mechanical_sdf.hpp"

using namespace rm;
using namespace rm::cube;

// 独立离散几何诊断；固定其他块，仅测试单块向外径向平移。
int main(int argc, char** argv) {
  try {
    if (argc != 3 && argc != 4)
      throw std::invalid_argument("用法: retention 场景XML 输出JSON [中心张开量m]");
    double center_opening = 0;
    if (argc == 4) {
      size_t consumed = 0;
      center_opening = std::stod(argv[3], &consumed);
      if (consumed != std::string_view(argv[3]).size() || !std::isfinite(center_opening) ||
          center_opening < 0 || center_opening > .003)
        throw std::invalid_argument("Center opening must be finite and within [0, 0.003] m");
    }
    register_mechanical_sdf();
    Simulation sim(argv[1]);
    auto* m = sim.model;
    auto* d = sim.data;
    int preload_count = 0;
    for (int j = 0; j < m->njnt; ++j) {
      const char* name = mj_id2name(m, mjOBJ_JOINT, j);
      if (!name || !std::string_view(name).starts_with("preload_")) continue;
      if (m->jnt_type[j] != mjJNT_SLIDE) throw std::runtime_error("Preload joint must be a slide");
      if (argc == 4) d->qpos[m->jnt_qposadr[j]] = center_opening;
      ++preload_count;
    }
    if (argc == 4 && preload_count != 6)
      throw std::runtime_error("Expected six center preload joints");
    std::vector<bool> active(m->ngeom, false);
    for (int g = 0; g < m->ngeom; ++g)
      active[g] = m->geom_contype[g] != 0 || m->geom_conaffinity[g] != 0;
    for (int p = 0; p < m->npair; ++p) active[m->pair_geom1[p]] = active[m->pair_geom2[p]] = true;
    auto forward = [&]() {
      mj_forward(m, d);
      for (int i = 0; i < mjNWARNING; ++i)
        if (d->warning[i].number) throw std::runtime_error("MuJoCo warning: " + std::to_string(i));
    };
    forward();
    const int core = mj_name2id(m, mjOBJ_BODY, "mechanical_core");
    int core_geom = -1;
    if (core < 0) {
      for (int g = 0; g < m->ngeom; ++g)
        if (m->geom_bodyid[g] == 0 && m->geom_type[g] == mjGEOM_SPHERE && active[g] &&
            (Eigen::Map<const Vec>(d->geom_xpos + 3 * g) - Vec(0, 0, .1)).norm() < 1e-9) {
          if (core_geom >= 0) throw std::runtime_error("Ambiguous fixed core sphere");
          core_geom = g;
        }
      if (core_geom < 0) throw std::runtime_error("Missing mechanical core");
    }
    const Vec origin = core >= 0 ? Vec(Eigen::Map<const Vec>(d->xpos + 3 * core))
                                 : Vec(Eigen::Map<const Vec>(d->geom_xpos + 3 * core_geom));
    Mat core_rotation = Mat::Identity();
    if (core >= 0)
      core_rotation =
          Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d->xmat + 9 * core);
    struct Piece {
      int body, site, joint, adr;
      std::string name;
    };
    std::vector<Piece> pieces, free_pieces;
    std::vector<bool> internal_body(m->nbody, false);
    if (core >= 0) internal_body[core] = true;
    for (int b = 1; b < m->nbody; ++b) {
      const char* name = mj_id2name(m, mjOBJ_BODY, b);
      if (!name || !std::string_view(name).starts_with("piece_")) continue;
      const int site = mj_name2id(m, mjOBJ_SITE, (std::string(name) + "_center").c_str());
      if (site < 0 || m->site_bodyid[site] != b || m->body_jntnum[b] < 1)
        throw std::runtime_error("Invalid mechanical piece: " + std::string(name));
      const int joint = m->body_jntadr[b];
      Piece p{b, site, joint, m->jnt_qposadr[joint], name};
      pieces.push_back(p);
      internal_body[b] = true;
      if (m->jnt_type[joint] == mjJNT_FREE) free_pieces.push_back(p);
    }
    if (free_pieces.size() != 20 || pieces.size() != 26)
      throw std::runtime_error("Expected 20 free edge/corner pieces and 26 total pieces");
    auto internal_geom = [&](int g) {
      return g >= 0 && active[g] && (internal_body[m->geom_bodyid[g]] || g == core_geom);
    };
    const std::vector<mjtNum> initial(d->qpos, d->qpos + m->nq);
    Json rows = Json::array();
    int static_count = 0, unblocked_count = 0;
    for (int angle = 0; angle <= 90; angle += 15) {
      mju_copy(d->qpos, initial.data(), m->nq);
      forward();
      const auto [axis, sign] = face_axis('F');
      const Eigen::Quaterniond turn(
          Eigen::AngleAxisd(sign * angle * pi / 180, core_rotation * Vec::Unit(axis)));
      int selected = 0;
      for (const auto& p : pieces) {
        const Vec position =
            core_rotation.transpose() * (Eigen::Map<const Vec>(d->site_xpos + 3 * p.site) - origin);
        if (sign * position[axis] < .01) continue;
        ++selected;
        if (m->jnt_type[p.joint] == mjJNT_FREE) {
          const Vec translated = origin + turn * (Eigen::Map<const Vec>(d->qpos + p.adr) - origin);
          for (int k = 0; k < 3; ++k) d->qpos[p.adr + k] = translated[k];
          const Eigen::Quaterniond q =
              turn * Eigen::Quaterniond(d->qpos[p.adr + 3], d->qpos[p.adr + 4], d->qpos[p.adr + 5],
                                        d->qpos[p.adr + 6]);
          d->qpos[p.adr + 3] = q.w();
          d->qpos[p.adr + 4] = q.x();
          d->qpos[p.adr + 5] = q.y();
          d->qpos[p.adr + 6] = q.z();
        } else if (m->jnt_type[p.joint] == mjJNT_HINGE) {
          d->qpos[p.adr] += angle * pi / 180;
        } else {
          throw std::runtime_error("Unsupported piece joint");
        }
      }
      if (selected != 9) throw std::runtime_error("F sweep must select nine pieces");
      forward();
      const std::vector<mjtNum> angled(d->qpos, d->qpos + m->nq);
      for (const auto& p : free_pieces) {
        mju_copy(d->qpos, angled.data(), m->nq);
        forward();
        Vec radial = Eigen::Map<const Vec>(d->site_xpos + 3 * p.site) - origin;
        if (!radial.allFinite() || radial.norm() < 1e-9)
          throw std::runtime_error("Invalid piece radial direction");
        radial.normalize();
        Json row = {{"angle_deg", angle},           {"piece", p.name},
                    {"first_contact_m", nullptr},   {"blocking_body", nullptr},
                    {"static_interference", false}, {"unblocked_within_3mm", true}};
        for (int step = 0; step <= 30; ++step) {
          const double displacement = step * .0001;
          mju_copy(d->qpos, angled.data(), m->nq);
          for (int k = 0; k < 3; ++k) d->qpos[p.adr + k] += displacement * radial[k];
          forward();
          double penetration = .00002;
          int blocker = -1;
          for (int i = 0; i < d->ncon; ++i) {
            const auto& c = d->contact[i];
            if (!internal_geom(c.geom[0]) || !internal_geom(c.geom[1])) continue;
            const int b0 = m->geom_bodyid[c.geom[0]], b1 = m->geom_bodyid[c.geom[1]];
            if (b0 == b1 || (b0 != p.body && b1 != p.body)) continue;
            if (-c.dist > penetration) {
              penetration = -c.dist;
              blocker = b0 == p.body ? b1 : b0;
            }
          }
          if (blocker < 0) continue;
          const char* name = mj_id2name(m, mjOBJ_BODY, blocker);
          row["first_contact_m"] = displacement;
          row["blocking_body"] = name ? name : "world";
          row["penetration_m"] = penetration;
          row["static_interference"] = step == 0;
          row["unblocked_within_3mm"] = false;
          if (step == 0) ++static_count;
          break;
        }
        if (row.at("unblocked_within_3mm").get<bool>()) ++unblocked_count;
        rows.push_back(row);
      }
    }
    Json report = {{"mode", "geometry_only_radial_retention"},
                   {"scene", argv[1]},
                   {"center_opening_m", argc == 4 ? Json(center_opening) : Json(nullptr)},
                   {"face", "F"},
                   {"angle_convention", "sweep.apply(F, positive_degrees)"},
                   {"free_piece_count", free_pieces.size()},
                   {"radial_step_m", .0001},
                   {"radial_max_m", .003},
                   {"penetration_threshold_m", .00002},
                   {"static_interference_count", static_count},
                   {"unblocked_count", unblocked_count},
                   {"note",
                    "仅离散几何扫描；零位干涉不代表径向保持。未发现阻挡仅指采样点；不证明连续路径畅"
                    "通或动力学通过。"},
                   {"samples", rows}};
    std::ofstream out(argv[2]);
    out << report.dump(2) << '\n';
    out.flush();
    if (!out) throw std::runtime_error("Cannot write retention report");
    out.close();
    if (!out) throw std::runtime_error("Cannot close retention report");
    std::cout << "samples=" << rows.size() << " static_interference=" << static_count
              << " unblocked=" << unblocked_count << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
