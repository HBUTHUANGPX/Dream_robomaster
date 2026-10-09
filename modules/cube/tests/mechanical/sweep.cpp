#include <fstream>
#include <iostream>

#include "rm/cube.hpp"

using namespace rm;
using namespace rm::cube;

// 仅用于离散构型的几何检查；不执行动力学，不作为物理还原结果。
int main(int argc, char** argv) {
  try {
    if (argc < 3 || argc > 5)
      throw std::invalid_argument("用法: sweep 场景XML 输出JSON [前置打乱] [扫描面]");
    std::string prefix = argc > 3 ? argv[3] : "", scan = argc > 4 ? argv[4] : faces;
    if (scan.empty()) throw std::invalid_argument("Scan faces must not be empty");
    for (char face : scan) face_axis(face);
    Simulation sim(argv[1]);
    auto* m = sim.model;
    auto* d = sim.data;
    auto apply = [&](char face, double deg) {
      mj_forward(m, d);
      auto [axis, sign] = face_axis(face);
      Eigen::Quaterniond turn(Eigen::AngleAxisd(sign * deg * pi / 180, Vec::Unit(axis)));
      int selected = 0;
      for (int b = 1; b < m->nbody; ++b) {
        const char* name = mj_id2name(m, mjOBJ_BODY, b);
        if (!name) continue;
        int site = mj_name2id(m, mjOBJ_SITE, (std::string(name) + "_center").c_str());
        if (site < 0) continue;
        Vec position = Eigen::Map<const Vec>(d->site_xpos + 3 * site) - Vec(0, 0, .1);
        if (sign * position[axis] < .01) continue;
        ++selected;
        int j = m->body_jntadr[b], adr = m->jnt_qposadr[j];
        if (m->jnt_type[j] == mjJNT_FREE) {
          auto q = turn * Eigen::Quaterniond(d->qpos[adr + 3], d->qpos[adr + 4], d->qpos[adr + 5],
                                             d->qpos[adr + 6]);
          d->qpos[adr + 3] = q.w();
          d->qpos[adr + 4] = q.x();
          d->qpos[adr + 5] = q.y();
          d->qpos[adr + 6] = q.z();
        } else if (m->jnt_type[j] == mjJNT_HINGE)
          d->qpos[adr] += deg * pi / 180;
        else
          throw std::runtime_error("Unsupported test body joint");
      }
      if (selected != 9) throw std::runtime_error("Sweep must select nine physical pieces");
    };
    for (auto& move : split_moves(prefix)) {
      auto [f, c] = parse_move(move);
      apply(f, -c * 90.);
    }
    std::vector<mjtNum> base(d->qpos, d->qpos + m->nq);
    Json rows = Json::array();
    double worst = 0;
    auto cube_geom = [&](int geom) {
      int body = m->geom_bodyid[geom];
      const char* name = mj_id2name(m, mjOBJ_BODY, body);
      return body == 0 || (name && (std::string_view(name).starts_with("piece_") ||
                                    std::string_view(name) == "mechanical_core"));
    };
    for (char face : scan) {
      for (int deg = -180; deg <= 180; deg += 5) {
        mju_copy(d->qpos, base.data(), m->nq);
        apply(face, deg);
        mj_forward(m, d);
        Json collisions = Json::array();
        double penetration = 0;
        for (int i = 0; i < d->ncon; ++i) {
          const auto& c = d->contact[i];
          if (!cube_geom(c.geom[0]) || !cube_geom(c.geom[1])) continue;
          penetration = std::max(penetration, -c.dist);
          if (c.dist >= -.00002) continue;
          Json contact = {{"distance_m", c.dist}};
          for (int k = 0; k < 2; ++k) {
            const int geom = c.geom[k], mesh = m->geom_dataid[geom];
            contact["body" + std::to_string(k)] = m->geom_bodyid[geom];
            contact["mesh" + std::to_string(k)] =
                mesh >= 0 ? mj_id2name(m, mjOBJ_MESH, mesh) : "primitive";
          }
          collisions.push_back(contact);
        }
        worst = std::max(worst, penetration);
        rows.push_back({{"face", std::string(1, face)},
                        {"angle_deg", deg},
                        {"max_penetration_m", penetration},
                        {"collisions", collisions}});
      }
    }
    if (rows.empty()) throw std::runtime_error("Sweep produced no samples");
    Json report = {{"mode", "geometry_only"},
                   {"prefix", prefix},
                   {"passed", worst <= .00002},
                   {"max_penetration_m", worst},
                   {"samples", rows}};
    std::ofstream out(argv[2]);
    out << report.dump(2) << '\n';
    out.flush();
    if (!out) throw std::runtime_error("Cannot write sweep report");
    std::cout << "samples=" << rows.size() << " max_penetration_m=" << worst << '\n';
    return worst <= .00002 ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
