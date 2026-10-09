#include <fstream>
#include <iostream>

#include "rm/cube.hpp"

using namespace rm;
using namespace rm::cube;

// 仅用于离散构型的几何检查；不执行动力学，不作为物理还原结果。
int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::invalid_argument("用法: sweep 场景XML 输出JSON");
    Simulation sim(argv[1]);
    auto* m = sim.model;
    auto* d = sim.data;
    Json rows = Json::array();
    double worst = 0;
    for (char face : faces) {
      const auto [axis, sign] = face_axis(face);
      for (int deg = -180; deg <= 180; deg += 5) {
        mju_copy(d->qpos, m->qpos0, m->nq);
        const Eigen::Quaterniond q(Eigen::AngleAxisd(sign * deg * pi / 180, Vec::Unit(axis)));
        int selected = 0;
        for (int b = 1; b < m->nbody; ++b) {
          const char* name = mj_id2name(m, mjOBJ_BODY, b);
          if (!name) continue;
          int site = mj_name2id(m, mjOBJ_SITE, (std::string(name) + "_center").c_str());
          if (site < 0 || sign * m->site_pos[3 * site + axis] < .01) continue;
          ++selected;
          const int j = m->body_jntadr[b], adr = m->jnt_qposadr[j];
          if (m->jnt_type[j] == mjJNT_FREE) {
            d->qpos[adr + 3] = q.w();
            d->qpos[adr + 4] = q.x();
            d->qpos[adr + 5] = q.y();
            d->qpos[adr + 6] = q.z();
          } else if (m->jnt_type[j] == mjJNT_HINGE) {
            d->qpos[adr] = deg * pi / 180;
          } else {
            throw std::runtime_error("Unsupported test body joint");
          }
        }
        if (selected != 9) throw std::runtime_error("Sweep must select nine physical pieces");
        mj_forward(m, d);
        Json collisions = Json::array();
        double penetration = 0;
        for (int i = 0; i < d->ncon; ++i) {
          const auto& c = d->contact[i];
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
    Json report = {{"mode", "geometry_only"},
                   {"passed", worst <= .00002},
                   {"max_penetration_m", worst},
                   {"samples", rows}};
    std::ofstream out(argv[2]);
    out << report.dump(2) << '\n';
    if (!out) throw std::runtime_error("Cannot write sweep report");
    std::cout << "samples=" << rows.size() << " max_penetration_m=" << worst << '\n';
    return worst <= .00002 ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
