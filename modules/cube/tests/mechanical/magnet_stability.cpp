#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "rm/cube_magnets.hpp"

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("需要场景XML");
    char error[2048]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(
        mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
    if (!m) throw std::runtime_error(error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
    rm::cube::CubeMagnets magnets(m.get());
    std::vector<int> pieces, balls, shells;
    for (int b = 1; b < m->nbody; ++b) {
      const char* name = mj_id2name(m.get(), mjOBJ_BODY, b);
      if (!name || !std::string_view(name).starts_with("piece_")) continue;
      pieces.push_back(b);
      const int j = m->body_jntadr[b];
      if (m->jnt_type[j] == mjJNT_BALL) balls.push_back(b);
    }
    for (int g = 0; g < m->ngeom; ++g) {
      const char* name = mj_id2name(m.get(), mjOBJ_GEOM, g);
      if (name && std::string_view(name).ends_with("_shell")) shells.push_back(g);
    }
    if (magnets.corner_count() != 24 || magnets.edge_count() != 24 || balls.size() != 20 ||
        shells.size() != 26)
      throw std::runtime_error("需要包含48个磁吸点、20个球关节及26个外壳的导向魔方场景");
    mj_resetData(m.get(), d.get());
    magnets.apply(m.get(), d.get(), .3);
    const double initial = magnets.potential(m.get(), d.get(), .3);
    std::vector<mjtNum> direction(m->nv, 0);
    auto energy = [&](double angle) {
      mj_resetData(m.get(), d.get());
      mj_integratePos(m.get(), d->qpos, direction.data(), angle);
      magnets.apply(m.get(), d.get(), .3);
      return magnets.potential(m.get(), d.get(), .3) - initial;
    };
    std::cout << std::setprecision(15)
              << "kind\tpiece\taxis\tangle_rad\tdelta_U_J\tvirtual_work_J\tmin_contact_m\tcontact_"
                 "geom0\tcontact_geom1\tpenetrating_contacts\tmin_geom_distance_m\tdistance_"
                 "geom0\tdistance_geom1\n";
    auto report = [&](const std::string& kind, const std::string& name, int axis, double angle,
                      const std::vector<int>& moved) {
      const double du = energy(angle);
      const double work = mju_dot(d->qfrc_applied, direction.data(), m->nv) * angle;
      // 独立静态诊断；没有步进或修改主项目的运行状态。
      mj_collision(m.get(), d.get());
      double min_contact = 0;
      int c0 = -1, c1 = -1, penetrations = 0;
      for (int i = 0; i < d->ncon; ++i) {
        const auto& c = d->contact[i];
        if (std::find(shells.begin(), shells.end(), c.geom[0]) == shells.end() ||
            std::find(shells.begin(), shells.end(), c.geom[1]) == shells.end())
          continue;
        if (c.dist < -1e-12) ++penetrations;
        if (c.dist < min_contact) {
          min_contact = c.dist;
          c0 = c.geom[0];
          c1 = c.geom[1];
        }
      }
      // 碰撞列表不包含正间隙：另外查询移动块与所有shell的有符号最近距离。
      double distance = .01;
      int g0 = -1, g1 = -1;
      for (std::size_t a = 0; a < shells.size(); ++a)
        for (std::size_t b = a + 1; b < shells.size(); ++b) {
          int ga = shells[a], gb = shells[b];
          if (!moved.empty() &&
              std::find(moved.begin(), moved.end(), m->geom_bodyid[ga]) == moved.end() &&
              std::find(moved.begin(), moved.end(), m->geom_bodyid[gb]) == moved.end())
            continue;
          const double dist = mj_geomDistance(m.get(), d.get(), ga, gb, .01, nullptr);
          if (dist < distance) {
            distance = dist;
            g0 = ga;
            g1 = gb;
          }
        }
      auto geom_name = [&](int id) { return id < 0 ? "-" : mj_id2name(m.get(), mjOBJ_GEOM, id); };
      std::cout << kind << '\t' << name << '\t' << axis << '\t' << angle << '\t' << du << '\t'
                << work << '\t' << min_contact << '\t' << geom_name(c0) << '\t' << geom_name(c1)
                << '\t' << penetrations << '\t' << distance << '\t' << geom_name(g0) << '\t'
                << geom_name(g1) << '\n';
    };
    report("baseline", "all", -1, 0, {});
    for (int body : balls)
      for (int axis = 0; axis < 3; ++axis) {
        std::fill(direction.begin(), direction.end(), 0);
        direction[m->jnt_dofadr[m->body_jntadr[body]] + axis] = 1;
        for (int sign : {-1, 1}) {
          const double du = energy(sign * .01);
          const double work = mju_dot(d->qfrc_applied, direction.data(), m->nv) * sign * .01;
          if (du > 1e-12 && work < -1e-12) continue;
          for (double angle : {.0001, .0005, .001, .002, .005, .01})
            report("ball", mj_id2name(m.get(), mjOBJ_BODY, body), axis, sign * angle, {body});
        }
      }
    for (int axis = 0; axis < 3; ++axis)
      for (int side : {-1, 1}) {
        std::fill(direction.begin(), direction.end(), 0);
        std::vector<int> moved;
        for (int body : pieces) {
          const std::string center = std::string(mj_id2name(m.get(), mjOBJ_BODY, body)) + "_center";
          const int site = mj_name2id(m.get(), mjOBJ_SITE, center.c_str());
          if (side * m->site_pos[3 * site + axis] < .01) continue;
          moved.push_back(body);
          const int joint = m->body_jntadr[body];
          if (m->jnt_type[joint] == mjJNT_BALL)
            direction[m->jnt_dofadr[joint] + axis] = side;
          else
            direction[m->jnt_dofadr[joint]] = side * m->jnt_axis[3 * joint + axis];
        }
        for (double angle : {-.01, -.001, .001, .01})
          report("layer", side < 0 ? "negative" : "positive", axis, angle, moved);
      }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
