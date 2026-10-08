#include <iostream>

#include "rm/navigation.hpp"
int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::invalid_argument("root and model directory required");
    for (std::string file : {"robot.xml", "navigation.xml"}) {
      rm::Simulation baseline(std::filesystem::path(argv[1]) / "assets" / file),
          native(std::filesystem::path(argv[2]) / file);
      auto a = baseline.model, b = native.model;
      if (a->nq != b->nq || a->nu != b->nu || a->nbody != b->nbody || a->ngeom != b->ngeom ||
          a->nsensor != b->nsensor)
        throw std::runtime_error("Generated model topology differs");
      for (int i = 0; i < a->nbody; ++i)
        if (std::abs(a->body_mass[i] - b->body_mass[i]) > 1e-9)
          throw std::runtime_error("Generated body mass differs");
      for (int i = 0; i < a->njnt * 3; ++i)
        if (std::abs(a->jnt_axis[i] - b->jnt_axis[i]) > 1e-9)
          throw std::runtime_error("Generated joint axis differs");
      if (a->nsite != b->nsite) throw std::runtime_error("Generated site count differs");
      for (int i = 0; i < a->nsite; ++i) {
        const char* original = mj_id2name(a, mjOBJ_SITE, i);
        const char* generated = mj_id2name(b, mjOBJ_SITE, i);
        if (std::string(original ? original : "") != std::string(generated ? generated : ""))
          throw std::runtime_error("Generated mounting site name differs");
      }
      for (int i = 0; i < 500; ++i) {
        mj_step(a, baseline.data);
        mj_step(b, native.data);
      }
      if ((Eigen::Map<rm::nav::V3>(baseline.data->qpos) -
           Eigen::Map<rm::nav::V3>(native.data->qpos))
              .norm() > 1e-4)
        throw std::runtime_error("Generated contact settling differs");
    }
    std::cout << "Generated robot/navigation topology, mass, joints and settling match\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
