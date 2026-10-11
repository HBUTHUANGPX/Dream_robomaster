#pragma once

#include <mujoco/mujoco.h>

#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace mechanical_test {
// 空地址表仅表示完整的 guided 结构，不能用于掩盖损坏的旧模式。
inline std::vector<int> center_preload_addresses(const mjModel* m, std::string_view faces) {
  std::vector<int> addresses;
  int named_preloads = 0;
  for (int j = 0; j < m->njnt; ++j) {
    const char* name = mj_id2name(m, mjOBJ_JOINT, j);
    if (name && std::string_view(name).starts_with("preload_")) ++named_preloads;
  }
  for (char face : faces) {
    const int j = mj_name2id(m, mjOBJ_JOINT, ("preload_" + std::string(1, face)).c_str());
    if (j < 0) continue;
    if (m->jnt_type[j] != mjJNT_SLIDE)
      throw std::invalid_argument("Center preload must be a slide joint");
    addresses.push_back(m->jnt_qposadr[j]);
  }
  if (addresses.size() == 6 && named_preloads == 6) return addresses;
  if (!addresses.empty() || named_preloads)
    throw std::invalid_argument("Replay requires all six center preload joints or none");
  const int core = mj_name2id(m, mjOBJ_BODY, "mechanical_core");
  int balls = 0, hinges = 0;
  for (int b = 1; b < m->nbody; ++b) {
    const char* name = mj_id2name(m, mjOBJ_BODY, b);
    if (!name || !std::string_view(name).starts_with("piece_")) continue;
    if (core < 0 || m->body_parentid[b] != core || m->body_jntnum[b] != 1)
      throw std::invalid_argument("Guided pieces require one joint attached to the core");
    const int type = m->jnt_type[m->body_jntadr[b]];
    if (type == mjJNT_BALL)
      ++balls;
    else if (type == mjJNT_HINGE)
      ++hinges;
    else
      throw std::invalid_argument("Unsupported guided piece joint");
  }
  if (balls != 20 || hinges != 6)
    throw std::invalid_argument("Missing preloads require 20 guided ball pieces and six centers");
  for (char face : faces) {
    const int j = mj_name2id(m, mjOBJ_JOINT, ("hinge_" + std::string(1, face)).c_str());
    if (j < 0 || m->jnt_type[j] != mjJNT_HINGE || m->body_parentid[m->jnt_bodyid[j]] != core)
      throw std::invalid_argument("Guided model requires all six center hinges");
  }
  return addresses;
}
}  // namespace mechanical_test
