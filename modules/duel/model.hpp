#pragma once
#include <mujoco/mujoco.h>

#include <filesystem>
namespace rm::duel {
mjModel* build_model(const std::filesystem::path& root);
}
