#pragma once
#include "rm/cube.hpp"
namespace rm::cube {
std::string mechanical_rx_scene(const std::string& xml, const std::filesystem::path& bundle,
                                double bearing_friction, double tip_friction);
}
