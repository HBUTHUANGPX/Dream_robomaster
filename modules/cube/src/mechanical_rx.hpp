#pragma once
#include "rm/cube.hpp"
namespace rm::cube {
std::string mechanical_rx_scene(const std::string& xml, const std::filesystem::path& bundle,
                                double bearing_friction, double tip_friction,
                                double tip_contact_time, double cube_mass,
                                bool corner_spheres = false, double sphere_radius = .003,
                                double sphere_offset = .005, double tip_torsional_friction = .002);
}
