#pragma once
#include "rm/navigation.hpp"
namespace rm::nav {
// Uncompressed ROS bag v2, standard ROS1 sensor_msgs wire encoding.
class RawBag {
 public:
  explicit RawBag(const std::filesystem::path& path);
  ~RawBag();
  void imu(const Json& packet);
  void cloud(const RawCloud& packet);
  void close();
  std::array<uint32_t, 2> count{};  // lidar, imu
 private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};
}  // namespace rm::nav
