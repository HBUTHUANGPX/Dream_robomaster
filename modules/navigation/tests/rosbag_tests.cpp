#include <fstream>
#include <iostream>

#include "rm/rosbag.hpp"
int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("output path required");
    std::filesystem::path path = argv[1];
    std::filesystem::remove(path);
    rm::nav::RawBag bag(path);
    bag.imu({{"stamp", 1.005}, {"angular_velocity", {.1, .2, .3}}, {"acceleration", {0, 0, 9.81}}});
    bool invalid_stamp = false;
    try {
      bag.imu({{"stamp", 1e300}, {"angular_velocity", {0, 0, 0}}, {"acceleration", {0, 0, 9.81}}});
    } catch (const std::invalid_argument&) {
      invalid_stamp = true;
    }
    if (!invalid_stamp || bag.count[1] != 1)
      throw std::runtime_error("Unrepresentable ROS timestamp accepted");
    bool invalid_point = false;
    try {
      bag.cloud({1.1,
                 "mid360_top",
                 {rm::nav::V3(1e300, 0, 0)},
                 rm::nav::M3::Identity(),
                 rm::nav::V3::Zero()});
    } catch (const std::invalid_argument&) {
      invalid_point = true;
    }
    if (!invalid_point || bag.count[0] != 0)
      throw std::runtime_error("Unrepresentable ROS point accepted");
    bag.cloud({1.1,
               "mid360_top",
               {rm::nav::V3(1, 2, 3), rm::nav::V3(4, 5, 6)},
               rm::nav::M3::Identity(),
               rm::nav::V3::Zero()});
    bag.close();
    if (bag.count != std::array<uint32_t, 2>{1, 1})
      throw std::runtime_error("Incorrect bag counts");
    std::ifstream in(path, std::ios::binary);
    std::string magic(13, '\0');
    in.read(magic.data(), 13);
    if (magic != "#ROSBAG V2.0\n" || std::filesystem::file_size(path) < 5000)
      throw std::runtime_error("Invalid bag container");
    bool rejected = false;
    try {
      rm::nav::RawBag duplicate(path);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    if (!rejected) throw std::runtime_error("Existing bag overwritten");
    auto multi_path = std::filesystem::path(path.string() + ".multi.bag");
    std::filesystem::remove(multi_path);
    rm::nav::RawBag multi(multi_path);
    rm::nav::RawCloud cloud{2, "mid360_top", rm::nav::Points(1000, rm::nav::V3(1, 2, 3)),
                            rm::nav::M3::Identity(), rm::nav::V3::Zero()};
    for (int i = 0; i < 80; ++i) {
      cloud.stamp = 2 + i * .1;
      multi.cloud(cloud);
    }
    multi.close();
    if (multi.count[0] != 80 || std::filesystem::file_size(multi_path) <= 1024 * 1024)
      throw std::runtime_error("Multiple bag chunks missing");
    std::cout << "ROS1 bag fixture saved " << path << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
