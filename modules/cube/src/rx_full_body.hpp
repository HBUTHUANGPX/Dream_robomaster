#pragma once
#include "rm/cube.hpp"
namespace rm::cube {
std::string rx_full_body_scene(const std::string& scene, const std::filesystem::path& bundle);
class RxFullBody {
 public:
  RxFullBody(mjModel* model, mjData* data, const std::filesystem::path& bundle,
             double centering_gain = 0);

  void reference(int hand, double angle, double dt);
  void control(mjData* data);
  const std::array<std::array<int, 2>, 2>& wrist_ranges() const { return wrist_ranges_; }
  double angle(int hand) const { return angles_[hand]; }
  double error(const mjData* data, int hand) const;
  double position_error(const mjData* data, int hand) const;
  Json report(const mjData* data) const;

 private:
  mjModel* m_;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> reference_;
  std::array<int, 2> hands_, sites_;
  std::array<std::array<int, 6>, 2> joints_;
  std::vector<int> driven_;
  std::vector<double> velocity_, limits_, velocity_limits_;
  std::array<std::array<int, 2>, 2> wrist_ranges_;
  std::vector<int> motor_type_;
  std::array<double, 2> angles_{};
  double peak_error_ = 0, centering_gain_ = 0;
  int core_ = -1;
  Vec center_offset_ = Vec::Zero();
  void solve(int hand, double angle);
};
}  // namespace rm::cube
