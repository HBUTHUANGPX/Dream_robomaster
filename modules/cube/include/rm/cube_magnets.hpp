#pragma once
#include <mujoco/mujoco.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace rm::cube {
// 块间短程、保守磁吸近似，无目标位姿或离散配对状态。
// GAN356 M 的48磁胶囊仅支持结构来源，力曲线和强度未经实测：
// https://www.gancube.cn/en/gan356m/
class CubeMagnets {
 public:
  static constexpr double range_m = .006;

  explicit CubeMagnets(const mjModel* m) : model_(m) {
    if (!m) throw std::invalid_argument("磁吸模型不能为空");
    for (int site = 0; site < m->nsite; ++site) {
      const char* name = mj_id2name(m, mjOBJ_SITE, site);
      if (!name) continue;
      if (std::string_view(name).starts_with("cube_corner_magnet_"))
        corners_.push_back(site);
      else if (std::string_view(name).starts_with("cube_edge_magnet_"))
        edges_.push_back(site);
    }
  }

  // 旧场景可以没有磁吸 site；启用磁吸的调用者应检查24个角site和24个棱site。
  std::size_t corner_count() const { return corners_.size(); }
  std::size_t edge_count() const { return edges_.size(); }

  // strength_n 是单对site的峰值吸力，不是整块或整层的合力。
  // 调用者每步统一清理 qfrc_applied；本函数只累加，不改变积分状态。
  // 刷新积分后的位姿与力雅可比所需量，不调用 mj_forward 或求解约束。
  void apply(const mjModel* m, mjData* d, double strength_n) const {
    validate(m, strength_n);
    if (strength_n == 0 || corners_.empty() || edges_.empty()) return;
    mj_kinematics(m, d);
    mj_comPos(m, d);
    evaluate(m, d, strength_n, d);
  }

  // 仅在 mj_step1 后、位姿未再修改时调用，复用已计算的运动学和力雅可比。
  void apply_prepared(const mjModel* m, mjData* d, double strength_n) const {
    validate(m, strength_n);
    if (strength_n == 0 || corners_.empty() || edges_.empty()) return;
    evaluate(m, d, strength_n, d);
  }

  // 单位J。只读当前 site_xpos；应在 apply 或位姿刷新之后调用。
  double potential(const mjModel* m, const mjData* d, double strength_n) const {
    validate(m, strength_n);
    if (strength_n == 0) return 0;
    return evaluate(m, d, strength_n, nullptr);
  }

 private:
  const mjModel* model_;
  std::vector<int> corners_, edges_;

  void validate(const mjModel* m, double strength_n) const {
    if (m != model_) throw std::invalid_argument("磁吸site必须使用构造时的模型");
    if (!std::isfinite(strength_n) || strength_n < 0)
      throw std::invalid_argument("磁吸强度必须是有限非负数");
  }

  double evaluate(const mjModel* m, const mjData* d, double strength_n, mjData* force_data) const {
    // F在r=R/sqrt(5)取峰值；U=-A*(1-r²/R²)³，r>=R时U和F均为零。
    const double amplitude = strength_n * range_m / (6 / std::sqrt(5.) * .8 * .8);
    double energy = 0;
    for (int corner : corners_)
      for (int edge : edges_) {
        const mjtNum* cp = d->site_xpos + 3 * corner;
        const mjtNum* ep = d->site_xpos + 3 * edge;
        mjtNum delta[3];
        mju_sub3(delta, ep, cp);
        const double r2 = mju_dot3(delta, delta);
        if (r2 >= range_m * range_m) continue;
        const double taper = 1 - r2 / (range_m * range_m);
        energy -= amplitude * taper * taper * taper;
        if (!force_data) continue;
        mjtNum force[3];
        mju_scl3(force, delta, 6 * amplitude / (range_m * range_m) * taper * taper);
        mj_applyFT(m, force_data, force, nullptr, cp, m->site_bodyid[corner],
                   force_data->qfrc_applied);
        mju_scl3(force, force, -1);
        mj_applyFT(m, force_data, force, nullptr, ep, m->site_bodyid[edge],
                   force_data->qfrc_applied);
      }
    return energy;
  }
};
}  // namespace rm::cube
