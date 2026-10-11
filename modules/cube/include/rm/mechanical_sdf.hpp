#pragma once

#include <mujoco/mjtype.h>

#include <array>

namespace rm::cube {

struct MechanicalSdfSample {
  mjtNum distance;
  std::array<mjtNum, 3> gradient;
};

// 实验几何：局部 +Z 球壳卡脚，单位米和度。
// inner > 0、outer > inner、0 < angle_deg < 90，notch 只能为 0 或 1。
// notch=1 额外要求 max(abs(x), abs(y)) <= .53 * norm(p)。
// 场值是各边界有符号距离的 max：正值在实体外，负值在实体内。
// 交界外侧不是精确欧氏距离；接缝、轴线和原点不可微，返回确定的有限单侧梯度。
// 连续曲面不等同于原三角楔体近似；仍需检查穿透、接触搜索收敛和保持能力。
// 本类不施加力，不增加约束，不修正位姿。查询点须为有限局部坐标。
class MechanicalCapSdf {
 public:
  MechanicalCapSdf(mjtNum inner = .0146, mjtNum outer = .0162, mjtNum angle_deg = 43,
                   int notch = 0);
  MechanicalSdfSample evaluate(const mjtNum point[3]) const;
  // MuJoCo 格式：中心 xyz，半尺寸 xyz；包含微小向外余量。
  std::array<mjtNum, 6> aabb() const;

 private:
  mjtNum inner_, outer_, sin_angle_, cos_angle_;
  bool notch_;
};

// 在加载引用 rm.cube.cap 的 MJCF 前调用。重复调用不重复注册。
// 属性顺序为 inner, outer, angle_deg, notch；默认值与上面的构造函数一致。
// 零积分状态，每个 mjData 持有独立且不变的几何参数。
void register_mechanical_sdf();

}  // namespace rm::cube
