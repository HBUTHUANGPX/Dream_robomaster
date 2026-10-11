#include "rm/mechanical_sdf.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <numbers>
#include <stdexcept>
#include <string>

namespace rm::cube {
namespace {
constexpr mjtNum notch_ratio = .53;
constexpr const char* attributes[] = {"inner", "outer", "angle_deg", "notch"};
constexpr std::array<mjtNum, 4> defaults{.0146, .0162, 43, 0};

// 属性在缩窄转换前检查；不接受 NaN、Inf、溢出或数字后面的垃圾字符。
mjtNum number(const char* text, int index) {
  if (!text || !*text) return defaults[index];
  errno = 0;
  char* end = nullptr;
  const auto value = std::strtod(text, &end);
  if (end == text || errno == ERANGE || !std::isfinite(value))
    throw std::invalid_argument(std::string("无效属性：") + attributes[index]);
  while (*end && std::isspace(static_cast<unsigned char>(*end))) ++end;
  if (*end) throw std::invalid_argument(std::string("属性尾部含非数字：") + attributes[index]);
  return value;
}

MechanicalCapSdf field(const mjtNum* values) {
  if (values[3] != 0 && values[3] != 1) throw std::invalid_argument("notch 必须为 0 或 1");
  return MechanicalCapSdf(values[0], values[1], values[2], static_cast<int>(values[3]));
}

const MechanicalCapSdf& instance(const mjData* data, int id) {
  return *reinterpret_cast<const MechanicalCapSdf*>(data->plugin_data[id]);
}
}  // namespace

MechanicalCapSdf::MechanicalCapSdf(mjtNum inner, mjtNum outer, mjtNum angle_deg, int notch)
    : inner_(inner), outer_(outer), notch_(notch != 0) {
  if (!std::isfinite(inner) || !std::isfinite(outer) || !std::isfinite(angle_deg) || inner <= 0 ||
      outer <= inner || angle_deg <= 0 || angle_deg >= 90 || (notch != 0 && notch != 1))
    throw std::invalid_argument("卡脚要求 0 < inner < outer、0 < angle_deg < 90、notch 为 0/1");
  const auto angle = angle_deg * std::numbers::pi / 180;
  sin_angle_ = std::sin(angle);
  cos_angle_ = std::cos(angle);
}

MechanicalSdfSample MechanicalCapSdf::evaluate(const mjtNum p[3]) const {
  const auto r = std::hypot(p[0], p[1], p[2]);
  const std::array<mjtNum, 3> radial =
      r > 0 ? std::array<mjtNum, 3>{p[0] / r, p[1] / r, p[2] / r} : std::array<mjtNum, 3>{0, 0, 1};
  MechanicalSdfSample result{inner_ - r, {-radial[0], -radial[1], -radial[2]}};
  // 严格大于才替换，交界时按内球、外球、角度、X 缺口、Y 缺口确定次序。
  const auto intersect = [&](mjtNum distance, std::array<mjtNum, 3> gradient) {
    if (distance > result.distance) result = {distance, gradient};
  };
  intersect(r - outer_, radial);

  // 正向无限圆锥的距离；投影落在锥顶后方时最近点是原点，不能延伸成双锥。
  const auto rho = std::hypot(p[0], p[1]);
  if (rho * sin_angle_ + p[2] * cos_angle_ < 0) {
    intersect(r, radial);
  } else {
    const auto ux = rho > 0 ? p[0] / rho : 1;
    const auto uy = rho > 0 ? p[1] / rho : 0;
    intersect(rho * cos_angle_ - p[2] * sin_angle_,
              {cos_angle_ * ux, cos_angle_ * uy, -sin_angle_});
  }

  if (notch_) {
    // |x| <= k*r 等价于 sqrt(1-k*k)*|x| <= k*hypot(y,z)。
    // 使用锥面距离而非 |x|-k*r，使光滑区域的梯度具有单位尺度。
    const mjtNum c = std::sqrt(1 - notch_ratio * notch_ratio);
    for (int axis = 0; axis < 2; ++axis) {
      const int other = 1 - axis;
      const auto transverse = std::hypot(p[other], p[2]);
      std::array<mjtNum, 3> gradient{};
      gradient[axis] = p[axis] < 0 ? -c : c;
      gradient[other] = transverse > 0 ? -notch_ratio * p[other] / transverse : 0;
      gradient[2] = transverse > 0 ? -notch_ratio * p[2] / transverse : -notch_ratio;
      intersect(c * std::abs(p[axis]) - notch_ratio * transverse, gradient);
    }
  }
  return result;
}

std::array<mjtNum, 6> MechanicalCapSdf::aabb() const {
  const auto radius = outer_ * (notch_ ? std::min(sin_angle_, notch_ratio) : sin_angle_);
  const auto low = inner_ * cos_angle_, high = outer_;
  const auto padding = outer_ * 1e-6;
  return {0, 0, (low + high) / 2, radius + padding, radius + padding, (high - low) / 2 + padding};
}

void register_mechanical_sdf() {
  // API 与生命周期依据 MuJoCo 3.15.0 mjplugin.h 和官方 plugin/sdf/torus.cc。
  // https://github.com/google-deepmind/mujoco/blob/3.15.0/plugin/sdf/torus.cc
  static std::once_flag registered;
  std::call_once(registered, [] {
    if (mjp_getPlugin("rm.cube.cap", nullptr)) return;
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);
    plugin.name = "rm.cube.cap";
    plugin.capabilityflags = mjPLUGIN_SDF;
    plugin.nattribute = 4;
    plugin.attributes = attributes;
    plugin.nstate = +[](const mjModel*, int) { return 0; };
    plugin.init = +[](const mjModel* model, mjData* data, int id) {
      try {
        std::array<mjtNum, 4> values;
        for (int i = 0; i < 4; ++i)
          values[i] = number(mj_getPluginConfig(model, id, attributes[i]), i);
        auto* geometry = new MechanicalCapSdf(field(values.data()));
        data->plugin_data[id] = reinterpret_cast<std::uintptr_t>(geometry);
        return 0;
      } catch (const std::exception& e) {
        mju_warning("rm.cube.cap: %s", e.what());
        return -1;
      }
    };
    plugin.destroy = +[](mjData* data, int id) {
      delete reinterpret_cast<MechanicalCapSdf*>(data->plugin_data[id]);
      data->plugin_data[id] = 0;
    };
    // 参数不可变。mj_copyData 保留目标 mjData 在 init 中独立分配的 plugin_data，
    // 因此无需 copy 回调；reset 不改几何，也没有任何积分状态或力需要更新。
    plugin.reset = +[](const mjModel*, mjtNum*, void*, int) {};
    plugin.compute = +[](const mjModel*, mjData*, int, int) {};
    plugin.sdf_distance = +[](const mjtNum point[3], const mjData* data, int id) {
      return instance(data, id).evaluate(point).distance;
    };
    plugin.sdf_gradient =
        +[](mjtNum gradient[3], const mjtNum point[3], const mjData* data, int id) {
          const auto sample = instance(data, id).evaluate(point);
          std::copy(sample.gradient.begin(), sample.gradient.end(), gradient);
        };
    plugin.sdf_attribute = +[](mjtNum values[], const char* names[], const char* text[]) {
      try {
        std::copy(defaults.begin(), defaults.end(), values);
        for (int i = 0; i < 4; ++i) {
          if (!names[i]) continue;
          int index = 0;
          while (index < 4 && std::strcmp(names[i], attributes[index])) ++index;
          if (index == 4) throw std::invalid_argument("未知卡脚属性");
          values[index] = number(text[i], index);
        }
        (void)field(values);
      } catch (const std::exception& e) {
        mju_error("rm.cube.cap: %s", e.what());
      }
    };
    plugin.sdf_staticdistance = +[](const mjtNum point[3], const mjtNum* values) -> mjtNum {
      try {
        return field(values).evaluate(point).distance;
      } catch (const std::exception& e) {
        mju_error("rm.cube.cap: %s", e.what());
        return 0;
      }
    };
    plugin.sdf_aabb = +[](mjtNum aabb[6], const mjtNum* values) {
      try {
        const auto box = field(values).aabb();
        std::copy(box.begin(), box.end(), aabb);
      } catch (const std::exception& e) {
        mju_error("rm.cube.cap: %s", e.what());
      }
    };
    if (mjp_registerPlugin(&plugin) < 0) throw std::runtime_error("注册 rm.cube.cap 失败");
  });
}

}  // namespace rm::cube
