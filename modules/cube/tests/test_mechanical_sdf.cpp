#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>

#include "rm/mechanical_sdf.hpp"

namespace {
using namespace rm::cube;
using Point = std::array<mjtNum, 3>;

void need(bool value, const char* text) {
  if (!value) throw std::runtime_error(text);
}
void near(mjtNum actual, mjtNum expected, mjtNum tolerance, const char* text) {
  need(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, text);
}
Point polar(mjtNum radius, mjtNum degrees, mjtNum azimuth = 0) {
  const auto t = degrees * std::numbers::pi / 180;
  const auto p = azimuth * std::numbers::pi / 180;
  return {radius * std::sin(t) * std::cos(p), radius * std::sin(t) * std::sin(p),
          radius * std::cos(t)};
}
void difference(const MechanicalCapSdf& field, Point point) {
  const auto sample = field.evaluate(point.data());
  constexpr mjtNum h = 1e-8;
  for (int axis = 0; axis < 3; ++axis) {
    auto lo = point, hi = point;
    lo[axis] -= h;
    hi[axis] += h;
    const auto numerical =
        (field.evaluate(hi.data()).distance - field.evaluate(lo.data()).distance) / (2 * h);
    near(sample.gradient[axis], numerical, 2e-6, "非交界点梯度与有限差分不符");
  }
}

void fields() {
  MechanicalCapSdf cap;
  near(cap.evaluate(Point{0, 0, .014}.data()).distance, .0006, 1e-12, "内腔符号错误");
  near(cap.evaluate(Point{0, 0, .017}.data()).distance, .0008, 1e-12, "外球面符号错误");
  need(cap.evaluate(Point{0, 0, .015}.data()).distance < 0, "球壳内应为负值");
  near(cap.evaluate(Point{0, 0, .0146}.data()).distance, 0, 1e-12, "内球面不是零值");
  near(cap.evaluate(Point{0, 0, .0162}.data()).distance, 0, 1e-12, "外球面不是零值");
  near(cap.evaluate(polar(.0154, 43).data()).distance, 0, 1e-12, "角度边界不是零值");
  need(cap.evaluate(polar(.0154, 42).data()).distance < 0, "角度内侧符号错误");
  need(cap.evaluate(polar(.0154, 45).data()).distance > 0, "角度外侧符号错误");
  need(cap.evaluate(Point{0, 0, -.0154}.data()).distance > 0, "负 Z 误成第二个卡脚");

  MechanicalCapSdf notched(.0146, .0162, 43, 1);
  const auto axis_cut = polar(.0154, 40);
  need(cap.evaluate(axis_cut.data()).distance < 0, "无缺口卡脚不应移除该点");
  need(notched.evaluate(axis_cut.data()).distance > 0, "轴向缺口没有切开");
  need(notched.evaluate(polar(.0154, 40, 45).data()).distance < 0, "对角承托翼被切掉");
  const auto boundary = polar(.0154, std::asin(.53) * 180 / std::numbers::pi);
  near(notched.evaluate(boundary.data()).distance, 0, 1e-12, "缺口边界不是零值");

  // 覆盖每个可能主导的边界，并避开 max 切换点和绝对值接缝。
  difference(cap, polar(.014, 12, 23));
  difference(cap, polar(.017, 12, 23));
  difference(cap, polar(.0154, 45, 23));
  difference(cap, Point{.001, .002, -.02});
  for (const auto azimuth : {10., 80., 170., 260.}) difference(notched, polar(.0154, 40, azimuth));

  for (const auto* field : {&cap, &notched}) {
    for (const auto point :
         {Point{0, 0, 0}, Point{0, 0, .0154}, Point{0, 0, -.0154}, polar(.0154, 40, 45)}) {
      const auto s = field->evaluate(point.data());
      need(std::isfinite(s.distance), "奇异点场值不是有限数");
      for (const auto g : s.gradient) need(std::isfinite(g), "奇异点梯度不是有限数");
    }
    near(field->evaluate(Point{}.data()).distance, .0146, 1e-12, "原点没有保留内腔");
    const auto box = field->aabb();
    for (int t = 0; t <= 43; ++t)
      for (int p = 0; p < 360; p += 5)
        for (mjtNum r : {.0146, .0154, .0162}) {
          const auto point = polar(r, t, p);
          if (field->evaluate(point.data()).distance > 1e-12) continue;
          for (int i = 0; i < 3; ++i)
            need(std::abs(point[i] - box[i]) <= box[i + 3], "AABB 遗漏实体边界");
        }
  }
  for (const auto bad : {std::array<mjtNum, 4>{0, .0162, 43, 0},
                         {.017, .0162, 43, 0},
                         {.0146, .0162, 0, 0},
                         {.0146, .0162, 90, 0},
                         {.0146, .0162, 43, 2},
                         {.0146, std::numeric_limits<mjtNum>::infinity(), 43, 0},
                         {.0146, .0162, std::numeric_limits<mjtNum>::quiet_NaN(), 0}}) {
    bool rejected = false;
    try {
      MechanicalCapSdf invalid(bad[0], bad[1], bad[2], static_cast<int>(bad[3]));
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    need(rejected, "非法几何参数未拒绝");
  }
}

void plugin() {
  register_mechanical_sdf();
  const int count = mjp_pluginCount();
  register_mechanical_sdf();
  need(mjp_pluginCount() == count, "重复注册插件");
  const auto* p = mjp_getPlugin("rm.cube.cap", nullptr);
  need(p && p->capabilityflags == mjPLUGIN_SDF && p->nattribute == 4, "插件描述错误");
  const char* names[] = {"inner", "outer", "angle_deg", "notch"};
  const char* values[] = {" .0146 ", ".0162", "43", "1"};
  mjtNum attributes[4], box[6], gradient[3];
  p->sdf_attribute(attributes, names, values);
  p->sdf_aabb(box, attributes);
  MechanicalCapSdf expected(.0146, .0162, 43, 1);
  const auto expected_box = expected.aabb();
  for (int i = 0; i < 6; ++i) near(box[i], expected_box[i], 1e-12, "静态 AABB 不一致");
  const auto point = polar(.0154, 40, 10);
  near(p->sdf_staticdistance(point.data(), attributes), expected.evaluate(point.data()).distance,
       1e-12, "静态场不一致");
  const char* omitted[] = {"", "", "", ""};
  p->sdf_attribute(attributes, names, omitted);
  near(attributes[0], .0146, 1e-12, "默认内径错误");
  near(attributes[3], 0, 0, "默认缺口开关错误");

  // 仅编译一个小几何验证回调和所有权；不运行物理步进或渲染。
  const char* xml = R"(<mujoco>
    <extension><plugin plugin="rm.cube.cap"><instance name="cap">
      <config key="inner" value=".0146"/><config key="outer" value=".0162"/>
      <config key="angle_deg" value="43"/><config key="notch" value="1"/>
    </instance></plugin></extension>
    <asset><mesh name="cap_mesh"><plugin instance="cap"/></mesh></asset>
    <worldbody><geom type="sdf" mesh="cap_mesh"><plugin instance="cap"/></geom></worldbody>
  </mujoco>)";
  char error[1024]{};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXMLString(xml, nullptr, error, sizeof(error)), mj_deleteSpec);
  need(bool(spec), error);
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_compile(spec.get(), nullptr),
                                                            mj_deleteModel);
  need(bool(model), mjs_getError(spec.get()));
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  need(bool(data) && model->nplugin == 1 && model->npluginstate == 0, "插件初始化失败");
  const auto address = data->plugin_data[0];
  need(address != 0, "实例几何未分配");
  near(p->sdf_distance(point.data(), data.get(), 0), expected.evaluate(point.data()).distance,
       1e-12, "运行时场与静态场不一致");
  p->sdf_gradient(gradient, point.data(), data.get(), 0);
  for (int i = 0; i < 3; ++i)
    near(gradient[i], expected.evaluate(point.data()).gradient[i], 1e-12, "回调梯度错误");
  mj_resetData(model.get(), data.get());
  need(data->plugin_data[0] == address, "重置破坏实例参数");
  std::unique_ptr<mjData, decltype(&mj_deleteData)> copy(
      mj_copyData(nullptr, model.get(), data.get()), mj_deleteData);
  need(bool(copy) && copy->plugin_data[0] != address, "复制实例共享了待释放指针");
  data.reset();
  near(p->sdf_distance(point.data(), copy.get(), 0), expected.evaluate(point.data()).distance,
       1e-12, "释放原实例使副本失效");
}
}  // namespace

int main() {
  try {
    fields();
    plugin();
    std::cout << "机械卡脚 SDF 字段与插件测试通过。\n";
  } catch (const std::exception& e) {
    std::cerr << "失败：" << e.what() << '\n';
    return 1;
  }
}
