#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "rm/torque_speed.hpp"

namespace {
constexpr double index_scale = (60. / (2. * std::numbers::pi)) / .05;
void need(bool ok, const char* text) {
  if (!ok) throw std::runtime_error(text);
}
void near(double actual, double expected, const char* text) {
  need(std::isfinite(actual) && std::abs(actual - expected) < 2e-12, text);
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::invalid_argument("用法: torque_speed_tests 仓库根目录");
    std::ifstream in(std::filesystem::path(argv[1]) / "assets/rx_gripper/tn/s_motor_reference.csv");
    need(bool(in), "无法读取独立 Python 参考表");
    std::string line;
    std::getline(in, line);
    std::vector<double> reference;
    while (std::getline(in, line)) {
      const auto comma = line.find(',');
      need(comma != std::string::npos, "参考表行格式错误");
      need(std::stoul(line.substr(0, comma)) == reference.size(), "参考表索引不连续");
      reference.push_back(std::stod(line.substr(comma + 1)));
    }
    need(reference.size() == 6194 && reference.back() == 0, "公共网格或 sentinel 错误");
    using rm::cube::rx_s_torque_limit;
    near(rx_s_torque_limit(0), 21.31, "零速不得使用静态1.2倍率或15Nm外部上限");
    near(rx_s_torque_limit(-0.), 21.31, "负零错误");
    near(rx_s_torque_limit(5. / (60. / (2. * std::numbers::pi))), 21.31, "低速锚点错误");
    // 每个格的内部速度必须选择上格；逐格与原始 Python PCHIP 结果比较。
    double previous = rx_s_torque_limit(0);
    for (size_t i = 1; i < reference.size(); ++i) {
      const double v = (double(i) - .25) / index_scale;
      const double limit = rx_s_torque_limit(v);
      near(limit, reference[i], "完整参考表不一致：插值、包络或上取整错误");
      near(rx_s_torque_limit(-v), limit, "正反转限矩不对称");
      need(limit <= previous + 1e-13, "限矩未单调化");
      previous = limit;
    }
    near(rx_s_torque_limit(170. / (60. / (2. * std::numbers::pi))), 20.54,
         "测量噪声回升未被累积最小值抑制");
    // 浮点格边界采用与参考实现相同的 ceil 语义，不加 epsilon 降档。
    for (double bin : {600., 800., 5600., 6000., 6192.}) {
      const double boundary = bin / index_scale;
      for (double v : {std::nextafter(boundary, 0.), boundary,
                       std::nextafter(boundary, std::numeric_limits<double>::infinity())}) {
        const auto index = static_cast<size_t>(std::ceil(v * index_scale));
        near(rx_s_torque_limit(v), reference.at(index), "交叉段或测量边界格选择错误");
      }
    }
    near(rx_s_torque_limit(6192. / index_scale), 19.12, "公共测量终点应保留实测扭矩");
    near(rx_s_torque_limit(6192.01 / index_scale), 0, "超测量范围应选择零 sentinel");
    near(rx_s_torque_limit(310.3 / (60. / (2. * std::numbers::pi))), 0,
         "不得使用第二台电机独有的额外测量范围");
    for (double v : {1e100, -1e100, std::numeric_limits<double>::max()})
      near(rx_s_torque_limit(v), 0, "巨大有限速度应安全归零，不能溢出索引");
    for (double v :
         {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()}) {
      bool rejected = false;
      try {
        rx_s_torque_limit(v);
      } catch (const std::invalid_argument&) {
        rejected = true;
      }
      need(rejected, "非有限速度必须报错");
    }
    std::ifstream fin(std::filesystem::path(argv[1]) /
                      "assets/rx_gripper/tn/f_motor_reference.csv");
    need(bool(fin), "无法读取F电机独立参考表");
    std::getline(fin, line);
    size_t fi = 0;
    while (std::getline(fin, line)) {
      auto comma = line.find(',');
      need(std::stoul(line.substr(0, comma)) == fi, "F参考索引不连续");
      double v = fi ? (double(fi) - .25) / index_scale : 0;
      near(rm::cube::rx_f_torque_limit(v), std::stod(line.substr(comma + 1)),
           "F电机完整参考表不一致");
      near(rm::cube::rx_f_torque_limit(-v), rm::cube::rx_f_torque_limit(v), "F反转不对称");
      ++fi;
    }
    need(fi == 3800, "F参考表长度错误");
    near(rm::cube::rx_f_torque_limit(1e100), 0, "F超速未归零");
    bool f_rejected = false;
    try {
      rm::cube::rx_f_torque_limit(std::numeric_limits<double>::quiet_NaN());
    } catch (const std::invalid_argument&) {
      f_rejected = true;
    }
    need(f_rejected, "F非有限速度未拒绝");
    std::cout << "F/S电机TN：6193格参考下包络、上取整、边界和异常检查通过\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
