#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace rm::cube {
struct CubeRotation {
  // 均按 URFDLB 编号；原面映射到旋转后的面，原贴纸映射到目标位置。
  std::array<char, 6> face_destination;
  std::array<std::size_t, 54> facelet_destination;
  bool operator==(const CubeRotation&) const = default;
};

// 仅含行列式为 +1 的 24 种旋转，索引 0 为恒等旋转，顺序固定。
const std::array<CubeRotation, 24>& cube_rotations();

// rotation 为上述数组索引。同时旋转位置和重标定颜色，保持中心 URFDLB。
// 检查长度、字符、颜色数量和中心；不检查魔方物理可解性。
// 非法面贴纸或旋转索引抛出 std::invalid_argument。
std::string conjugate_facelets(const std::string& facelets, std::size_t rotation);

// 将旋转后坐标中的解映射回原坐标；proper rotation 不改变转动次数。
// 仅接受 U/R/F/D/L/B 及可选后缀 2 或 '，非法输入抛出 std::invalid_argument。
std::vector<std::string> map_moves_to_original(const std::vector<std::string>& moves,
                                               std::size_t rotation);

}  // namespace rm::cube
