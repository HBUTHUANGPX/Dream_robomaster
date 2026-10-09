#include "rm/solution_variants.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace rm::cube {
namespace {
using V = std::array<int, 3>;
constexpr std::array<char, 6> faces{'U', 'R', 'F', 'D', 'L', 'B'};
constexpr std::array<V, 6> normals{
    {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}, {0, -1, 0}, {-1, 0, 0}, {0, 0, -1}}};
constexpr std::array<V, 6> right{
    {{1, 0, 0}, {0, 0, -1}, {1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {-1, 0, 0}}};
constexpr std::array<V, 6> down{
    {{0, 0, 1}, {0, -1, 0}, {0, -1, 0}, {0, 0, -1}, {0, -1, 0}, {0, -1, 0}}};
int dot(V a, V b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V cross(V a, V b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
std::size_t face_index(char face) {
  auto it = std::find(faces.begin(), faces.end(), face);
  if (it == faces.end()) throw std::invalid_argument("面名称必须为 URFDLB");
  return static_cast<std::size_t>(it - faces.begin());
}
std::pair<std::size_t, int> parse(const std::string& move) {
  if (move.empty() || move.size() > 2 || (move.size() == 2 && move[1] != '2' && move[1] != '\''))
    throw std::invalid_argument("面转格式必须为面名称及可选后缀 2 或 '");
  return {face_index(move[0]), move.size() == 1 ? 1 : move[1] == '2' ? 2 : 3};
}
std::string format(std::size_t face, int power) {
  std::string move(1, faces[face]);
  if (power == 2) move += '2';
  if (power == 3) move += '\'';
  return move;
}
const CubeRotation& checked_rotation(std::size_t rotation) {
  if (rotation >= 24) throw std::invalid_argument("旋转索引必须在 0 到 23 之间");
  return cube_rotations()[rotation];
}
}  // namespace

const std::array<CubeRotation, 24>& cube_rotations() {
  static const auto rotations = [] {
    std::array<CubeRotation, 24> result{};
    constexpr std::array<V, 6> axes{
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-1, 0, 0}, {0, -1, 0}, {0, 0, -1}}};
    std::size_t index = 0;
    for (V x : axes)
      for (V y : axes) {
        if (dot(x, y) != 0) continue;
        const V z = cross(x, y);  // 右手正交基排除反射。
        const auto rotate = [&](V v) -> V {
          V out{};
          for (int a = 0; a < 3; ++a) out[a] = x[a] * v[0] + y[a] * v[1] + z[a] * v[2];
          return out;
        };
        auto& rotation = result[index++];
        for (std::size_t face = 0; face < 6; ++face) {
          const auto normal = rotate(normals[face]);
          const auto target = static_cast<std::size_t>(
              std::find(normals.begin(), normals.end(), normal) - normals.begin());
          rotation.face_destination[face] = faces[target];
          for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col) {
              V pos = normals[face];
              for (int a = 0; a < 3; ++a)
                pos[a] += (row - 1) * down[face][a] + (col - 1) * right[face][a];
              pos = rotate(pos);
              const int r = dot(pos, down[target]) + 1, c = dot(pos, right[target]) + 1;
              rotation.facelet_destination[9 * face + 3 * row + col] = 9 * target + 3 * r + c;
            }
        }
      }
    return result;
  }();
  return rotations;
}

std::string conjugate_facelets(const std::string& facelets, std::size_t rotation) {
  const auto& map = checked_rotation(rotation);
  if (facelets.size() != 54) throw std::invalid_argument("魔方面贴纸必须为 54 个字符");
  std::array<int, 6> counts{};
  std::string result(54, '?');
  for (std::size_t i = 0; i < facelets.size(); ++i) {
    const auto color = face_index(facelets[i]);
    ++counts[color];
    result[map.facelet_destination[i]] = map.face_destination[color];
  }
  for (std::size_t face = 0; face < 6; ++face)
    if (counts[face] != 9 || facelets[9 * face + 4] != faces[face])
      throw std::invalid_argument("魔方面贴纸数量或中心颜色非法");
  return result;
}

std::vector<std::string> map_moves_to_original(const std::vector<std::string>& moves,
                                               std::size_t rotation) {
  const auto& map = checked_rotation(rotation);
  std::vector<std::string> result;
  result.reserve(moves.size());
  for (const auto& move : moves) {
    const auto [face, power] = parse(move);
    const auto original = static_cast<std::size_t>(
        std::find(map.face_destination.begin(), map.face_destination.end(), faces[face]) -
        map.face_destination.begin());
    result.push_back(format(original, power));
  }
  return result;
}

}  // namespace rm::cube
