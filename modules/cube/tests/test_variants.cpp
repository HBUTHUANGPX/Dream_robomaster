#include <algorithm>
#include <array>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>

#include "rm/solution_variants.hpp"

using namespace rm::cube;
namespace {
using Moves = std::vector<std::string>;
const std::string faces = "URFDLB";
const std::string solved = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
// 独立面转置换环：不调用被测工具的几何、贴纸映射或解析函数。
const std::array<std::array<int, 12>, 6> rings{{{18, 19, 20, 36, 37, 38, 45, 46, 47, 9, 10, 11},
                                                {2, 5, 8, 51, 48, 45, 29, 32, 35, 20, 23, 26},
                                                {6, 7, 8, 9, 12, 15, 29, 28, 27, 44, 41, 38},
                                                {24, 25, 26, 15, 16, 17, 51, 52, 53, 42, 43, 44},
                                                {0, 3, 6, 18, 21, 24, 27, 30, 33, 53, 50, 47},
                                                {0, 1, 2, 42, 39, 36, 35, 34, 33, 11, 14, 17}}};
std::string replay(std::string state, const Moves& moves) {
  for (const auto& move : moves) {
    require(!move.empty() && faces.find(move[0]) != std::string::npos, "测试面名称非法");
    require(move.size() == 1 || (move.size() == 2 && (move[1] == '2' || move[1] == '\'')),
            "测试转动格式非法");
    const auto face = faces.find(move[0]);
    const int power = move.size() == 1 ? 1 : move[1] == '2' ? 2 : 3;
    for (int turn = 0; turn < power; ++turn) {
      auto next = state;
      for (int i = 0; i < 12; ++i) next[rings[face][(i + 3) % 12]] = state[rings[face][i]];
      for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
          next[9 * face + 3 * col + 2 - row] = state[9 * face + 3 * row + col];
      state = std::move(next);
    }
  }
  return state;
}
Moves inverse(const Moves& moves) {
  Moves result;
  for (auto it = moves.rbegin(); it != moves.rend(); ++it)
    result.push_back(it->size() == 1 ? *it + "'" : (*it)[1] == '2' ? *it : it->substr(0, 1));
  return result;
}
Moves forward(Moves moves, std::size_t rotation) {
  for (auto& move : moves)
    move[0] = cube_rotations()[rotation].face_destination[faces.find(move[0])];
  return moves;
}
template <class F>
void rejects(F&& fn) {
  bool rejected = false;
  try {
    fn();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "非法输入未抛出 invalid_argument");
}
}  // namespace

int main() {
  try {
    const auto& rotations = cube_rotations();
    require(rotations.size() == 24, "旋转数量错误");
    std::set<std::array<char, 6>> unique_faces;
    std::set<std::array<std::size_t, 54>> unique_stickers;
    for (std::size_t r = 0; r < rotations.size(); ++r) {
      const auto& rotation = rotations[r];
      unique_faces.insert(rotation.face_destination);
      unique_stickers.insert(rotation.facelet_destination);
      auto permutation = rotation.facelet_destination;
      std::sort(permutation.begin(), permutation.end());
      for (std::size_t i = 0; i < 54; ++i) require(permutation[i] == i, "贴纸映射不是双射");
      require(conjugate_facelets(solved, r) == solved, "颜色重标定未保持已还原状态");
      require(map_moves_to_original({}, r).empty(), "空动作映射错误");
    }
    require(unique_faces.size() == 24 && unique_stickers.size() == 24, "存在重复旋转");
    for (std::size_t i = 0; i < 6; ++i)
      require(rotations[0].face_destination[i] == faces[i], "首项不是恒等旋转");
    for (std::size_t i = 0; i < 54; ++i)
      require(rotations[0].facelet_destination[i] == i, "恒等旋转移动了贴纸");
    // 组合闭包和逆元排除不完整旋转集合。
    for (const auto& a : rotations) {
      bool found_inverse = false;
      for (const auto& b : rotations) {
        CubeRotation composed{};
        for (std::size_t i = 0; i < 6; ++i)
          composed.face_destination[i] = b.face_destination[faces.find(a.face_destination[i])];
        for (std::size_t i = 0; i < 54; ++i)
          composed.facelet_destination[i] = b.facelet_destination[a.facelet_destination[i]];
        require(std::find(rotations.begin(), rotations.end(), composed) != rotations.end(),
                "旋转组合不封闭");
        found_inverse |= composed == rotations[0];
      }
      require(found_inverse, "旋转缺少逆元");
    }
    std::mt19937 random(20261009);
    std::size_t checked = 0;
    for (int sample = 0; sample < 32; ++sample) {
      Moves scramble;
      for (int i = 0; i < 20; ++i) {
        std::string move(1, faces[random() % 6]);
        const auto power = random() % 3;
        if (power == 1) move += '2';
        if (power == 2) move += '\'';
        scramble.push_back(move);
      }
      const auto state = replay(solved, scramble);
      require(conjugate_facelets(state, 0) == state, "恒等共轭改变状态");
      for (std::size_t r = 0; r < 24; ++r) {
        const auto rotated = conjugate_facelets(state, r);
        require(rotated == replay(solved, forward(scramble, r)), "共轭与独立转动不一致");
        const auto rotated_solution = forward(inverse(scramble), r);
        require(replay(rotated, rotated_solution) == solved, "旋转状态解未还原");
        const auto original_solution = map_moves_to_original(rotated_solution, r);
        require(replay(state, original_solution) == solved, "映射回原坐标的解未还原");
        // 所有基本面转在共轭前后必须一致；反射会颠倒四分之一转。
        for (char face : faces) {
          const Moves move{std::string(1, face)};
          require(conjugate_facelets(replay(state, move), r) == replay(rotated, forward(move, r)),
                  "共轭不保持顺时针方向");
        }
        ++checked;
      }
    }
    for (const auto& bad : {"", "X", "R3", "RR", "U2'", "r", "R "}) {
      rejects([&] { map_moves_to_original({bad}, 0); });
    }
    rejects([&] { conjugate_facelets("", 0); });
    rejects([&] { conjugate_facelets(std::string(54, 'X'), 0); });
    rejects([&] { conjugate_facelets(std::string(54, 'U'), 0); });
    auto bad_center = solved;
    std::swap(bad_center[4], bad_center[13]);
    rejects([&] { conjugate_facelets(bad_center, 0); });
    rejects([&] { conjugate_facelets(solved, 24); });
    rejects([&] { map_moves_to_original({}, 24); });
    std::cout << "旋转共轭测试通过：24 姿态、" << checked
              << " 组随机状态映射还原、独立贴纸重放与非法输入。\n";
  } catch (const std::exception& error) {
    std::cerr << "失败：" << error.what() << '\n';
    return 1;
  }
}
