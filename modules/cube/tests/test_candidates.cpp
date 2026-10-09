#include "rm/solution_candidates.hpp"

#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

extern "C" {
#include "search.h"
}

using namespace rm::cube;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace {
const std::string solved = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
const std::string order = "URFDLB";
using V = std::array<int, 3>;
const std::array<V, 6> normals{{{0,1,0}, {1,0,0}, {0,0,1}, {0,-1,0}, {-1,0,0}, {0,0,-1}}};
const std::array<V, 6> right{{{1,0,0}, {0,0,-1}, {1,0,0}, {1,0,0}, {0,0,1}, {-1,0,0}}};
const std::array<V, 6> down{{{0,0,1}, {0,-1,0}, {0,-1,0}, {0,0,-1}, {0,-1,0}, {0,-1,0}}};
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
int dot(V a, V b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
V turn(V v, V n) {
  V cross{n[1]*v[2]-n[2]*v[1], n[2]*v[0]-n[0]*v[2], n[0]*v[1]-n[1]*v[0]};
  const int d = dot(n, v);
  for (int i = 0; i < 3; ++i) v[i] = n[i]*d - cross[i];
  return v;
}
// 独立整数贴纸几何，不使用 Kociemba 的 cubie 或移动表。
std::string replay(std::string state, const std::vector<std::string>& moves) {
  for (const auto& move : moves) {
    require(!move.empty() && order.find(move[0]) != std::string::npos, "候选面名称错误");
    require(move.size() == 1 || (move.size() == 2 && (move[1] == '2' || move[1] == '\'')),
            "候选转动格式错误");
    const V axis = normals[order.find(move[0])];
    const int count = move.size() == 1 ? 1 : move[1] == '2' ? 2 : 3;
    for (int t = 0; t < count; ++t) {
      std::string next(54, '?');
      for (int f = 0; f < 6; ++f)
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c) {
            V p = normals[f], n = normals[f];
            for (int a = 0; a < 3; ++a) p[a] += (r-1)*down[f][a] + (c-1)*right[f][a];
            if (dot(p, axis) == 1) { p = turn(p, axis); n = turn(n, axis); }
            int target = 0;
            while (normals[target] != n) ++target;
            next[9*target + 3*(dot(p, down[target])+1) + dot(p, right[target])+1] = state[9*f+3*r+c];
          }
      state = next;
    }
  }
  return state;
}
template<class F> void rejects(F&& fn) {
  bool rejected = false;
  try { fn(); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "非法输入未被拒绝");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 2, "需要仓库根目录参数");
    const std::filesystem::path root = argv[1];
    const auto scramble = replay(solved, {"R", "U", "F'", "L2", "D", "B", "R'", "U2", "F", "D'"});
    size_t calls = 0;
    const auto noop = [&](const auto&) { ++calls; return true; };
    auto stats = enumerate_solutions(scramble, root, Clock::now(), noop);
    require(stats.timed_out && stats.candidates == 0 && calls == 0, "零预算仍交付候选");
    auto start = Clock::now();
    stats = enumerate_solutions(scramble, root, start + 1us, noop);
    require(stats.timed_out && Clock::now() - start < 100ms, "极小预算未及时返回");
    start = Clock::now();
    stats = enumerate_solutions(scramble, root, start + 1ms, noop);
    require(stats.timed_out && Clock::now() - start < 100ms, "表加载预算未及时截止");
    const auto run = [&](const std::string& state, size_t limit) {
      std::set<std::vector<std::string>> seen;
      auto result = enumerate_solutions(state, root, Clock::now() + 3s, [&](const auto& moves) {
        require(replay(state, moves) == solved, "候选独立重放未还原");
        require(seen.insert(moves).second, "重复交付候选");
        return seen.size() < limit;
      });
      require(result.candidates == limit && seen.size() == limit && !result.timed_out,
              "候选数量或停止语义错误");
    };
    const auto short_state = replay(solved, {"R", "U", "F'", "L2"});
    const std::vector<std::string> inverse{"L2", "F", "U'", "R'"};
    bool found_inverse = false;
    start = Clock::now();
    stats = enumerate_solutions(short_state, root, start + 1500ms, [&](const auto& moves) {
      require(replay(short_state, moves) == solved, "短打乱候选独立重放失败");
      found_inverse = moves == inverse;
      return !found_inverse;
    });
    require(found_inverse && !stats.timed_out && stats.candidates >= 1,
            "枚举困在单个第二阶段子树，未找到四步逆序解");
    std::cout << "短打乱四步解：候选 " << stats.candidates << "，耗时 "
              << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count()
              << " 毫秒。\n";
    // 连续请求仍可获得多个不同候选。
    run(short_state, 5);
    run(scramble, 5);
    run(replay(solved, {"R"}), 4);
    run(replay(solved, {"U2", "F", "B2", "D'"}), 3);
    run(scramble, 1);
    // 固定种子的长打乱同时覆盖不同第一阶段路径，失败可复现。
    unsigned seed = 20261009;
    for (int sample = 0; sample < 8; ++sample) {
      std::vector<std::string> scramble_moves;
      for (int i = 0; i < 20; ++i) {
        seed = seed * 1664525U + 1013904223U;
        std::string move(1, order[(seed >> 16) % 6]);
        const auto power = (seed >> 24) % 3;
        if (power == 1) move += '2';
        if (power == 2) move += '\'';
        scramble_moves.push_back(move);
      }
      run(replay(solved, scramble_moves), 2);
    }
    stats = enumerate_solutions(solved, root / "不存在", Clock::now() + 1s, [&](const auto& moves) {
      require(moves.empty(), "已还原状态未返回空序列"); return true;
    });
    require(stats.candidates == 1 && !stats.timed_out, "已还原状态统计错误");
    for (const auto& bad : {std::string{}, std::string(54, 'X')})
      rejects([&] { enumerate_solutions(bad, root, Clock::now() + 1s, noop); });
    auto bad = solved;
    std::swap(bad[5], bad[10]);
    rejects([&] { enumerate_solutions(bad, root, Clock::now() + 1s, noop); });
    bad = solved;
    std::swap(bad[4], bad[13]);
    rejects([&] { enumerate_solutions(bad, root, Clock::now() + 1s, noop); });
    bool threw = false;
    try {
      enumerate_solutions(scramble, root, Clock::now() + 1s, [](const auto&) -> bool {
        throw std::logic_error("回调失败");
      });
    } catch (const std::logic_error&) { threw = true; }
    require(threw, "回调异常未传播");
    run(scramble, 2);
    const auto deadline = Clock::now() + 100ms;
    stats = enumerate_solutions(scramble, root, deadline, [&](const auto& moves) {
      require(replay(scramble, moves) == solved, "超时前候选无效");
      std::this_thread::sleep_until(deadline + 5ms);
      return false;
    });
    require(stats.timed_out && stats.candidates == 1, "回调耗时未计入预算");
    start = Clock::now();
    stats = enumerate_solutions(short_state, root, start + 50ms, [&](const auto& moves) {
      require(replay(short_state, moves) == solved, "持续枚举候选无效"); return true;
    });
    require(stats.timed_out && stats.candidates > 1 && Clock::now() - start < 300ms,
            "持续枚举未及时截止");
    // 原入口仍可返回可重放的解。
    std::string input = scramble;
    const auto cache = (root / "modules/cube/third_party/kociemba/cprunetables").string();
    char* answer = solution(input.data(), 24, 30, 0, cache.c_str());
    require(answer != nullptr, "原 solution 入口失败");
    std::istringstream words(answer);
    std::free(answer);
    std::vector<std::string> moves;
    for (std::string word; words >> word;) moves.push_back(word);
    require(replay(scramble, moves) == solved, "原 solution 解失效");
    require(solution(input.data(), 1, 30, 0, cache.c_str()) == nullptr,
            "原入口深度限制失效");
    bad = solved;
    std::swap(bad[5], bad[10]);
    require(solution(bad.data(), 24, 30, 0, cache.c_str()) == nullptr,
            "原入口接受不可能状态");
    // 缺表和同长度损坏表即使全局表已加载也必须报错，不允许隐式生成。
    const auto scratch = root / "output/cube-candidate-tests";
    const auto tables = scratch / "modules/cube/third_party/kociemba/cprunetables";
    std::filesystem::remove_all(scratch);
    bool table_error = false;
    try { enumerate_solutions(scramble, scratch, Clock::now() + 1s, noop); }
    catch (const std::runtime_error&) { table_error = true; }
    require(table_error && !std::filesystem::exists(scratch), "缺表未报错或生成了文件");
    std::filesystem::create_directories(tables);
    std::filesystem::copy_file(root / "modules/cube/third_party/kociemba/cprunetables/twistMove",
                               tables / "twistMove");
    { std::fstream file(tables / "twistMove", std::ios::binary | std::ios::in | std::ios::out);
      file.put('\x7f'); }
    table_error = false;
    try { enumerate_solutions(scramble, scratch, Clock::now() + 1s, noop); }
    catch (const std::runtime_error&) { table_error = true; }
    std::filesystem::remove_all(scratch);
    require(table_error, "损坏表未报错");
    run(scramble, 1);
    std::cout << "候选枚举测试通过：独立重放、多候选、截止时间、回调停止、异常、状态隔离、表校验、原入口。\n";
  } catch (const std::exception& error) {
    std::cerr << "失败：" << error.what() << '\n';
    return 1;
  }
}
