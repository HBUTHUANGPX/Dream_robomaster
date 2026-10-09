#include <array>
#include <barrier>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "rm/solution_candidates.hpp"

extern "C" {
#include "coordcube.h"
}

using namespace rm::cube;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace {
using Moves = std::vector<std::string>;
const std::string faces = "URFDLB";
const std::string solved = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
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

// 有截止时间的回调屏障：串行实现会失败返回，而不是令测试永久挂起。
class CallbackBarrier {
 public:
  bool arrive_and_wait(Clock::time_point deadline) {
    arrivals_.fetch_add(1, std::memory_order_acq_rel);
    while (arrivals_.load(std::memory_order_acquire) != 4) {
      if (Clock::now() >= deadline) return false;
      std::this_thread::sleep_for(50us);
    }
    return true;
  }

 private:
  std::atomic<int> arrivals_{0};
};
struct CallbackFailure : std::runtime_error {
  CallbackFailure() : std::runtime_error("预期回调异常") {}
};
template <class F>
void table_rejects(F&& fn) {
  bool rejected = false;
  try {
    fn();
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "缺失或损坏的源表未拒绝");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 2, "需要仓库根目录参数");
    const std::filesystem::path root = argv[1];
    const auto no_op = [](const auto&) { return true; };
    require(PRUNING_INITED == 0, "冷启动测试必须在独立进程运行");
    require(!prepare_candidate_tables(root / "不存在", Clock::now()), "零预算仍准备了会话");
    auto stats = enumerate_solutions(solved, root / "不存在", Clock::now() + 1s, no_op);
    require(stats.candidates == 1 && !stats.timed_out && PRUNING_INITED == 0,
            "已还原状态意外加载了表");
    auto start = Clock::now();
    require(!prepare_candidate_tables(root, start + 1ms), "小预算意外交付完整表会话");
    require(PRUNING_INITED == 0 && Clock::now() - start < 500ms, "加载取消后发布了表或超时过长");

    const std::array<std::string, 4> states{
        replay(solved, {"R", "U", "F'", "L2"}), replay(solved, {"F", "U2", "R"}),
        replay(solved, {"L", "D", "B2"}), replay(solved, {"B", "R2", "D'"})};
    std::array<std::shared_ptr<const CandidateTables>, 4> sessions;
    std::array<std::exception_ptr, 4> errors{};
    std::array<CandidateSearchStats, 4> results{};
    std::array<bool, 4> expected_exception{};
    std::barrier launch(4);
    CallbackBarrier callbacks;
    auto deadline = Clock::now() + 5s;
    {
      std::array<std::jthread, 4> workers;
      for (std::size_t i = 0; i < workers.size(); ++i)
        workers[i] = std::jthread([&, i] {
          launch.arrive_and_wait();
          try {
            // 同时首次准备、发布、搜索，覆盖发布与其他线程仍校验文件的重叠。
            sessions[i] = prepare_candidate_tables(root, deadline);
            require(bool(sessions[i]), "并发准备超时");
            results[i] =
                enumerate_solutions(states[i], *sessions[i], deadline, [&](const auto& moves) {
                  require(replay(states[i], moves) == solved, "并行首解无法独立还原");
                  require(callbacks.arrive_and_wait(deadline),
                          "回调不能同时到达屏障，搜索仍被串行化");
                  if (i == 0) throw CallbackFailure();
                  return false;
                });
          } catch (const CallbackFailure&) {
            expected_exception[i] = true;
          } catch (...) {
            errors[i] = std::current_exception();
          }
        });
    }
    for (auto error : errors)
      if (error) std::rethrow_exception(error);
    require(expected_exception[0], "工作线程回调异常未传播");
    for (std::size_t i = 1; i < results.size(); ++i)
      require(!expected_exception[i] && results[i].candidates == 1 && !results[i].timed_out,
              "一个回调异常污染了其他请求");
    require(PRUNING_INITED == 1, "完整表未发布");

    // 同一会话被多个工作线程共享；另一个线程持续 prepare，不能重写已发布表。
    deadline = Clock::now() + 250ms;
    errors = {};
    {
      std::array<std::jthread, 4> workers;
      for (std::size_t i = 0; i < workers.size(); ++i)
        workers[i] = std::jthread([&, i] {
          try {
            if (i == 3) {
              for (int pass = 0; pass < 8 && Clock::now() < deadline; ++pass)
                prepare_candidate_tables(root, deadline);
            } else {
              results[i] =
                  enumerate_solutions(states[i], *sessions[0], deadline, [&](const auto& moves) {
                    require(replay(states[i], moves) == solved, "会话复用时状态污染");
                    return true;
                  });
            }
          } catch (...) {
            errors[i] = std::current_exception();
          }
        });
    }
    for (auto error : errors)
      if (error) std::rethrow_exception(error);
    for (std::size_t i = 0; i < 3; ++i)
      require(results[i].timed_out && results[i].candidates > 0, "共享截止时间或状态隔离失效");
    stats = enumerate_solutions("非法输入", *sessions[0], Clock::now(), {});
    require(stats.timed_out && stats.candidates == 0, "零预算语义改变");
    stats = enumerate_solutions(solved, *sessions[0], Clock::now() + 1s, no_op);
    require(stats.candidates == 1 && !stats.timed_out, "会话已还原语义错误");
    bool empty_callback_rejected = false;
    try {
      enumerate_solutions(states[0], *sessions[0], Clock::now() + 1s, {});
    } catch (const std::invalid_argument&) {
      empty_callback_rejected = true;
    }
    require(empty_callback_rejected, "空回调语义改变");

    // 取消可打断尚未交付候选的 C 搜索，不必等待三十秒截止时间。
    std::atomic<bool> cancel{true};
    stats = enumerate_solutions("非法输入", *sessions[0], Clock::now() + 30s, {}, &cancel);
    require(stats.cancelled && !stats.timed_out && stats.candidates == 0,
            "预置取消未立即返回或被误报为超时");
    cancel.store(false);
    std::barrier cancel_launch(2);
    const auto difficult =
        replay(solved, {"R", "U",  "F'", "L2", "D",  "B",  "R'", "U2", "F",  "D'",
                        "L", "B2", "U",  "R2", "D2", "F2", "L'", "B'", "U2", "R"});
    std::exception_ptr cancel_error;
    CandidateSearchStats cancel_result;
    start = Clock::now();
    {
      std::jthread worker([&] {
        cancel_launch.arrive_and_wait();
        try {
          cancel_result = enumerate_solutions(
              difficult, *sessions[0], Clock::now() + 30s,
              [&](const auto& moves) {
                require(replay(difficult, moves) == solved, "取消前候选无效");
                return true;
              },
              &cancel);
        } catch (...) {
          cancel_error = std::current_exception();
        }
      });
      cancel_launch.arrive_and_wait();
      std::this_thread::sleep_for(2ms);
      cancel.store(true);
    }
    if (cancel_error) std::rethrow_exception(cancel_error);
    require(cancel_result.cancelled && !cancel_result.timed_out && Clock::now() - start < 500ms,
            "运行中取消未及时终止 C 搜索");
    stats = enumerate_solutions(states[0], *sessions[0], Clock::now() + 2s,
                                [](const auto&) { return false; });
    require(stats.candidates == 1 && !stats.cancelled && !stats.timed_out,
            "其他请求的取消污染了独立搜索");

    // 独立临时根目录，不修改仓库表；源文件变化只影响下一次 prepare。
    const auto scratch = root / "output/candidate-parallel-fixture";
    const std::filesystem::path relative = "modules/cube/third_party/kociemba/cprunetables";
    std::filesystem::remove_all(scratch);
    std::filesystem::create_directories(scratch / relative);
    std::filesystem::copy(root / relative, scratch / relative,
                          std::filesystem::copy_options::recursive);
    auto saved = prepare_candidate_tables(scratch, Clock::now() + 2s);
    require(bool(saved), "独立根目录准备失败");
    std::filesystem::remove(scratch / relative / "Slice_Flip_Prun");
    table_rejects([&] { prepare_candidate_tables(scratch, Clock::now() + 2s); });
    std::filesystem::copy_file(root / relative / "Slice_Flip_Prun",
                               scratch / relative / "Slice_Flip_Prun");
    // 修改最后一个表，确认暖加载仍校验全部文件，并拒绝同长度损坏。
    {
      std::fstream file(scratch / relative / "Slice_Flip_Prun",
                        std::ios::binary | std::ios::in | std::ios::out);
      char value = 0;
      file.get(value);
      file.seekp(0);
      file.put(static_cast<char>(static_cast<unsigned char>(value) ^ 0x01));
    }
    table_rejects([&] { prepare_candidate_tables(scratch, Clock::now() + 2s); });
    start = Clock::now();
    require(!prepare_candidate_tables(root, start + 1ms), "暖加载跳过了源文件校验");
    std::filesystem::remove_all(scratch);
    // 删除全部源文件后已有会话仍可使用，枚举过程没有重复 I/O。
    stats = enumerate_solutions(states[0], *saved, Clock::now() + 2s, [&](const auto& moves) {
      require(replay(states[0], moves) == solved, "失败的 prepare 污染已有会话");
      return false;
    });
    require(stats.candidates == 1 && !stats.timed_out, "已有会话不能独立于源文件复用");
    std::cout << "并行候选测试通过：首次并发发布、四回调屏障、独立异常、同截止时间、只读会话复用、"
                 "缺损表拒绝与加载取消。\n";
  } catch (const std::exception& error) {
    std::cerr << "失败：" << error.what() << '\n';
    return 1;
  }
}
