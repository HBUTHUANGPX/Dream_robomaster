#pragma once

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace rm::cube {
struct CandidateSearchStats {
  size_t candidates = 0;
  bool timed_out = false;
};

// root 是仓库根目录。回调收到 URFDLB 面转序列，true 继续，false 停止。
// 截止时间包含表加载、等待锁和回调；回调本身须及时返回，无法被抢占。
// 已还原状态交付一个空序列。过期预算直接返回，不验证输入或读取表。
// 非法或不可能状态抛出 invalid_argument；表缺失或校验失败抛出 runtime_error。
// 回调异常在 C 搜索清理后原样重抛。枚举最长 24 步，不保证穷举或最优。
// 枚举调用之间串行化；不得与原 solution/initPruning 并发调用。
CandidateSearchStats enumerate_solutions(
    const std::string& facelets, const std::filesystem::path& root,
    std::chrono::steady_clock::time_point deadline,
    const std::function<bool(const std::vector<std::string>&)>& on_solution);
}  // namespace rm::cube
