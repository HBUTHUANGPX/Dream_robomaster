#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rm::cube {
struct CandidateSearchStats {
  size_t candidates = 0;
  bool timed_out = false;
  bool cancelled = false;
};

class CandidateTables;

// 每次校验 root 下全部附带表；仅首次受锁发布，此后全局表只读。
// 超时返回 nullptr；缺失或损坏抛出 runtime_error。截止时间包含等待发布锁。
// 会话可跨线程复用，不再次读取源文件；源文件后续变化由下一次 prepare 检出。
std::shared_ptr<const CandidateTables> prepare_candidate_tables(
    const std::filesystem::path& root, std::chrono::steady_clock::time_point deadline);

// 已准备会话的枚举无全局搜索锁，各调用的回调和取消状态相互独立。
// cancel 为可选外部取消标志，调用期间必须有效；取消只设置 cancelled。
// 若截止时间也已到达，timed_out 同时为 true。回调 false 不视为外部取消。
CandidateSearchStats enumerate_solutions(
    const std::string& facelets, const CandidateTables& tables,
    std::chrono::steady_clock::time_point deadline,
    const std::function<bool(const std::vector<std::string>&)>& on_solution,
    const std::atomic<bool>* cancel = nullptr);

// root 是仓库根目录。回调收到 URFDLB 面转序列，true 继续，false 停止。
// 截止时间包含表加载、等待锁和回调；回调本身须及时返回，无法被抢占。
// 已还原状态交付一个空序列。过期预算直接返回，不验证输入或读取表。
// 非法或不可能状态抛出 invalid_argument；表缺失或校验失败抛出 runtime_error。
// 回调异常在 C 搜索清理后原样重抛。枚举最长 24 步，不保证穷举或最优。
// prepare 与枚举可以并发；不得与原 solution/initPruning 并发调用。
// 文件系统调用和用户回调无法抢占；原入口仅允许在无并发搜索时使用。
CandidateSearchStats enumerate_solutions(
    const std::string& facelets, const std::filesystem::path& root,
    std::chrono::steady_clock::time_point deadline,
    const std::function<bool(const std::vector<std::string>&)>& on_solution);
}  // namespace rm::cube
