#pragma once
#include <atomic>
#include <chrono>

#include "rm/primitive.hpp"
namespace rm::cube {
struct SearchOptions {
  double max_search_ms = 1000;
  int threads = 3;
  bool action_count = false, home = false;
  size_t memory_limit_mb = 64;
  PrimitiveCosts costs = PrimitiveCosts::defaults(8, 32, true);
};
struct RobotPlan {
  std::vector<Primitive> actions;
  RobotState end;
  double execution_s = 0, cost = 0;
  size_t expanded = 0;
  bool optimal_for_sequence = false;
  std::string stop_reason;
};
// 仅共享成本上界与取消标记；队列、标签及方案由每次搜索独立持有。
struct SearchFeedback {
  std::atomic<double> upper_bound{std::numeric_limits<double>::infinity()};
  std::atomic<bool> cancel{false};
};
std::optional<RobotPlan> optimize_robot_moves(
    const std::vector<std::string>& moves, const RobotState& start, const SearchOptions& options,
    std::chrono::steady_clock::time_point deadline,
    double upper_bound = std::numeric_limits<double>::infinity(),
    const SearchFeedback* feedback = nullptr);
struct RobotSearchResult {
  bool found = false;
  RobotPlan plan;
  RobotState start;
  double search_ms = 0, preparation_ms = 0;
  int threads_used = 0;
  size_t worker_memory_limit_mb = 0;
  Json workers = Json::array();
  size_t candidates = 0, optimized_candidates = 0;
  std::string stop_reason;
  Json improvements = Json::array();
  Json json(const SearchOptions& options) const;
};
RobotSearchResult search_robot_solution(const std::string& facelets, const RobotState& start,
                                        const SearchOptions& options,
                                        const std::filesystem::path& root);
SearchOptions search_options(const Json& args, const PrimitiveCosts& defaults);
std::string replay_facelet_moves(const std::string& state, const std::vector<std::string>& moves);
}  // namespace rm::cube
