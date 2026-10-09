#include <barrier>
#include <future>
#include <iostream>
#include <thread>

#include "rm/robot_search.hpp"

using namespace rm::cube;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace {
void need(bool value, const char* why) {
  if (!value) throw std::runtime_error(why);
}
void verify(const RobotSearchResult& result, const std::string& state, const RobotState& start,
            const SearchOptions& options) {
  need(result.found, "预算内没有方案");
  need(replay_facelet_moves(state, primitive_moves(result.plan.actions, start)) == solved,
       "并行方案未还原");
  need(result.threads_used >= 1 && result.threads_used <= options.threads, "工作线程数错误");
  need(result.worker_memory_limit_mb * result.threads_used <= options.memory_limit_mb,
       "线程合计图预算超过请求上限");
  size_t candidates = 0, optimized = 0;
  for (const auto& worker : result.workers) {
    candidates += worker.at("candidates").get<size_t>();
    optimized += worker.at("optimized_candidates").get<size_t>();
  }
  need(candidates == result.candidates && optimized == result.optimized_candidates,
       "并行统计合计不符");
  double previous = std::numeric_limits<double>::infinity();
  double time = -1;
  for (const auto& improvement : result.improvements) {
    const double cost = options.action_count ? improvement.at("action_count").get<double>()
                                             : improvement.at("execution_s").get<double>();
    need(cost <= previous && improvement.at("search_ms").get<double>() >= time,
         "共享最好方案退化或发布时间倒退");
    previous = cost;
    time = improvement.at("search_ms");
  }
  need(result.json(options).at("threads_requested") == options.threads, "线程配置未序列化");
}
}  // namespace
int main(int argc, char** argv) {
  try {
    need(argc == 2, "需要仓库路径");
    const std::filesystem::path root = argv[1];
    // 第一次访问动作图即并发，验证初始化发布，不依赖预热。
    std::barrier gate(6);
    std::vector<std::future<void>> cold;
    for (int i = 0; i < 6; ++i)
      cold.push_back(std::async(std::launch::async, [&, i] {
        SearchOptions options;
        options.home = i % 2;
        options.costs.duration[int(Primitive::A_P90)][1] = .1 + i * .01;
        RobotState start{i, {i % 5 - 2, 0}, {true, i % 2 == 0}};
        gate.arrive_and_wait();
        const std::vector<std::string> target{"R2", "U", "F'"};
        auto p = optimize_robot_moves(target, start, options, Clock::now() + 3s);
        need(p && p->optimal_for_sequence, "冷启动并发图搜索失败");
        need(replay_facelet_moves(solved, primitive_moves(p->actions, start)) ==
                 replay_facelet_moves(solved, target),
             "并发固定序列结果污染");
      }));
    for (auto& future : cold) future.get();
    SearchFeedback feedback;
    feedback.cancel.store(true);
    need(!optimize_robot_moves({"U"}, {}, {}, Clock::now() + 1s,
                               std::numeric_limits<double>::infinity(), &feedback),
         "预先取消仍继续搜索");
    feedback.cancel.store(false);
    feedback.upper_bound.store(0);
    need(!optimize_robot_moves({"U"}, {}, {}, Clock::now() + 1s,
                               std::numeric_limits<double>::infinity(), &feedback),
         "未采用共享上界剪枝");
    SearchFeedback interrupted;
    SearchOptions long_options;
    long_options.memory_limit_mb = 256;
    std::barrier entered(2);
    auto began = Clock::now();
    auto cancelled_search = std::async(std::launch::async, [&] {
      std::vector<std::string> moves;
      for (int i = 0; i < 40; ++i) moves.insert(moves.end(), {"R", "U", "F"});
      entered.arrive_and_wait();
      return optimize_robot_moves(moves, {}, long_options, Clock::now() + 30s,
                                  std::numeric_limits<double>::infinity(), &interrupted);
    });
    entered.arrive_and_wait();
    std::this_thread::sleep_for(2ms);
    interrupted.cancel.store(true);
    cancelled_search.get();
    need(Clock::now() - began < 2s, "运行中 A* 取消仍等待长截止时间");
    const auto state = replay_facelet_moves(solved, {"R", "U", "F'", "L2"});
    for (int threads : {1, 2, 3}) {
      SearchOptions options;
      options.threads = threads;
      options.max_search_ms = 400;
      verify(search_robot_solution(state, {}, options, root), state, {}, options);
    }
    // 同时发起两个完整请求，成本、起态、目标与缓存不能串扰。
    std::vector<std::future<void>> requests;
    for (int i = 0; i < 2; ++i)
      requests.push_back(std::async(std::launch::async, [&, i] {
        SearchOptions options;
        options.max_search_ms = 500;
        options.home = i;
        options.action_count = i;
        options.costs.duration[0][1] = .002 + i;
        RobotState start{7 * i, {i, 0}, {true, false}};
        verify(search_robot_solution(state, start, options, root), state, start, options);
      }));
    for (auto& request : requests) request.get();
    SearchOptions low;
    low.max_search_ms = 300;
    low.memory_limit_mb = 4;
    const auto one_turn = replay_facelet_moves(solved, {"R"});
    auto result = search_robot_solution(one_turn, {}, low, root);
    verify(result, one_turn, {}, low);
    need(result.threads_used == 1, "低内存未退回单线程");
    low.max_search_ms = 0;
    result = search_robot_solution(one_turn, {}, low, root);
    need(!result.found && result.threads_used == 0 && result.stop_reason == "deadline",
         "零预算仍启动搜索线程");
    result = search_robot_solution(solved, {}, low, root);
    need(result.found && result.plan.actions.empty(), "并行配置丢失已还原空解");
    for (auto invalid : {rm::Json(0), rm::Json(4), rm::Json(1.5), rm::Json(true), rm::Json("3")}) {
      bool rejected = false;
      try {
        search_options({{"threads", invalid}}, low.costs);
      } catch (const std::exception&) {
        rejected = true;
      }
      need(rejected, "非法线程数未拒绝");
    }
    std::cout << "并行搜索测试通过：冷启动、请求隔离、线程配置、内存合计、取消、共享上界。\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
