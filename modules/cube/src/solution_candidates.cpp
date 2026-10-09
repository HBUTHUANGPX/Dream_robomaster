#include "rm/solution_candidates.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

extern "C" {
#include "coordcube.h"
#include "facecube.h"
#include "search.h"
}

namespace rm::cube {
// 不透明凭证：只由完成校验及发布的 prepare 创建，不保存可写表指针。
class CandidateTables {};

namespace {
using Clock = std::chrono::steady_clock;
constexpr char order[] = "URFDLB";
const std::string solved = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
std::mutex publish_mutex;
std::atomic<bool> tables_published{false};

struct Context {
  Clock::time_point deadline;
  const std::function<bool(const std::vector<std::string>&)>& callback;
  CandidateSearchStats stats;
  std::exception_ptr error;
  const std::atomic<bool>* cancel;
};

int cancelled(void* raw) {
  auto& ctx = *static_cast<Context*>(raw);
  if (Clock::now() >= ctx.deadline) ctx.stats.timed_out = true;
  if (ctx.cancel && ctx.cancel->load(std::memory_order_relaxed)) ctx.stats.cancelled = true;
  return ctx.stats.timed_out || ctx.stats.cancelled || bool(ctx.error);
}

int deliver(const int* axes, const int* powers, int length, void* raw) noexcept {
  auto& ctx = *static_cast<Context*>(raw);
  if (cancelled(raw)) return 0;
  try {
    std::vector<std::string> moves;
    moves.reserve(length);
    for (int i = 0; i < length; ++i) {
      std::string move(1, order[axes[i]]);
      if (powers[i] == 2) move += '2';
      if (powers[i] == 3) move += '\'';
      moves.push_back(std::move(move));
    }
    if (cancelled(raw)) return 0;
    ++ctx.stats.candidates;
    const bool keep_going = ctx.callback(moves);
    return !cancelled(raw) && keep_going;
  } catch (...) {
    ctx.error = std::current_exception();
    return 0;
  }
}

struct Table {
  const char* name;
  void* target;
  size_t size;
  uint64_t checksum;
};

// 校验值对应仓库附带表的原始字节，使用 FNV-1a 64；不接受重新生成的替代表。
const Table tables[] = {
    {"twistMove", twistMove, sizeof(twistMove), 0xbfb3e230cad57375ULL},
    {"flipMove", flipMove, sizeof(flipMove), 0xe935b23dbeb2a925ULL},
    {"FRtoBR_Move", FRtoBR_Move, sizeof(FRtoBR_Move), 0xa92b31db4b39efe5ULL},
    {"URFtoDLF_Move", URFtoDLF_Move, sizeof(URFtoDLF_Move), 0xa0d11018d6ba6669ULL},
    {"URtoDF_Move", URtoDF_Move, sizeof(URtoDF_Move), 0x48f24292eb2c0251ULL},
    {"URtoUL_Move", URtoUL_Move, sizeof(URtoUL_Move), 0xf651041c12f37c21ULL},
    {"UBtoDF_Move", UBtoDF_Move, sizeof(UBtoDF_Move), 0xf651041c12f37c21ULL},
    {"MergeURtoULandUBtoDF", MergeURtoULandUBtoDF, sizeof(MergeURtoULandUBtoDF), 0xad8a8972355dad59ULL},
    {"Slice_URFtoDLF_Parity_Prun", Slice_URFtoDLF_Parity_Prun, sizeof(Slice_URFtoDLF_Parity_Prun), 0x4ac7db36793f6225ULL},
    {"Slice_URtoDF_Parity_Prun", Slice_URtoDF_Parity_Prun, sizeof(Slice_URtoDF_Parity_Prun), 0xad56903d88b68621ULL},
    {"Slice_Twist_Prun", Slice_Twist_Prun, sizeof(Slice_Twist_Prun), 0xa3400d51e05ffe71ULL},
    {"Slice_Flip_Prun", Slice_Flip_Prun, sizeof(Slice_Flip_Prun), 0x1c4c07f828a483f4ULL},
};

bool read_tables(const std::filesystem::path& root, Clock::time_point deadline,
                 std::vector<std::vector<unsigned char>>& data) {
  // 全部验证完成后才发布，超时或失败不会发布部分初始化的全局表。
  for (const auto& table : tables) {
    if (Clock::now() >= deadline) return false;
    const auto path = root / "modules/cube/third_party/kociemba/cprunetables" / table.name;
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error) ||
        std::filesystem::file_size(path, error) != table.size || error)
      throw std::runtime_error("求解表缺失或长度错误: " + path.string());
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("无法读取求解表: " + path.string());
    auto& bytes = data.emplace_back(table.size);
    uint64_t hash = 14695981039346656037ULL;
    for (size_t offset = 0; offset < bytes.size();) {
      if (Clock::now() >= deadline) return false;
      const auto count = std::min<size_t>(4096, bytes.size() - offset);
      if (!input.read(reinterpret_cast<char*>(bytes.data() + offset), count))
        throw std::runtime_error("求解表读取失败: " + path.string());
      for (size_t i = offset; i < offset + count; ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ULL;
      }
      offset += count;
    }
    if (Clock::now() >= deadline) return false;
    if (hash != table.checksum)
      throw std::runtime_error("求解表校验失败: " + path.string());
  }
  if (Clock::now() >= deadline) return false;
  return true;
}
}  // namespace

std::shared_ptr<const CandidateTables> prepare_candidate_tables(const std::filesystem::path& root,
                                                                Clock::time_point deadline) {
  if (Clock::now() >= deadline) return {};
  std::vector<std::vector<unsigned char>> data;
  if (!read_tables(root, deadline, data)) return {};
  auto session = std::make_shared<const CandidateTables>();
  if (!tables_published.load(std::memory_order_acquire)) {
    std::unique_lock<std::mutex> lock(publish_mutex, std::defer_lock);
    // 短间隔尝试保证等待锁也受单调截止时间限制，不依赖平台的定时锁时钟。
    while (!lock.try_lock()) {
      const auto now = Clock::now();
      if (now >= deadline) return {};
      std::this_thread::sleep_until(std::min(deadline, now + std::chrono::microseconds(50)));
    }
    if (Clock::now() >= deadline) return {};
    if (!tables_published.load(std::memory_order_relaxed)) {
      // 仅首发线程可写。中途超时不发布标志，也不交付会话；下次完整覆盖。
      // 枚举不可能观察尚未发布的表。legacy 初始化不得与此路径并发。
      const uint16_t endian = 1;
      for (size_t i = 0; i < std::size(tables); ++i) {
        for (size_t offset = 0; offset < data[i].size();) {
          if (Clock::now() >= deadline) return {};
          const auto count = std::min<size_t>(4096, data[i].size() - offset);
          if (i < 8 && *reinterpret_cast<const unsigned char*>(&endian) != 1)
            for (size_t j = offset; j < offset + count; j += 2)
              std::swap(data[i][j], data[i][j + 1]);
          std::memcpy(static_cast<unsigned char*>(tables[i].target) + offset,
                      data[i].data() + offset, count);
          offset += count;
        }
      }
      if (Clock::now() >= deadline) return {};
      PRUNING_INITED = 1;
      // release 发布涵盖 C 表与普通 int 标志；之后二者均不再写入。
      tables_published.store(true, std::memory_order_release);
    }
  }
  if (Clock::now() >= deadline) return {};
  return session;
}

namespace {
CandidateSearchStats enumerate_impl(
    const std::string& facelets, const std::filesystem::path* root, Clock::time_point deadline,
    const std::function<bool(const std::vector<std::string>&)>& on_solution,
    const std::atomic<bool>* cancel) {
  Context ctx{deadline, on_solution, {}, {}, cancel};
  if (cancelled(&ctx)) return ctx.stats;
  if (!on_solution) throw std::invalid_argument("候选回调不能为空");
  std::array<int, 6> counts{};
  if (facelets.size() != 54) throw std::invalid_argument("魔方面贴纸必须为 54 个字符");
  for (char face : facelets) {
    const auto* found = std::strchr(order, face);
    if (!found || face == '\0') throw std::invalid_argument("魔方面贴纸颜色非法");
    ++counts[found - order];
  }
  for (size_t i = 0; i < counts.size(); ++i)
    if (counts[i] != 9 || facelets[i * 9 + 4] != order[i])
      throw std::invalid_argument("魔方面贴纸数量或中心颜色非法");
  std::string input = facelets;
  std::unique_ptr<facecube_t, decltype(&std::free)> fc(get_facecube_fromstring(input.data()),
                                                       std::free);
  std::unique_ptr<cubiecube_t, decltype(&std::free)> cc(toCubieCube(fc.get()), std::free);
  if (verify(cc.get()) != 0) throw std::invalid_argument("魔方状态不可能还原");
  if (cancelled(&ctx)) return ctx.stats;
  if (facelets == solved) {
    deliver(nullptr, nullptr, 0, &ctx);
  } else {
    std::shared_ptr<const CandidateTables> session;
    if (root) {
      session = prepare_candidate_tables(*root, deadline);
      if (!session) {
        ctx.stats.timed_out = true;
        return ctx.stats;
      }
    }
    // 与首次发布建立 happens-before，C 层读取的表及 PRUNING_INITED 均已稳定。
    if (!tables_published.load(std::memory_order_acquire)) throw std::logic_error("候选表尚未发布");
    enumerate_search(input.data(), cancelled, deliver, &ctx);
  }
  if (ctx.error) std::rethrow_exception(ctx.error);
  cancelled(&ctx);
  return ctx.stats;
}
}  // namespace

CandidateSearchStats enumerate_solutions(
    const std::string& facelets, const std::filesystem::path& root, Clock::time_point deadline,
    const std::function<bool(const std::vector<std::string>&)>& on_solution) {
  return enumerate_impl(facelets, &root, deadline, on_solution, nullptr);
}

CandidateSearchStats enumerate_solutions(
    const std::string& facelets, const CandidateTables&, Clock::time_point deadline,
    const std::function<bool(const std::vector<std::string>&)>& on_solution,
    const std::atomic<bool>* cancel) {
  return enumerate_impl(facelets, nullptr, deadline, on_solution, cancel);
}
}  // namespace rm::cube
