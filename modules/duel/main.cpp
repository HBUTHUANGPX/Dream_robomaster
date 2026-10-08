#include <charconv>
#include <cmath>
#include <iostream>
#include <string_view>

#include "duel.hpp"

int main(int argc, char** argv) {
  try {
    bool rpc = false;
    bool duration_set = false;
    bool ticks_set = false;
    int ticks = 0;
    auto value = [&](int& i, const std::string& flag) -> std::string {
      if (++i >= argc) throw std::invalid_argument(flag + " requires a value");
      return argv[i];
    };
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--rpc") {
        rpc = true;
      } else if (arg == "--root") {
        auto path = value(i, arg);
        if (path.empty()) throw std::invalid_argument("--root requires a path");
      } else if (arg == "--ticks") {
        const std::string text = value(i, arg);
        auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), ticks);
        if (error != std::errc{} || end != text.data() + text.size() || ticks < 0 || ticks > 72000)
          throw std::invalid_argument("--ticks must be an integer in 0..72000");
        ticks_set = true;
      } else if (arg == "--duration") {
        const std::string text = value(i, arg);
        size_t used = 0;
        double seconds = std::stod(text, &used);
        if (used != text.size() || !std::isfinite(seconds) || seconds < 0 || seconds > 3600)
          throw std::invalid_argument("--duration must be finite seconds in 0..3600");
        ticks = static_cast<int>(std::ceil(seconds / .05));
        duration_set = true;
      } else if (arg == "--headless") {
        // Native workers always render offscreen. No viewer is required.
      } else if (arg == "--help") {
        std::cout
            << "自瞄对战程序用法：\n"
               "rm_duel [--root 仓库路径] [--rpc | --headless [--ticks 步数 | --duration 秒数]]\n"
               "程序使用无窗口渲染。网页入口为 ./rm duel。\n";
        return 0;
      } else {
        throw std::invalid_argument("unknown argument: " + arg);
      }
    }
    if ((ticks_set && duration_set) || (rpc && (ticks_set || duration_set)))
      throw std::invalid_argument("--rpc, --ticks, and --duration are mutually exclusive");
    rm::duel::Duel game(rm::repo_root(argc, argv));
    if (rpc)
      return rm::rpc_loop(
          [&](const std::string& name, const rm::Json& args) { return game.command(name, args); });
    for (int i = 0; i < ticks; ++i) game.step();
    std::cout << game.state().dump() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "rm_duel: " << e.what() << '\n';
    return 1;
  }
}
