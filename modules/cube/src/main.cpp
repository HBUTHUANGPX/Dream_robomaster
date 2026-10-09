#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>

#include "rm/cube.hpp"
#include "rm/robot_search.hpp"
namespace {
using namespace rm;
using namespace rm::cube;
void write_json(const std::filesystem::path& p, const Json& j) {
  std::ofstream out(p);
  if (!out) throw std::runtime_error("Cannot write " + p.string());
  out << j.dump(2) << '\n';
}
struct Recorder {
  int fd = -1;
  pid_t pid = -1;
  void open(const std::filesystem::path& output) {
    int pipefd[2];
    if (pipe(pipefd)) throw std::runtime_error("Recorder pipe failed");
    pid = fork();
    if (pid < 0) {
      close(pipefd[0]);
      close(pipefd[1]);
      throw std::runtime_error("Recorder fork failed");
    }
    if (pid == 0) {
      dup2(pipefd[0], STDIN_FILENO);
      close(pipefd[0]);
      close(pipefd[1]);
      execlp("ffmpeg", "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
             "-s", "960x720", "-r", "30", "-i", "-", "-an", "-c:v", "libx264", "-preset", "fast",
             "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", output.c_str(),
             static_cast<char*>(nullptr));
      _exit(127);
    }
    close(pipefd[0]);
    fd = pipefd[1];
    signal(SIGPIPE, SIG_IGN);
  }
  void write(const cv::Mat& frame) {
    size_t total = frame.total() * frame.elemSize(), offset = 0;
    while (offset < total) {
      auto n = ::write(fd, frame.data + offset, total - offset);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) throw std::runtime_error("FFmpeg frame pipe failed");
      offset += n;
    }
  }
  void release() {
    if (fd >= 0) {
      close(fd);
      fd = -1;
    }
    if (pid > 0) {
      int status = 0;
      while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      pid = -1;
      if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
        throw std::runtime_error("FFmpeg failed encoding video");
    }
  }
  ~Recorder() {
    try {
      release();
    } catch (...) {
    }
  }
};
struct App {
  std::filesystem::path root;
  bool dual, paused = false;
  double speed;
  std::optional<double> ws, js;
  std::unique_ptr<Cube> cube;
  std::unique_ptr<Renderer> renderer;
  App(std::filesystem::path r, bool d, double s, std::optional<double> w, std::optional<double> j)
      : root(std::move(r)), dual(d), speed(s), ws(w), js(j) {
    reset();
  }
  void reset() {
    renderer.reset();
    cube = std::make_unique<Cube>(root, dual, speed, ws, js);
    paused = false;
  }
  cv::Mat frame(bool closeup = false) {
    if (!renderer) renderer = std::make_unique<Renderer>(cube->model, 960, 720);
    mjvCamera camera;
    mjv_defaultCamera(&camera);
    camera.lookat[0] = dual && !closeup ? .025 : 0;
    camera.lookat[1] = dual && !closeup ? -.025 : 0;
    camera.lookat[2] = dual ? (closeup ? .22 : .195) : .105;
    camera.distance = dual ? (closeup ? .155 : .69) : .24;
    camera.azimuth = 135;
    camera.elevation = -30;
    return renderer->render(cube->data, camera);
  }
  Json search_defaults = Json::object();
  SearchOptions options(const Json& args) const {
    Json merged = search_defaults;
    merged.update(args);
    return search_options(merged,
                          PrimitiveCosts::defaults(cube->wrist_speed, cube->jaw_speed, cube->fast));
  }
  void robot_moves(const std::vector<std::string>& moves, const Json& args) {
    auto o = options(args);
    auto start = robot_snapshot(*cube);
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double, std::milli>(o.max_search_ms));
    auto plan = optimize_robot_moves(moves, start, o, deadline);
    if (!plan) throw std::runtime_error("No robot plan within search budget");
    execute_primitives(*cube, plan->actions, start);
  }
  Json handle(const std::string& cmd, const Json& args) {
    if (cmd == "state") return cube->report();
    if (cmd == "reset") {
      reset();
      return cube->report();
    }
    if (cmd == "pause") {
      paused = args.value("paused", !paused);
      return {{"paused", paused}};
    }
    if (cmd == "tick") {
      if (!paused) cube->advance(.016);
      return cube->report();
    }
    if (cmd == "frame") {
      auto view = args.value("view", std::string("scene"));
      if (view != "scene" && view != "closeup") throw std::invalid_argument("Unknown cube view");
      return jpeg_response(frame(view == "closeup"));
    }
    if (cmd == "move" || cmd == "scramble") {
      auto moves = split_moves(args.value(
          "moves", args.value("move", cmd == "scramble" ? std::string("R U F' L2 D B R' U2 F D'")
                                                        : std::string())));
      if (moves.empty()) throw std::invalid_argument("Supply move or moves");
      if (cube->robot_ready)
        robot_moves(moves, args);
      else
        for (auto& m : moves) cube->turn(m);
      return cube->report();
    }
    if (cmd == "solve" || cmd == "plan") {
      auto state = args.value("facelets", cube->facelets());
      if (dual) {
        if (cmd == "solve" && args.value("execute", true) && state != cube->facelets())
          throw std::invalid_argument("Cannot execute solution for a different physical cube");
        auto o = options(args);
        auto start = robot_snapshot(*cube);
        auto result = search_robot_solution(state, start, o, root);
        auto report = result.json(o);
        if (cmd == "solve" && args.value("execute", true) && result.found) {
          if (!cube->robot_ready) cube->initialize_grasps();
          execute_primitives(*cube, result.plan.actions, start);
          if (cube->facelets() != rm::cube::solved)
            throw std::runtime_error("Physical cube did not finish solved");
        }
        report["state"] = cube->report();
        return report;
      }
      auto moves = solve_facelets(state, root);
      auto raw = compile_moves(moves, cube->orientation),
           plan = optimize_plan(raw, cube->orientation);
      if (cmd == "solve" && args.value("execute", true)) {
        if (args.contains("facelets") && state != cube->facelets())
          throw std::invalid_argument("Cannot execute solution for a different physical cube");
        if (dual) {
          if (!cube->robot_ready) cube->initialize_grasps();
          cube->execute(plan);
        } else
          for (auto& m : moves) cube->turn(m);
      }
      return {{"solution", moves},
              {"unoptimized_actions", plan_json(raw)},
              {"actions", plan_json(plan)},
              {"state", cube->report()}};
    }
    if (cmd == "gripper") {
      if (!dual) throw std::invalid_argument("Start with --dual for gripper commands");
      auto action = args.value("action", std::string("initialize"));
      if (action == "initialize")
        cube->initialize_grasps();
      else if (action == "execute") {
        auto moves = split_moves(args.at("moves").get<std::string>());
        robot_moves(moves, args);
      } else if (action == "execute_primitives") {
        std::vector<Primitive> plan;
        const auto& items = args.at("actions");
        if (!items.is_array() || items.size() > 10000)
          throw std::invalid_argument("actions must be array of at most 10000 primitives");
        for (const auto& item : items) plan.push_back(parse_primitive(item.get<std::string>()));
        execute_primitives(*cube, plan, robot_snapshot(*cube));
      } else
        throw std::invalid_argument(
            "Gripper action must be initialize, execute or execute_primitives");
      return cube->report();
    }
    throw std::invalid_argument("Unknown cube command: " + cmd);
  }
};
}  // namespace
int main(int argc, char** argv) {
  try {
    auto root = rm::repo_root(argc, argv);
    bool rpc = false, dual = false, solve = false, record = false, viewer = false;
    double speed = 1, playback = 4;
    bool plan_only = false;
    Json search_args = Json::object();
    std::optional<double> ws, js;
    std::string scramble = "R U F' L2 D B R' U2 F D'";
    std::filesystem::path output = root / "output/native-cube", exportxml;
    for (int i = 1; i < argc; i++) {
      std::string a = argv[i];
      auto value = [&]() {
        if (++i >= argc) throw std::invalid_argument("Missing value for " + a);
        return std::string(argv[i]);
      };
      auto numeric = [&]() {
        auto text = value();
        size_t used = 0;
        double result = std::stod(text, &used);
        if (used != text.size() || !std::isfinite(result))
          throw std::invalid_argument("Invalid numeric value for " + a);
        return result;
      };
      if (a == "--root")
        value();
      else if (a == "--rpc")
        rpc = true;
      else if (a == "--dual")
        dual = true;
      else if (a == "--headless") {
      } else if (a == "--viewer")
        viewer = true;
      else if (a == "--solve" || a == "--restore")
        solve = true;
      else if (a == "--search-ms")
        search_args["max_search_ms"] = numeric();
      else if (a == "--objective")
        search_args["objective"] = value();
      else if (a == "--terminal-policy")
        search_args["terminal_policy"] = value();
      else if (a == "--search-memory-mb") {
        double m = numeric();
        if (m != std::floor(m) || m < 4 || m > 4096)
          throw std::invalid_argument("Invalid search memory limit");
        search_args["memory_limit_mb"] = int(m);
      } else if (a == "--cost-profile") {
        auto path = value();
        std::ifstream in(path);
        if (!in) throw std::invalid_argument("Cannot open cost profile: " + path);
        in >> search_args["cost_profile"];
      } else if (a == "--plan-only") {
        plan_only = true;
        solve = true;
      } else if (a == "--record")
        record = true;
      else if (a == "--scramble" || a == "--moves")
        scramble = value();
      else if (a == "--speed")
        speed = numeric();
      else if (a == "--wrist-speed")
        ws = numeric();
      else if (a == "--jaw-speed")
        js = numeric();
      else if (a == "--playback")
        playback = numeric();
      else if (a == "--output")
        output = value();
      else if (a == "--export-xml")
        exportxml = value();
      else if (a == "--help") {
        std::cout << "物理魔方用法：\n"
                     "rm_cube --headless [--dual] [--scramble \"R U F'\"] [--solve] [--record] "
                     "[--wrist-speed 8 --jaw-speed 32] [--playback 4] [--output 输出目录]\n"
                     "rm_cube --root 仓库路径 --rpc [--dual]\n"
                     "窗口模式使用 --viewer，需要图形桌面。--dual 启用双夹爪。\n"
                     "--record 需要 FFmpeg。--solve 读取当前状态并求解。\n"
                     "双夹爪默认使用十二元动作搜索：--search-ms 1000 --objective "
                     "execution_time|action_count\n"
                     "--cost-profile 配置文件 --terminal-policy stable|home --search-memory-mb 64\n"
                     "--plan-only 只规划不执行还原。搜索耗时不计入目标；本版不支持动作重叠。\n";
        return 0;
      } else
        throw std::invalid_argument("Unknown option: " + a);
    }
    if (!std::isfinite(playback) || playback <= 0 || playback > 1000)
      throw std::invalid_argument("Invalid playback rate");
    App app(root, dual, speed, ws, js);
    if (plan_only && rpc)
      throw std::invalid_argument("--plan-only is a CLI option; use RPC plan command");
    app.search_defaults = search_args;
    app.options(Json::object());
    if ((viewer || !dual) && (!search_args.empty() || plan_only))
      throw std::invalid_argument("Robot search options require --dual without --viewer");
    if (rpc)
      return rm::rpc_loop(
          [&](const std::string& c, const rm::Json& a) { return app.handle(c, a); });
    auto moves = rm::cube::split_moves(scramble);
    if (viewer) {
      int status = rm::cube::interactive_viewer(*app.cube, root, moves);
      std::cout << app.cube->report().dump(2) << '\n';
      return status;
    }
    std::filesystem::create_directories(output);
    std::ofstream(output / "scene.xml") << app.cube->xml;
    if (!exportxml.empty()) {
      if (exportxml.has_parent_path()) std::filesystem::create_directories(exportxml.parent_path());
      std::ofstream(exportxml) << app.cube->xml;
    }
    Recorder writer;
    double next_frame = app.cube->data->time;
    rm::cube::Cube::Callback capture;
    if (record) {
      writer.open(output / "demo.mp4");
      capture = [&](rm::cube::Cube& c) {
        if (c.data->time + 1e-8 < next_frame) return;
        auto rgb = app.frame();
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        std::string status = c.active_face.empty() ? (c.robot_ready ? "Friction gripper execution"
                                                                    : "Cube simulation")
                                                   : "Turn " + c.active_face;
        cv::putText(bgr, status, {20, 35}, cv::FONT_HERSHEY_SIMPLEX, .7, {230, 230, 230}, 1,
                    cv::LINE_AA);
        writer.write(bgr);
        next_frame += playback / 30;
      };
    }
    try {
      for (auto& m : moves) app.cube->turn(m, capture);
      write_json(output / "controller_profile.json", {{"speed", speed},
                                                      {"wrist_speed", app.cube->wrist_speed},
                                                      {"jaw_speed", app.cube->jaw_speed},
                                                      {"timestep_s", app.cube->model->opt.timestep},
                                                      {"fast_parallel_fingers", app.cube->fast},
                                                      {"video_playback", playback},
                                                      {"grasp_model", "friction only"}});
      std::string state = app.cube->facelets();
      std::optional<RobotSearchResult> robot_result;
      auto opts = app.options(Json::object());
      if (solve && dual)
        robot_result = search_robot_solution(state, robot_snapshot(*app.cube), opts, root);
      if (robot_result) {
        write_json(output / "robot_plan.json", robot_result->json(opts));
        if (!robot_result->found)
          throw std::runtime_error(
              "No robot solution within search budget; inspect robot_plan.json");
      }
      auto solution = robot_result
                          ? primitive_moves(robot_result->plan.actions, robot_result->start)
                      : solve ? rm::cube::solve_facelets(state, root)
                              : std::vector<std::string>{};
      auto raw = rm::cube::compile_moves(solution), plan = rm::cube::optimize_plan(raw);
      write_json(output / "plan_unoptimized.json", rm::cube::plan_json(raw));
      if (robot_result) {
        auto description = robot_result->json(opts);
        description["initial_facelets"] = state;
        write_json(output / "plan.json", description);
        write_json(output / "optimization.json",
                   {{"objective", opts.action_count ? "action_count" : "execution_time"},
                    {"primitive_actions", robot_result->plan.actions.size()},
                    {"estimated_execution_s", robot_result->plan.execution_s},
                    {"search_ms", robot_result->search_ms}});
        std::ofstream csv(output / "steps.csv");
        csv << "index,primitive,mode,move,estimated_duration_s\n";
        auto rows = description["actions"];
        for (size_t i = 0; i < rows.size(); ++i)
          csv << i << ',' << rows[i]["action"].get<std::string>() << ','
              << rows[i]["mode"].get<std::string>() << ',' << rows[i]["move"].get<std::string>()
              << ',' << rows[i]["duration_s"].get<double>() << '\n';
      } else {
        write_json(
            output / "plan.json",
            {{"initial_facelets", state}, {"solution", solution}, {"actions", plan_json(plan)}});
        write_json(output / "optimization.json",
                   {{"raw_actions", raw.size()}, {"optimized_actions", plan.size()}});
        std::ofstream csv(output / "steps.csv");
        csv << "index,kind,hand,target,mode,move\n";
        for (size_t i = 0; i < plan.size(); ++i) {
          auto& a = plan[i];
          csv << i << ',' << a.kind << ',' << a.hand << ',' << a.target << ',' << a.mode << ','
              << a.move << '\n';
        }
      }
      if (solve && !plan_only) {
        if (dual) {
          app.cube->initialize_grasps(capture);
          execute_primitives(*app.cube, robot_result->plan.actions, robot_result->start, capture);
        } else
          for (auto& m : solution) app.cube->turn(m, capture);
      }
      app.cube->advance(.5, capture);
      if (solve && !plan_only && app.cube->facelets() != rm::cube::solved)
        throw std::runtime_error("Physical cube did not finish solved");
      write_json(output / "verification.json", app.cube->report());
      write_json(output / "grasp_checks.json", app.cube->grasp_checks);
      write_json(output / "motion_checks.json", app.cube->motion_checks);
      writer.release();
      std::cout << app.cube->report().dump(2) << '\n';
    } catch (const std::exception& e) {
      write_json(output / "failure.json", {{"error", e.what()}, {"state", app.cube->report()}});
      try {
        writer.release();
      } catch (const std::exception& encoding) {
        std::cerr << encoding.what() << '\n';
      }
      throw;
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "rm_cube: " << e.what() << '\n';
    return 1;
  }
}
