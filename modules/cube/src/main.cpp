#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>

#include "rm/cube.hpp"
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
        cube->execute(optimize_plan(compile_moves(moves, cube->orientation)));
      else
        for (auto& m : moves) cube->turn(m);
      return cube->report();
    }
    if (cmd == "solve" || cmd == "plan") {
      auto state = args.value("facelets", cube->facelets());
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
        cube->execute(optimize_plan(compile_moves(moves, cube->orientation)));
      } else
        throw std::invalid_argument(
            "Gripper action must be initialize or execute; raw unsupported actions are rejected");
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
      else if (a == "--record")
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
                     "--record 需要 FFmpeg。--solve 读取当前状态并求解。\n";
        return 0;
      } else
        throw std::invalid_argument("Unknown option: " + a);
    }
    if (!std::isfinite(playback) || playback <= 0 || playback > 1000)
      throw std::invalid_argument("Invalid playback rate");
    App app(root, dual, speed, ws, js);
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
      auto solution = solve ? rm::cube::solve_facelets(state, root) : std::vector<std::string>{};
      auto raw = rm::cube::compile_moves(solution), plan = rm::cube::optimize_plan(raw);
      write_json(output / "plan_unoptimized.json", rm::cube::plan_json(raw));
      write_json(output / "plan.json", {{"initial_facelets", state},
                                        {"solver", "muodov/kociemba 1.2.1 C"},
                                        {"solution", solution},
                                        {"actions", rm::cube::plan_json(plan)}});
      write_json(output / "optimization.json", {{"raw_actions", raw.size()},
                                                {"optimized_actions", plan.size()},
                                                {"wrist_order_preserved", true}});
      std::ofstream csv(output / "steps.csv");
      csv << "index,kind,hand,target,mode,move\n";
      for (size_t i = 0; i < plan.size(); i++) {
        auto& a = plan[i];
        csv << i << ',' << a.kind << ',' << a.hand << ',' << a.target << ',' << a.mode << ','
            << a.move << '\n';
      }
      if (solve) {
        if (dual) {
          app.cube->initialize_grasps(capture);
          app.cube->execute(plan, capture);
        } else
          for (auto& m : solution) app.cube->turn(m, capture);
      }
      app.cube->advance(.5, capture);
      if (solve && app.cube->facelets() != rm::cube::solved)
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
