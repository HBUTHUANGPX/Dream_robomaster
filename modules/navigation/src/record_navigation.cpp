#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

#include "rm/navigation.hpp"
namespace {
class Video {
  int input = -1;
  pid_t child = -1;

 public:
  Video(const std::filesystem::path& output, double playback) {
    int fds[2];
    if (pipe(fds)) throw std::runtime_error("Cannot create video pipe");
    std::string rate = std::to_string(10 * playback), file = output.string();
    child = fork();
    if (child < 0) {
      close(fds[0]);
      close(fds[1]);
      throw std::runtime_error("Cannot start ffmpeg");
    }
    if (child == 0) {
      dup2(fds[0], STDIN_FILENO);
      close(fds[0]);
      close(fds[1]);
      execlp("ffmpeg", "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", "1280x720", "-r", rate.c_str(), "-i", "-", "-an", "-c:v", "libx264", "-preset",
             "fast", "-crf", "21", "-pix_fmt", "yuv420p", "-movflags", "+faststart", file.c_str(),
             static_cast<char*>(nullptr));
      _exit(127);
    }
    close(fds[0]);
    input = fds[1];
    std::signal(SIGPIPE, SIG_IGN);
  }
  void frame(const cv::Mat& image) {
    if (!image.isContinuous() || image.type() != CV_8UC3 || image.cols != 1280 || image.rows != 720)
      throw std::invalid_argument("Video frame shape mismatch");
    size_t remaining = image.total() * 3;
    const auto* data = image.data;
    while (remaining) {
      auto n = write(input, data, remaining);
      if (n <= 0) throw std::runtime_error("ffmpeg pipe failed");
      data += n;
      remaining -= size_t(n);
    }
  }
  void finish() {
    if (input >= 0) {
      close(input);
      input = -1;
    }
    if (child > 0) {
      int status;
      waitpid(child, &status, 0);
      child = -1;
      if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
        throw std::runtime_error("ffmpeg encoding failed");
    }
  }
  ~Video() {
    try {
      finish();
    } catch (...) {
    }
  }
};
}  // namespace
int main(int argc, char** argv) {
  try {
    auto root = rm::repo_root(argc, argv);
    std::filesystem::path output = root / "output/navigation_omni_demo.mp4";
    int selected = 2;
    double yaw = 0, playback = 1, seconds = 80;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto next = [&]() {
        if (++i >= argc) throw std::invalid_argument("Missing option argument");
        return std::string(argv[i]);
      };
      if (arg == "--root")
        next();
      else if (arg == "--output")
        output = next();
      else if (arg == "--case") {
        double value = rm::nav::parse_number(next());
        if (value != std::floor(value) || value < 0 || value >= 7)
          throw std::invalid_argument("Invalid preset index");
        selected = int(value);
      } else if (arg == "--yaw-rate")
        yaw = rm::nav::parse_number(next());
      else if (arg == "--playback")
        playback = rm::nav::parse_number(next());
      else if (arg == "--seconds")
        seconds = rm::nav::parse_number(next());
      else
        throw std::invalid_argument("Unknown option: " + arg);
    }
    if (playback <= 0 || playback > 4 || seconds <= 0 || seconds > 86400)
      throw std::invalid_argument("Invalid playback or duration");
    if (!output.parent_path().empty()) std::filesystem::create_directories(output.parent_path());
    rm::nav::Task task(root);
    auto goal = task.presets()[selected];
    task.set_goal(goal["x"], goal["y"]);
    task.set_yaw_rate(yaw);
    Video video(output, playback);
    rm::Renderer renderer(task.simulation.model, 920, 600);
    auto& terrain = task.terrain;
    cv::Mat map(terrain.rows, terrain.cols, CV_8UC3, cv::Scalar(17, 25, 35));
    for (int r = 0; r < terrain.rows; ++r)
      for (int c = 0; c < terrain.cols; ++c) {
        int cell = r * terrain.cols + c;
        if (terrain.reachable[cell]) {
          double h = terrain.heights[cell];
          map.at<cv::Vec3b>(terrain.rows - 1 - r, c) = cv::Vec3b(
              cv::saturate_cast<uchar>(45 + 90 * h), cv::saturate_cast<uchar>(100 + 70 * h),
              cv::saturate_cast<uchar>(95 + 40 * h));
        }
      }
    cv::resize(map, map, cv::Size(300, 555), 0, 0, cv::INTER_NEAREST);
    auto xy = [&](rm::nav::V3 p) {
      return cv::Point(int(20 + p.x() / terrain.width * 300),
                       int(110 + (terrain.height - p.y()) / terrain.height * 555));
    };
    mjvCamera camera;
    mjv_defaultCamera(&camera);
    camera.distance = 2.4;
    camera.elevation = -33;
    camera.azimuth = 140;
    mjvOption option;
    mjv_defaultOption(&option);
    option.geomgroup[3] = 0;
    int arrived = 0, count = 0;
    cv::Mat frame;
    for (int step = 0; step < int(std::ceil(seconds * 10)); ++step) {
      task.step();
      for (int i = 0; i < 3; ++i) camera.lookat[i] = task.mapper->pose[i] + (i == 2 ? .1 : 0);
      renderer.update(task.simulation.data, camera, &option);
      auto& scene = renderer.scene();
      for (int which = 0; which < 2; ++which) {
        const auto& path = which ? task.local : task.path;
        float color[4] = {which ? 1.f : .1f, which ? .5f : .85f, which ? .1f : 1.f, 1};
        for (size_t i = 1; i < path.size() && scene.ngeom < scene.maxgeom; ++i) {
          auto& g = scene.geoms[scene.ngeom++];
          mjtNum zero[3] = {0, 0, 0}, identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
          mjv_initGeom(&g, mjGEOM_CAPSULE, zero, zero, identity, color);
          rm::nav::V3 a = path[i - 1] + rm::nav::V3(0, 0, .05),
                      b = path[i] + rm::nav::V3(0, 0, .05);
          mjv_connector(&g, mjGEOM_CAPSULE, .018, a.data(), b.data());
        }
      }
      frame = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(10, 17, 27));
      map.copyTo(frame(cv::Rect(20, 110, 300, 555)));
      renderer.read().copyTo(frame(cv::Rect(340, 85, 920, 600)));
      cv::putText(frame, "RMUC 2023 | MID-360 NAVIGATION", {22, 38}, cv::FONT_HERSHEY_SIMPLEX, .7,
                  {210, 239, 246}, 1, cv::LINE_AA);
      cv::putText(frame, "Omnidirectional DWA + PID | LiDAR / encoder localization", {22, 70},
                  cv::FONT_HERSHEY_SIMPLEX, .5, {142, 183, 198}, 1, cv::LINE_AA);
      for (int which = 0; which < 3; ++which) {
        const auto& path = which == 0 ? task.path : (which == 1 ? task.trail : task.local);
        cv::Scalar color =
            which == 0 ? cv::Scalar(65, 215, 245)
                       : (which == 1 ? cv::Scalar(245, 245, 245) : cv::Scalar(255, 174, 100));
        for (size_t i = 1; i < path.size(); ++i)
          cv::line(frame, xy(path[i - 1]), xy(path[i]), color, which == 2 ? 4 : 2, cv::LINE_AA);
      }
      cv::circle(frame, xy(task.mapper->pose.head<3>()), 5, {255, 255, 255}, -1);
      cv::circle(frame, xy(rm::nav::V3(goal["x"], goal["y"], goal["z"])), 5, {255, 170, 90}, -1);
      std::string status = task.status == "已到达"
                               ? "ARRIVED"
                               : (task.paused || !task.goal ? "STOPPED" : "NAVIGATING");
      cv::putText(frame, status, {355, 115}, cv::FONT_HERSHEY_SIMPLEX, .7, {20, 40, 45}, 2,
                  cv::LINE_AA);
      std::ostringstream telemetry;
      telemetry << std::fixed << std::setprecision(2) << "t=" << task.simulation.data->time
                << "s v=" << task.measured_velocity.head<2>().norm()
                << "m/s yaw=" << task.sensors.velocity.z() << "rad/s P=" << std::setprecision(0)
                << task.power_w << "/" << task.drive.budget << "W";
      cv::putText(frame, telemetry.str(), {350, 705}, cv::FONT_HERSHEY_SIMPLEX, .5, {215, 233, 240},
                  1, cv::LINE_AA);
      cv::putText(frame, "Global / Local / Estimated trail", {20, 700}, cv::FONT_HERSHEY_SIMPLEX,
                  .42, {173, 195, 205}, 1, cv::LINE_AA);
      video.frame(frame);
      ++count;
      if (!task.goal) {
        if (task.status != "已到达") break;
        if (++arrived >= 30) break;
      }
      if (task.paused) break;
    }
    video.finish();
    double error =
        (Eigen::Map<rm::nav::V2>(task.simulation.data->qpos) - rm::nav::V2(goal["x"], goal["y"]))
            .norm();
    bool warnings = false;
    for (int i = 0; i < mjNWARNING; ++i) warnings |= task.simulation.data->warning[i].number != 0;
    auto metadata = output;
    metadata.replace_extension(".json");
    std::ofstream out(metadata);
    out << rm::Json({{"goal", goal},
                     {"truth_error_xy", error},
                     {"power_peak_w", task.power_peak_w},
                     {"playback", playback},
                     {"yaw_rate_request", yaw},
                     {"frames", count},
                     {"status", task.status},
                     {"warnings", warnings}})
               .dump(2)
        << '\n';
    auto image = output;
    image.replace_extension(".png");
    cv::Mat bgr;
    cv::cvtColor(frame, bgr, cv::COLOR_RGB2BGR);
    if (!cv::imwrite(image.string(), bgr)) throw std::runtime_error("Snapshot failed");
    std::cout << rm::Json({{"output", output.string()}, {"frames", count}, {"status", task.status}})
                     .dump()
              << '\n';
    return arrived && error <= .2 && !warnings ? 0 : 2;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
