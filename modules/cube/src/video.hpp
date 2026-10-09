#pragma once
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <filesystem>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
namespace rm::cube {
struct Recorder {
  int fd = -1;
  pid_t pid = -1;
  void open(const std::filesystem::path& output, int width = 960, int height = 720, int fps = 30) {
    const auto dimensions = std::to_string(width) + "x" + std::to_string(height),
               frame_rate = std::to_string(fps);
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
             "-s", dimensions.c_str(), "-r", frame_rate.c_str(), "-i", "-", "-an", "-c:v",
             "libx264", "-preset", "fast", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags",
             "+faststart", output.c_str(), static_cast<char*>(nullptr));
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
}  // namespace rm::cube
