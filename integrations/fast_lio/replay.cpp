// Native supervisor for the unmodified ROS1 FAST-LIO offline integration.
// Process lifecycle self-tests compile without ROS; production bag validation
// and master queries are compiled only in the ROS target.
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#ifndef RM_REPLAY_SUPERVISOR_SELF_TEST
#include <nav_msgs/Odometry.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/this_node.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <xmlrpcpp/XmlRpcValue.h>
#endif

namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
volatile sig_atomic_t interruption = 0;

void interrupted(int signal) { interruption = signal; }

void install_handler(int signal, void (*handler)(int)) {
  struct sigaction action{};
  action.sa_handler = handler;
  sigemptyset(&action.sa_mask);
  if (sigaction(signal, &action, nullptr) != 0)
    throw std::system_error(errno, std::generic_category(), "sigaction");
}

void check_interrupted() {
  if (interruption)
    throw std::runtime_error("Replay interrupted by signal " + std::to_string(interruption));
}

class Process {
 public:
  Process() = default;
  Process(const Process&) = delete;
  Process& operator=(const Process&) = delete;

  ~Process() {
    // Explicit ordered cleanup is used on the normal/error paths. This guard
    // also prevents abandoned children if allocation or logging throws there.
    if (pid_ > 0 && !cleaned_) {
      try {
        stop();
      } catch (const std::exception& error) {
        std::cerr << "Cleanup: " << error.what() << '\n';
      }
    }
  }

  void start(const std::vector<std::string>& arguments) {
    if (arguments.empty() || pid_ > 0) throw std::logic_error("Invalid process start");
    std::vector<char*> argv;
    for (const auto& arg : arguments) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);
    int pipe_fds[2];
    if (pipe2(pipe_fds, O_CLOEXEC) != 0)
      throw std::system_error(errno, std::generic_category(), "exec status pipe");
    const pid_t child = fork();
    if (child == -1) {
      const int error = errno;
      close(pipe_fds[0]);
      close(pipe_fds[1]);
      throw std::system_error(error, std::generic_category(), "fork");
    }
    if (child == 0) {
      close(pipe_fds[0]);
      // Only async-signal-safe operations occur between fork and exec. roscpp
      // may have threads even though this supervisor owns no subscriptions.
      struct sigaction action{};
      action.sa_handler = SIG_DFL;
      sigemptyset(&action.sa_mask);
      sigaction(SIGINT, &action, nullptr);
      sigaction(SIGTERM, &action, nullptr);
      if (setsid() != -1) execvp(argv[0], argv.data());
      const int error = errno;
      const auto ignored = write(pipe_fds[1], &error, sizeof(error));
      (void)ignored;
      _exit(127);
    }
    pid_ = child;
    name_ = arguments.front();
    close(pipe_fds[1]);
    int error = 0;
    ssize_t count;
    do {
      count = read(pipe_fds[0], &error, sizeof(error));
    } while (count < 0 && errno == EINTR);
    const int read_error = errno;
    close(pipe_fds[0]);
    if (count > 0) {
      reap();
      cleaned_ = true;
      throw std::system_error(error, std::generic_category(), "Cannot execute " + name_);
    }
    if (count < 0) throw std::system_error(read_error, std::generic_category(), "exec status read");
    check_interrupted();
  }

  std::optional<int> poll() {
    if (pid_ <= 0) return {};
    if (status_) return status_;
    int status = 0;
    pid_t result;
    do {
      result = waitpid(pid_, &status, WNOHANG);
    } while (result < 0 && errno == EINTR);
    if (result < 0) throw std::system_error(errno, std::generic_category(), "waitpid " + name_);
    if (result == pid_) status_ = decode(status);
    return status_;
  }

  void stop(std::chrono::milliseconds grace = 15s) {
    if (pid_ <= 0 || cleaned_) return;
    poll();
    // Keep the process group identity after its leader exits: lingering
    // descendants must not survive an early roslaunch/player failure.
    if (!group_alive()) {
      reap();
      cleaned_ = true;
      return;
    }
    send(SIGINT);
    const auto deadline = Clock::now() + grace;
    while (Clock::now() < deadline) {
      poll();
      if (!group_alive()) {
        reap();
        cleaned_ = true;
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
    send(SIGKILL);
    reap();
    cleaned_ = true;
    throw std::runtime_error(name_ + " did not shut down cleanly; process group killed");
  }

 private:
  pid_t pid_ = -1;
  std::optional<int> status_;
  bool cleaned_ = false;
  std::string name_;

  static int decode(int status) {
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
    throw std::runtime_error("Unexpected child wait status");
  }

  bool group_alive() const {
    if (kill(-pid_, 0) == 0) return true;
    if (errno == ESRCH) return false;
    throw std::system_error(errno, std::generic_category(), "process group status " + name_);
  }

  void send(int signal) const {
    if (kill(-pid_, signal) != 0 && errno != ESRCH)
      throw std::system_error(errno, std::generic_category(), "signal process group " + name_);
  }

  void reap() {
    if (status_) return;
    int status = 0;
    pid_t result;
    do {
      result = waitpid(pid_, &status, 0);
    } while (result < 0 && errno == EINTR);
    if (result < 0) throw std::system_error(errno, std::generic_category(), "reap " + name_);
    status_ = decode(status);
  }
};

void require_running(const std::vector<Process*>& processes, const std::string& description) {
  check_interrupted();
  for (auto* process : processes)
    if (auto code = process->poll())
      throw std::runtime_error("ROS process exited while " + description + ": " +
                               std::to_string(*code));
}

void wait_ready(const std::function<bool()>& predicate, const std::vector<Process*>& processes,
                const std::string& description, std::chrono::milliseconds timeout = 30s) {
  const auto deadline = Clock::now() + timeout;
  while (Clock::now() < deadline) {
    require_running(processes, "waiting for " + description);
    if (predicate()) return;
    std::this_thread::sleep_for(100ms);
  }
  throw std::runtime_error("Timed out waiting for " + description);
}

// All three stop calls execute, even if SIGKILL fallback or another failure
// occurs. Recorder indexing completes before the mapping/master are stopped.
std::exception_ptr stop_all(Process& player, Process& recorder, Process& launch,
                            std::chrono::milliseconds grace = 15s) {
  std::exception_ptr first_error;
  for (auto* process : {&player, &recorder, &launch}) {
    try {
      process->stop(grace);
    } catch (const std::exception& error) {
      std::cerr << "Cleanup: " << error.what() << '\n';
      if (!first_error) first_error = std::current_exception();
    }
  }
  return first_error;
}

#ifndef RM_REPLAY_SUPERVISOR_SELF_TEST
const std::filesystem::path input = "/data/input.bag";
const std::filesystem::path output = "/data/lio_output.bag";

void validate_input() {
  if (!std::filesystem::is_regular_file(input)) throw std::runtime_error("Missing /data/input.bag");
  if (std::filesystem::exists(output) || std::filesystem::exists(output.string() + ".active"))
    throw std::runtime_error("Refusing to overwrite previous recording");
  rosbag::Bag bag(input.string(), rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{"/sim/lidar", "/sim/imu"}));
  uint64_t lidar_count = 0, imu_count = 0;
  for (const auto& instance : view) {
    check_interrupted();
    if (instance.getTopic() == "/sim/lidar") {
      auto cloud = instance.instantiate<sensor_msgs::PointCloud2>();
      if (!cloud || instance.getDataType() != "sensor_msgs/PointCloud2")
        throw std::runtime_error("Invalid /sim/lidar type; expected sensor_msgs/PointCloud2");
      for (const std::string field_name : {"x", "y", "z", "intensity"}) {
        auto field = std::find_if(cloud->fields.begin(), cloud->fields.end(),
                                  [&](const auto& field) { return field.name == field_name; });
        if (field == cloud->fields.end() || field->datatype != sensor_msgs::PointField::FLOAT32 ||
            field->count != 1 || uint64_t(field->offset) + sizeof(float) > cloud->point_step)
          throw std::runtime_error("MARSIM requires valid float32 PointXYZI fields");
      }
      if (uint64_t(cloud->row_step) < uint64_t(cloud->width) * cloud->point_step ||
          uint64_t(cloud->row_step) * cloud->height != cloud->data.size())
        throw std::runtime_error("Invalid PointCloud2 row/data layout");
      ++lidar_count;
    } else {
      if (instance.getDataType() != "sensor_msgs/Imu" || !instance.instantiate<sensor_msgs::Imu>())
        throw std::runtime_error("Invalid /sim/imu type; expected sensor_msgs/Imu");
      ++imu_count;
    }
  }
  if (!lidar_count || !imu_count)
    throw std::runtime_error("Missing or empty /sim/lidar or /sim/imu");
  std::cout << "Input: /sim/lidar=" << lidar_count << ", /sim/imu=" << imu_count << std::endl;
}

bool has_topic(XmlRpc::XmlRpcValue& state, int section, const std::string& topic,
               const std::string& node = "") {
  if (state.getType() != XmlRpc::XmlRpcValue::TypeArray || state.size() != 3 ||
      state[section].getType() != XmlRpc::XmlRpcValue::TypeArray)
    return false;
  auto& list = state[section];
  for (int i = 0; i < list.size(); ++i) {
    auto& entry = list[i];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() != 2 ||
        entry[0].getType() != XmlRpc::XmlRpcValue::TypeString ||
        entry[1].getType() != XmlRpc::XmlRpcValue::TypeArray)
      continue;
    if (static_cast<std::string>(entry[0]) != topic) continue;
    if (node.empty()) return entry[1].size() > 0;
    for (int j = 0; j < entry[1].size(); ++j)
      if (entry[1][j].getType() == XmlRpc::XmlRpcValue::TypeString &&
          static_cast<std::string>(entry[1][j]) == node)
        return true;
  }
  return false;
}

bool ready(bool recorder) {
  XmlRpc::XmlRpcValue args, response, state;
  args[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", args, response, state, false)) return false;
  if (recorder) return has_topic(state, 1, "/Odometry", "/lio_recorder");
  return has_topic(state, 1, "/sim/lidar", "/laserMapping") &&
         has_topic(state, 1, "/sim/imu", "/laserMapping") && has_topic(state, 0, "/Odometry");
}

void validate_output() {
  if (std::filesystem::exists(output.string() + ".active"))
    throw std::runtime_error("Recorder left an unfinalized .active bag");
  rosbag::Bag bag(output.string(), rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(std::string("/Odometry")));
  uint64_t count = 0;
  std::optional<ros::Time> first_stamp, last_stamp;
  for (const auto& instance : view) {
    check_interrupted();
    auto message = instance.instantiate<nav_msgs::Odometry>();
    if (!message || instance.getDataType() != "nav_msgs/Odometry")
      throw std::runtime_error("Invalid /Odometry message type");
    const auto& p = message->pose.pose.position;
    const auto& q = message->pose.pose.orientation;
    for (double value : {p.x, p.y, p.z, q.x, q.y, q.z, q.w})
      if (!std::isfinite(value)) throw std::runtime_error("Nonfinite odometry");
    const auto stamp = message->header.stamp;
    if (last_stamp && stamp < *last_stamp) throw std::runtime_error("Odometry stamps regress");
    if (!first_stamp) first_stamp = stamp;
    last_stamp = stamp;
    ++count;
  }
  if (!count) throw std::runtime_error("FAST-LIO produced no /Odometry messages");
  std::cout << std::setprecision(12) << "Recorded " << count << " finite odometry messages, stamps "
            << first_stamp->toSec() << ".." << last_stamp->toSec()
            << ". This is a recording check, not an accuracy evaluation." << std::endl;
}

void replay() {
  validate_input();
  Process launch, recorder, player;
  std::exception_ptr failure;
  try {
    launch.start({"roslaunch", "/opt/fast_lio/offline.launch"});
    wait_ready([] { return ready(false); }, {&launch}, "mapping subscribers");
    recorder.start(
        {"rosbag", "record", "-O", output.string(), "/Odometry", "__name:=lio_recorder"});
    wait_ready([] { return ready(true); }, {&launch, &recorder}, "odometry recorder");
    // Only playback pacing and /clock change; original message header stamps
    // and payloads are passed through unchanged by the upstream rosbag player.
    player.start({"rosbag", "play", "--clock", "--rate", "0.5", "--delay", "2", input.string(),
                  "--topics", "/sim/lidar", "/sim/imu"});
    while (!player.poll()) {
      require_running({&launch, &recorder}, "playing input");
      std::this_thread::sleep_for(100ms);
    }
    check_interrupted();
    if (*player.poll() != 0)
      throw std::runtime_error("rosbag play failed: " + std::to_string(*player.poll()));
    // Simulated time stops at EOF. Drain callbacks with a wall-clock deadline.
    const auto deadline = Clock::now() + 5s;
    while (Clock::now() < deadline) {
      require_running({&launch, &recorder}, "draining playback");
      std::this_thread::sleep_for(100ms);
    }
  } catch (...) {
    failure = std::current_exception();
  }
  const auto cleanup_failure = stop_all(player, recorder, launch);
  if (failure) std::rethrow_exception(failure);
  if (cleanup_failure) std::rethrow_exception(cleanup_failure);
  check_interrupted();
  validate_output();
}
#else
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error("Self-test: " + message);
}

int helper(int argc, char** argv) {
  if (argc != 3) throw std::runtime_error("Invalid self-test helper arguments");
  const std::string mode = argv[1];
  if (mode == "--exit") return std::stoi(argv[2]);
  if (mode != "--wait" && mode != "--ignore" && mode != "--group")
    throw std::runtime_error("Unknown self-test helper");
  install_handler(SIGINT, mode == "--ignore" ? SIG_IGN : interrupted);
  install_handler(SIGTERM, interrupted);
  // A readiness marker removes timing assumptions from the shutdown tests.
  const std::filesystem::path marker = argv[2];
  pid_t descendant = -1;
  if (mode == "--group") {
    descendant = fork();
    if (descendant < 0) throw std::system_error(errno, std::generic_category(), "test descendant");
    if (descendant == 0) {
      while (!interruption) std::this_thread::sleep_for(10ms);
      {
        std::ofstream done(marker.string() + ".descendant");
        done << "stopped\n";
      }
      _exit(0);
    }
  }
  {
    std::ofstream ready_file(marker.string() + ".ready");
    ready_file << "ready\n";
  }
  while (!interruption) std::this_thread::sleep_for(10ms);
  if (descendant > 0) {
    int status;
    while (waitpid(descendant, &status, 0) < 0 && errno == EINTR) {
    }
  }
  {
    std::ofstream finalized(marker);
    finalized << "finalized\n";
  }
  {
    std::ofstream order(marker.parent_path() / "order.log", std::ios::app);
    order << marker.filename().string() << '\n';
  }
  return 0;
}

void process_tests(const std::string& executable) {
  // TMPDIR may select writable /home storage on hosts with a full root disk.
  std::string pattern =
      (std::filesystem::temp_directory_path() / "replay-supervisor-XXXXXX").string();
  std::vector<char> directory(pattern.begin(), pattern.end());
  directory.push_back('\0');
  char* created = mkdtemp(directory.data());
  if (!created) throw std::system_error(errno, std::generic_category(), "mkdtemp");
  const std::filesystem::path work = created;
  struct RemoveTemporary {
    std::filesystem::path path;
    ~RemoveTemporary() {
      std::error_code error;
      std::filesystem::remove_all(path, error);
    }
  } cleanup{work};
  Process natural;
  natural.start({executable, "--exit", "7"});
  wait_ready([&] { return natural.poll().has_value(); }, {}, "natural child exit", 2s);
  require(natural.poll() == 7, "nonzero child exit preserved");
  bool exited_rejected = false;
  try {
    require_running({&natural}, "testing dependency exit");
  } catch (const std::runtime_error&) {
    exited_rejected = true;
  }
  require(exited_rejected, "failed dependency aborts readiness/playback");
  natural.stop(100ms);

  Process missing;
  bool rejected = false;
  try {
    missing.start({(work / "missing-program").string()});
  } catch (const std::system_error&) {
    rejected = true;
  }
  require(rejected, "exec failure propagated");

  Process launch, recorder, player;
  auto spawn = [&](Process& process, const std::string& mode, const std::string& label) {
    const auto marker = work / label;
    process.start({executable, mode, marker.string()});
    wait_ready([&] { return std::filesystem::exists(marker.string() + ".ready"); }, {&process},
               label + " helper", 2s);
  };
  spawn(launch, "--wait", "launch");
  spawn(recorder, "--wait", "recorder");
  spawn(player, "--ignore", "player");
  auto failure = stop_all(player, recorder, launch, 100ms);
  require(bool(failure), "unresponsive player reports SIGKILL fallback");
  require(player.poll() == 128 + SIGKILL, "SIGKILL fallback reaps child");
  require(std::filesystem::exists(work / "recorder") && std::filesystem::exists(work / "launch"),
          "cleanup continues and allows finalization after earlier failure");
  require(recorder.poll() == 0 && launch.poll() == 0, "ordered children stop cleanly");
  std::ifstream order(work / "order.log");
  std::string first, second;
  std::getline(order, first);
  std::getline(order, second);
  require(first == "recorder" && second == "launch", "recorder finalizes before mapper shutdown");

  Process group;
  spawn(group, "--group", "group");
  group.stop(1s);
  require(std::filesystem::exists(work / "group.descendant"),
          "SIGINT reaches process-group descendants");

  bool timed_out = false;
  try {
    wait_ready([] { return false; }, {}, "unavailable dependency", 1ms);
  } catch (const std::runtime_error&) {
    timed_out = true;
  }
  require(timed_out, "readiness timeout reported");
  Process cancelled_child;
  spawn(cancelled_child, "--wait", "cancelled");
  raise(SIGTERM);
  bool cancelled = false;
  try {
    check_interrupted();
  } catch (const std::runtime_error&) {
    cancelled = true;
  }
  require(cancelled, "signal cancellation reaches main cleanup path");
  cancelled_child.stop(1s);
  require(std::filesystem::exists(work / "cancelled"), "SIGTERM cancellation permits finalization");
  interruption = 0;
  std::cout << "Native process-supervisor tests passed; ROS bag/master APIs were not tested.\n";
}
#endif
}  // namespace

int main(int argc, char** argv) {
  try {
    install_handler(SIGINT, interrupted);
    install_handler(SIGTERM, interrupted);
#ifdef RM_REPLAY_SUPERVISOR_SELF_TEST
    if (argc > 1) return helper(argc, argv);
    process_tests(std::filesystem::canonical("/proc/self/exe").string());
#else
    if (argc != 1)
      throw std::invalid_argument(
          "Usage: fast_lio_replay (fixed /data/input.bag and /data/lio_output.bag)");
    ros::init(argc, argv, "offline_supervisor", ros::init_options::NoSigintHandler);
    replay();
    ros::shutdown();
#endif
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAST-LIO replay: " << error.what() << '\n';
    return interruption ? 128 + interruption : 1;
  }
}
