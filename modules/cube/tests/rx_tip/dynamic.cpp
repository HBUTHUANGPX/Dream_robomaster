#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstring>
#include <opencv2/imgproc.hpp>

#include "common.hpp"
#include "rm/sim.hpp"

using namespace rx_tip;
namespace {
// 与主程序 Recorder 相同的管道、子进程和回收流程；不经 shell 解释路径。
class Recorder {
 public:
  Recorder() = default;
  Recorder(const Recorder&) = delete;
  Recorder& operator=(const Recorder&) = delete;
  void open(const std::filesystem::path& output) {
    int pipefd[2];
    if (pipe(pipefd)) throw std::runtime_error("视频管道创建失败");
    pid_ = fork();
    if (pid_ < 0) {
      close(pipefd[0]);
      close(pipefd[1]);
      throw std::runtime_error("视频子进程创建失败");
    }
    if (pid_ == 0) {
      if (dup2(pipefd[0], STDIN_FILENO) < 0) _exit(126);
      if (pipefd[0] != STDIN_FILENO) close(pipefd[0]);
      close(pipefd[1]);
      execlp("ffmpeg", "ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pixel_format", "rgb24",
             "-video_size", "960x720", "-framerate", "30", "-i", "-", "-an", "-c:v", "libx264",
             "-preset", "fast", "-crf", "23", "-pix_fmt", "yuv420p", output.c_str(),
             static_cast<char*>(nullptr));
      _exit(127);
    }
    close(pipefd[0]);
    fd_ = pipefd[1];
    struct sigaction ignored{};
    ignored.sa_handler = SIG_IGN;
    sigemptyset(&ignored.sa_mask);
    if (sigaction(SIGPIPE, &ignored, &previous_)) throw std::runtime_error("设置视频管道信号失败");
    restore_signal_ = true;
  }
  void write(const cv::Mat& frame) {
    if (fd_ < 0 || frame.rows != 720 || frame.cols != 960 || frame.type() != CV_8UC3 ||
        !frame.isContinuous())
      throw std::runtime_error("视频帧必须为连续的 960×720 RGB24");
    const size_t total = frame.total() * frame.elemSize();
    size_t offset = 0;
    while (offset < total) {
      const auto count = ::write(fd_, frame.data + offset, total - offset);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0)
        throw std::runtime_error("视频帧写入失败: " + std::string(std::strerror(errno)));
      offset += static_cast<size_t>(count);
    }
  }
  int release() noexcept {
    int result = 0;
    if (fd_ >= 0) {
      if (close(fd_) < 0) result = -1;
      fd_ = -1;
    }
    if (pid_ > 0) {
      int status = 0;
      pid_t waited;
      do {
        waited = waitpid(pid_, &status, 0);
      } while (waited < 0 && errno == EINTR);
      pid_ = -1;
      if (waited < 0)
        result = -1;
      else if (!WIFEXITED(status))
        result = WIFSIGNALED(status) ? 128 + WTERMSIG(status) : -1;
      else if (WEXITSTATUS(status) != 0)
        result = WEXITSTATUS(status);
    }
    if (restore_signal_) {
      if (sigaction(SIGPIPE, &previous_, nullptr)) result = -1;
      restore_signal_ = false;
    }
    return result;
  }
  ~Recorder() { release(); }

 private:
  int fd_ = -1;
  pid_t pid_ = -1;
  struct sigaction previous_{};
  bool restore_signal_ = false;
};

bool valid_tag(const std::string& tag) {
  return !tag.empty() && std::all_of(tag.begin(), tag.end(), [](unsigned char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' ||
           c == '_';
  });
}

void run(const std::filesystem::path& out, int active, double goal, const std::string& stem, J& r) {
  const double shift = default_shift, targetA = default_grip, targetB = default_grip;
  Model model(out / (active ? "dual55B.xml" : "dual55.xml"));
  auto* m = model.model.get();
  auto* d = model.data.get();
  // 与原型初始化一致，读取场景后先执行一次正向计算。
  mj_forward(m, d);
  const double half = (.055 - .0007 * 55 / 60) / 2;
  const double cx = -.00189107 + (rz(angle(motor_for_gap(2 * half))) * (B - A) - (B - A)).x();
  const auto roots = hand_roots(m);
  if (std::abs(m->body_pos[3 * roots[0]] - (.055 / 3 + cx + shift)) > 1e-10 ||
      std::abs(m->body_pos[3 * roots[1] + 1] - (-.055 / 3 - cx - shift)) > 1e-10)
    throw std::runtime_error("XML 安装位置不正确；请先用 geometry 重新生成场景");
  m->opt.timestep = .0005;
  conf(m, d, "l", -2.25);
  conf(m, d, "r", -2.25);
  auto aid = [&](const char* n) { return id(m, mjOBJ_ACTUATOR, n); };
  auto jid = [&](const char* n) { return id(m, mjOBJ_JOINT, n); };
  const int wa = aid(active ? "B_yaw_drive" : "A_yaw_drive"),
            wb = aid(active ? "A_yaw_drive" : "B_yaw_drive"), ja = jid(active ? "B_yaw" : "A_yaw"),
            jb = jid(active ? "A_yaw" : "B_yaw"), lj = jid("probe_layer_hinge"),
            cj = jid("probe_free"), ca = aid("l_gripper_motor_joint"),
            cb = aid("r_gripper_motor_joint");
  d->qpos[m->jnt_qposadr[jb]] = pi / 2;
  d->ctrl[wb] = pi / 2;
  mj_forward(m, d);
  std::vector<int> hand, part;
  classify(m, roots, hand, part);
  std::array<std::array<int, 2>, 2> pads{};
  for (int h = 0; h < 2; ++h)
    for (int f = 0; f < 2; ++f)
      pads[h][f] =
          id(m, mjOBJ_GEOM,
             std::string(h == 0 ? "l" : "r") + "_finger" + std::to_string(f + 1) + "_link_pad");
  const int reference_pad = pads[0][0];
  if (m->geom_type[reference_pad] != mjGEOM_BOX) throw std::runtime_error("指尖 pad 必须为 box");
  r["pad_mm"] = m->geom_size[3 * reference_pad] * 2000;
  r["pad_metadata"] = {
      {"source_geom", "l_finger1_link_pad"},
      {"full_size_mm",
       {m->geom_size[3 * reference_pad] * 2000, m->geom_size[3 * reference_pad + 1] * 2000,
        m->geom_size[3 * reference_pad + 2] * 2000}}};
  std::vector<bool> original_finger(m->ngeom, false);
  for (int g = 0; g < m->ngeom; ++g) {
    const char* name = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
    original_finger[g] = name && (std::string(name).ends_with("_finger1_link") ||
                                  std::string(name).ends_with("_finger2_link"));
  }
  const std::array<int, 2> wrist_joints{jid("A_yaw"), jid("B_yaw")};
  const std::array<int, 2> grip_joints{jid("l_gripper_motor_joint"), jid("r_gripper_motor_joint")};
  std::array<std::array<double, 2>, 2> min_pad;
  for (auto& value : min_pad) value.fill(std::numeric_limits<double>::infinity());
  std::array<double, 2> original_max{}, original_sum{}, peak_wrist{}, peak_grip{};
  std::array<int, 2> original_bearing_steps{}, original_contact_samples{};
  size_t load_samples = 0;
  rm::Renderer renderer(m, 960, 720);
  mjvCamera camera;
  mjv_defaultCamera(&camera);
  camera.lookat[2] = .22;
  camera.azimuth = 135;
  camera.elevation = -32;
  camera.distance = .36;
  mjvOption opt;
  mjv_defaultOption(&opt);
  opt.geomgroup[3] = 0;
  opt.sitegroup[4] = 0;
  Recorder video;
  J rows = J::array();
  int maxself = 0, maxcross = 0, warnings = 0, maxwrong = 0, completed_steps = 0;
  double peakforce = 0, maxdrop = 0, maxup = 0, maxtranslation = 0, maxangle = 0, maxwrongdepth = 0,
         maxwrongforce = 0;
  J firstwrong;
  const int layerbody = id(m, mjOBJ_BODY, "probe_layer");
  std::string stage, failure;
  bool video_started = false;
  try {
    video.open(out / (stem + ".mp4"));
    video_started = true;
    for (int step = 0; step < 24000; ++step) {
      const double t = d->time, close = std::clamp(t / 2., 0., 1.);
      d->ctrl[ca] = -2.25 + (targetA + 2.25) * close;
      d->ctrl[cb] = -2.25 + (targetB + 2.25) * close;
      stage = t < 2   ? "closing"
              : t < 4 ? "grip settling"
              : t < 6 ? "gravity hold"
              : t < 9 ? ("turn " + hand_name(active) + " " + std::to_string(int(goal)) + " deg")
                      : "hold turned layer";
      if (t >= 4) m->opt.gravity[2] = -9.81;
      double rot = std::clamp((t - 6) / 3., 0., 1.);
      rot = rot * rot * (3 - 2 * rot);
      d->ctrl[wa] = rot * goal * pi / 180;
      mj_step(m, d);
      ++completed_steps;
      if (!std::isfinite(d->time)) throw std::runtime_error("仿真时间不是有限数");
      for (int i = 0; i < m->nq; ++i)
        if (!std::isfinite(d->qpos[i])) throw std::runtime_error("仿真出现非有限位置");
      for (int i = 0; i < m->nv; ++i)
        if (!std::isfinite(d->qvel[i]) || !std::isfinite(d->qfrc_actuator[i]))
          throw std::runtime_error("仿真出现非有限速度或力矩");
      int self = 0, cross = 0, wrong = 0;
      double wrongdepth = 0, wrongforce = 0;
      std::string pair;
      double force[2] = {};
      std::array<std::array<double, 2>, 2> pad_force{};
      std::array<double, 2> original_force{};
      std::array<int, 2> original_contacts{};
      for (int c = 0; c < d->ncon; ++c) {
        const auto& k = d->contact[c];
        const int a = k.geom[0], b = k.geom[1];
        if (k.dist < -1e-5 && hand[a] >= 0 && hand[a] == hand[b] &&
            (extension(m, a) || extension(m, b)))
          ++self;
        if (k.dist < -1e-5 && hand[a] >= 0 && hand[b] >= 0 && hand[a] != hand[b]) ++cross;
        for (int h = 0; h < 2; ++h) {
          if (!((hand[a] == h && part[b] >= 0) || (hand[b] == h && part[a] >= 0))) continue;
          mjtNum f[6];
          mj_contactForce(m, d, c, f);
          if (!std::isfinite(f[0])) throw std::runtime_error("仿真出现非有限接触力");
          force[h] += f[0];
          const int hg = hand[a] == h ? a : b;
          for (int finger = 0; finger < 2; ++finger)
            if (hg == pads[h][finger]) pad_force[h][finger] += f[0];
          if (original_finger[hg]) {
            original_force[h] += f[0];
            if (f[0] > .01) ++original_contacts[h];
          }
          const int pg = part[a] >= 0 ? a : b;
          const bool islayer = m->geom_bodyid[pg] == layerbody;
          if (((h == active && !islayer) || (h != active && islayer)) && k.dist < -1e-6 &&
              f[0] > .01) {
            ++wrong;
            wrongforce += f[0];
            if (-k.dist > wrongdepth) {
              wrongdepth = -k.dist;
              pair = geom_name(m, a) + " / " + geom_name(m, b);
            }
          }
        }
      }
      // 承载统计只取已经启用重力的原型采样阶段；不改变接触或控制参数。
      if (t >= 4) {
        ++load_samples;
        for (int h = 0; h < 2; ++h) {
          for (int f = 0; f < 2; ++f) min_pad[h][f] = std::min(min_pad[h][f], pad_force[h][f]);
          original_max[h] = std::max(original_max[h], original_force[h]);
          original_sum[h] += original_force[h];
          original_bearing_steps[h] += original_force[h] > .01;
          original_contact_samples[h] += original_contacts[h];
        }
      }
      for (int h = 0; h < 2; ++h) {
        peak_wrist[h] =
            std::max(peak_wrist[h], std::abs(d->qfrc_actuator[m->jnt_dofadr[wrist_joints[h]]]));
        peak_grip[h] =
            std::max(peak_grip[h], std::abs(d->qfrc_actuator[m->jnt_dofadr[grip_joints[h]]]));
      }
      maxself = std::max(maxself, self);
      maxcross = std::max(maxcross, cross);
      peakforce = std::max({peakforce, force[0], force[1]});
      const int qa = m->jnt_qposadr[cj];
      const double drop = .22 - d->qpos[qa + 2];
      maxdrop = std::max(maxdrop, drop);
      maxup = std::max(maxup, -drop);
      const V pos(d->qpos[qa], d->qpos[qa + 1], d->qpos[qa + 2]);
      const double caa = 2 * acos(std::clamp(std::abs(d->qpos[qa + 3]), 0., 1.));
      maxangle = std::max(maxangle, caa);
      maxtranslation = std::max(maxtranslation, (pos - V(0, 0, .22)).norm());
      maxwrong = std::max(maxwrong, wrong);
      maxwrongdepth = std::max(maxwrongdepth, wrongdepth);
      maxwrongforce = std::max(maxwrongforce, wrongforce);
      if (wrong && firstwrong.is_null())
        firstwrong = {{"time_s", d->time},
                      {"pair", pair},
                      {"depth_mm", wrongdepth * 1000},
                      {"force_N", wrongforce}};
      if (step % 200 == 0)
        rows.push_back(
            {{"time_s", d->time},
             {"stage", stage},
             {"cube_z_m", d->qpos[qa + 2]},
             {"cube_position_m", {pos.x(), pos.y(), pos.z()}},
             {"cube_quat", {d->qpos[qa + 3], d->qpos[qa + 4], d->qpos[qa + 5], d->qpos[qa + 6]}},
             {"wrong_contacts", wrong},
             {"wrong_depth_mm", wrongdepth * 1000},
             {"wrong_force_N", wrongforce},
             {"active_wrist_rad", d->qpos[m->jnt_qposadr[ja]]},
             {"support_wrist_rad", d->qpos[m->jnt_qposadr[jb]]},
             {"layer_rad", d->qpos[m->jnt_qposadr[lj]]},
             {"forces_N", {force[0], force[1]}},
             {"newpad_forces_N",
              {{"active", pad_force[active]}, {"support", pad_force[1 - active]}}},
             {"original_finger_forces_N",
              {{"active", original_force[active]}, {"support", original_force[1 - active]}}},
             {"self_contacts", self},
             {"cross_contacts", cross}});
      if (step % 67 == 0) {
        auto im = renderer.render(d, camera, &opt);
        cv::putText(im, "RX narrow tips / 55 mm cube / single-layer probe", {20, 32},
                    cv::FONT_HERSHEY_SIMPLEX, .6, {255, 255, 255}, 2);
        cv::putText(im, stage + " | t=" + std::to_string(t).substr(0, 5) + " s", {20, 62},
                    cv::FONT_HERSHEY_SIMPLEX, .6, {255, 255, 255}, 1);
        video.write(im);
      }
      if (drop > .06) {
        stage = "cube dropped";
        break;
      }
    }
  } catch (const std::exception& error) {
    failure = error.what();
  }
  int status = video.release();
  if (!video_started && status == 0) status = -1;
  for (const auto& warning : d->warning) warnings += warning.number;
  const double final_layer = d->qpos[m->jnt_qposadr[lj]] * 180 / pi;
  const double layer_error = std::abs(final_layer - goal);
  const bool completed = completed_steps == 24000 && std::isfinite(d->time) && d->time >= 12 - 1e-7;
  // 直接比较有符号层铰链角，不将 ±180 的误差折叠或借核心倾斜抵消。
  const bool passed = failure.empty() && status == 0 && completed && std::isfinite(layer_error) &&
                      layer_error <= .5 && maxangle * 180 / pi <= 1 && maxtranslation * 1000 <= 1 &&
                      maxself == 0 && maxcross == 0 && maxwrong == 0 && warnings == 0;
  r.update({{"sim_time_s", d->time},
            {"stage", stage},
            {"completed_12s", completed},
            {"completed_steps", completed_steps},
            {"final_passed", passed},
            {"max_self_contacts", maxself},
            {"max_cross_contacts", maxcross},
            {"max_upward_displacement_mm", maxup * 1000},
            {"max_translation_mm", maxtranslation * 1000},
            {"max_core_angle_deg", maxangle * 180 / pi},
            {"max_wrong_contacts", maxwrong},
            {"max_wrong_depth_mm", maxwrongdepth * 1000},
            {"max_wrong_force_N", maxwrongforce},
            {"first_wrong", firstwrong},
            {"peak_summed_normal_force_N", peakforce},
            {"max_downward_displacement_mm", maxdrop * 1000},
            {"warnings", warnings},
            {"final_layer_deg", final_layer},
            {"absolute_layer_error_deg", layer_error},
            {"final_active_wrist_deg", d->qpos[m->jnt_qposadr[ja]] * 180 / pi},
            {"final_support_wrist_deg", d->qpos[m->jnt_qposadr[jb]] * 180 / pi},
            {"encoder_exit", status},
            {"rows", std::move(rows)}});
  r["load_statistics"] = {{"start_time_s", 4},
                          {"samples", load_samples},
                          {"finger_order", {1, 2}},
                          {"bearing_threshold_N", .01}};
  for (int h = 0; h < 2; ++h) {
    const std::string role = h == active ? "active" : "support";
    J minimum = J::array();
    for (double force : min_pad[h]) minimum.push_back(load_samples ? J(force) : J(nullptr));
    r["load_statistics"][role] = {
        {"newpad_min_normal_force_N", minimum},
        {"original_finger_peak_normal_force_N", original_max[h]},
        {"original_finger_mean_normal_force_N",
         load_samples ? J(original_sum[h] / load_samples) : J(nullptr)},
        {"original_finger_bearing_samples", original_bearing_steps[h]},
        {"original_finger_bearing_fraction",
         load_samples ? J(double(original_bearing_steps[h]) / load_samples) : J(nullptr)},
        {"original_finger_contact_samples", original_contact_samples[h]}};
    r["peak_motor_torque_Nm"][role] = {{"wrist", peak_wrist[h]}, {"grip", peak_grip[h]}};
  }
  r["motor_torque_source"] = "电机关节自由度 qfrc_actuator，全程绝对峰值";
  if (!failure.empty()) r["error"] = failure;
}
}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path output_json;
  J report = {{"mode", "single_layer_probe"}, {"full_solve", false}, {"final_passed", false}};
  try {
    if (argc != 4 && argc != 5)
      throw std::invalid_argument("用法: dynamic OUTPUT_DIR A|B ANGLE_DEG [TAG]");
    const int active = parse_hand(argv[2]);
    const double goal = finite_number(argv[3]);
    if (std::abs(goal) != 90 && std::abs(goal) != 180)
      throw std::invalid_argument("ANGLE_DEG 仅支持 ±90 或 ±180");
    const std::string stem = argc == 5 ? argv[4]
                                       : "narrow55-" + hand_name(active) + (goal < 0 ? "m" : "p") +
                                             std::to_string(int(std::abs(goal)));
    if (!valid_tag(stem)) throw std::invalid_argument("TAG 仅允许非空字母、数字、- 和 _");
    const auto out = directory(argv[1], true);
    // 在登记报告路径前拒绝链接，避免异常处理写报告时覆盖链接目标。
    for (const auto& path : {out / (stem + ".json"), out / (stem + ".mp4")})
      if (std::filesystem::is_symlink(path)) throw std::invalid_argument("输出文件不能是符号链接");
    output_json = out / (stem + ".json");
    report.update({{"active_hand", hand_name(active)},
                   {"support_hand", hand_name(1 - active)},
                   {"goal_deg", goal},
                   {"shift_mm", default_shift * 1000},
                   {"target_active_rad", default_grip},
                   {"target_support_rad", default_grip},
                   {"forces_order", {"A", "B"}},
                   {"video", (out / (stem + ".mp4")).string()},
                   {"acceptance",
                    {{"absolute_layer_error_deg", .5},
                     {"max_core_angle_deg", 1},
                     {"max_translation_mm", 1},
                     {"required_duration_s", 12}}}});
    run(out, active, goal, stem, report);
    write_json(output_json, report);
    std::cout << report.dump(2) << '\n';
    if (!report.at("final_passed").get<bool>()) {
      std::cerr << "失败：单层探针未达到验收标准；报告已保存，视频状态见 encoder_exit。\n";
      if (report.contains("error")) std::cerr << report["error"] << '\n';
      return 1;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "失败：" << error.what() << '\n';
    report["final_passed"] = false;
    report["error"] = error.what();
    if (!output_json.empty()) {
      try {
        write_json(output_json, report);
      } catch (const std::exception& output_error) {
        std::cerr << "报告保存失败：" << output_error.what() << '\n';
      }
    }
    return 1;
  }
}
