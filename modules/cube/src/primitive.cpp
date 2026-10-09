#include "rm/primitive.hpp"
namespace rm::cube {
namespace {
constexpr std::array<const char*, 12> names{"A_P90",  "A_N90",   "A_P180", "A_N180",
                                            "A_OPEN", "A_CLOSE", "B_P90",  "B_N90",
                                            "B_P180", "B_N180",  "B_OPEN", "B_CLOSE"};
constexpr std::array<int, 4> turns{1, -1, 2, -2};
void valid(const RobotState& s) {
  if (s.orientation < 0 || s.orientation >= 24 || std::abs(s.wrist[0]) > 2 ||
      std::abs(s.wrist[1]) > 2 || (!s.closed[0] && !s.closed[1]) ||
      (std::abs(s.wrist[0]) % 2 && std::abs(s.wrist[1]) % 2))
    throw std::invalid_argument("Invalid robot state");
}
int mode_index(const std::string& mode) {
  if (mode == "whole" || mode == "jaw") return 0;
  if (mode == "face") return 1;
  if (mode == "empty") return 2;
  throw std::invalid_argument("Invalid primitive mode");
}
}  // namespace
std::string primitive_name(Primitive a) { return names.at(static_cast<size_t>(a)); }
Primitive parse_primitive(const std::string& name) {
  for (int i = 0; i < 12; ++i)
    if (names[i] == name) return Primitive(i);
  throw std::invalid_argument("Unknown primitive: " + name);
}
const std::vector<Mat>& robot_orientations() {
  static const std::vector<Mat> group = [] {
    std::vector<Mat> out{Mat::Identity()};
    for (size_t i = 0; i < out.size(); ++i)
      for (auto hand : {"A", "B"}) {
        Mat q = wrist_rotation(hand, pi / 2) * out[i];
        if (std::none_of(out.begin(), out.end(), [&](const Mat& m) { return m == q; }))
          out.push_back(q);
      }
    return out;
  }();
  return group;
}
int orientation_index(const Mat& q) {
  const auto& group = robot_orientations();
  for (int i = 0; i < 24; ++i)
    if (q.isApprox(group[i], 1e-8)) return i;
  throw std::invalid_argument("Robot orientation is not aligned");
}
int robot_state_id(const RobotState& s) {
  valid(s);
  int grip = s.closed[0] ? (s.closed[1] ? 0 : 1) : 2;
  return ((s.orientation * 5 + s.wrist[0] + 2) * 5 + s.wrist[1] + 2) * 3 + grip;
}
RobotState robot_state_from_id(int id) {
  if (id < 0 || id >= 1800) throw std::invalid_argument("Invalid robot state id");
  RobotState s;
  int grip = id % 3;
  id /= 3;
  s.closed = {grip != 2, grip != 1};
  s.wrist[1] = id % 5 - 2;
  id /= 5;
  s.wrist[0] = id % 5 - 2;
  s.orientation = id / 5;
  return s;
}
Json robot_state_json(const RobotState& s) {
  valid(s);
  return {{"orientation", matrix_json(robot_orientations()[s.orientation])},
          {"wrist_quarters", s.wrist},
          {"closed", s.closed}};
}
RobotState robot_snapshot(const Cube& c) {
  RobotState s;
  s.orientation = orientation_index(c.orientation);
  if (!c.robot_ready) return s;
  if (!c.active_face.empty()) throw std::invalid_argument("Finish current layer before planning");
  if (c.rx) {
    int core = c.id(mjOBJ_BODY, "core");
    const Mat delta = c.orientation.transpose() * c.body_rotation(core);
    double angle = std::acos(std::clamp((delta.trace() - 1) / 2, -1., 1.));
    if ((c.body_position(core) - Vec(0, 0, .22)).norm() > .001 || angle > .02)
      throw std::invalid_argument("RX cube slipped from robot alignment; angle=" +
                                  std::to_string(angle));
  }
  for (int h = 0; h < 2; ++h) {
    std::string hand = h == 0 ? "A" : "B";
    double target = c.data->ctrl[c.id(mjOBJ_ACTUATOR, hand + "_yaw_drive")] / (pi / 2);
    s.wrist[h] = std::lround(target);
    if (std::abs(target - s.wrist[h]) > 1e-7)
      throw std::invalid_argument("Wrist not at quarter turn");
    int joint = c.id(mjOBJ_JOINT, hand + "_yaw");
    // 支撑腕存在静态负载偏差；离散化采用与块体对齐一致的0.02弧度容差。
    // 主动轴在 move_actuator 中仍须通过0.006弧度跟踪检查。
    if (std::abs(c.data->qpos[c.model->jnt_qposadr[joint]] - target * pi / 2) > .02)
      throw std::invalid_argument(hand + ": wrist not settled; actual=" +
                                  std::to_string(c.data->qpos[c.model->jnt_qposadr[joint]]) +
                                  ", target=" + std::to_string(target * pi / 2));
    s.closed[h] = !c.grasped.at(hand).empty();
    if (s.closed[h] && c.pad_contacts(hand).size() != 2)
      throw std::invalid_argument("Closed gripper lost fingertip contact");
    if (!s.closed[h] && c.open_gap(hand) < c.required_open_gap)
      throw std::invalid_argument("Open gripper has insufficient clearance");
    if (s.closed[h] && c.grasped.at(hand) != "core")
      throw std::invalid_argument("Layer still grasped");
  }
  valid(s);
  return s;
}
std::optional<PrimitiveTransition> primitive_transition(const RobotState& s, Primitive a) {
  valid(s);
  primitive_name(a);
  int h = int(a) / 6, k = int(a) % 6, o = 1 - h;
  PrimitiveTransition t{s, "jaw", ""};
  if (k == 4) {
    if (!s.closed[h] || !s.closed[o]) return {};
    t.state.closed[h] = false;
  } else if (k == 5) {
    if (s.closed[h] || (std::abs(s.wrist[h]) % 2 && std::abs(s.wrist[o]) % 2)) return {};
    t.state.closed[h] = true;
  } else {
    // 横置夹爪会侵入另一腕的旋转扫掠区，打开后仍可能相撞。
    if (std::abs(s.wrist[o]) % 2) return {};
    int delta = turns[k];
    t.state.wrist[h] += delta;
    if (std::abs(t.state.wrist[h]) > 2) return {};
    if (!s.closed[h])
      t.mode = "empty";
    else if (!s.closed[o]) {
      t.mode = "whole";
      t.state.orientation = orientation_index(wrist_rotation(h == 0 ? "A" : "B", delta * pi / 2) *
                                              robot_orientations()[s.orientation]);
    } else {
      t.mode = "face";
      Vec n =
          robot_orientations()[s.orientation].transpose() * (h == 0 ? Vec(1, 0, 0) : Vec(0, -1, 0));
      for (char f : faces) {
        auto [axis, sign] = face_axis(f);
        if (n == Vec::Unit(axis) * sign)
          t.move = std::string(1, f) + (std::abs(delta) == 2 ? "2" : delta > 0 ? "'" : "");
      }
    }
  }
  return t;
}
PrimitiveCosts PrimitiveCosts::defaults(double ws, double js, bool fast) {
  if (!std::isfinite(ws) || ws <= 0 || !std::isfinite(js) || js <= 0)
    throw std::invalid_argument("Invalid speed");
  PrimitiveCosts c;
  for (int i = 0; i < 12; ++i) {
    int k = i % 6;
    double d = k < 4 ? (k < 2 ? .85 : 1.2) / ws + .15 : fast ? .3 / js + .006 : .3 / js + .15;
    if (k == 5) d += fast ? .008 : .08;
    c.duration[i].fill(d);
    if (k < 4) c.duration[i][1] += .15 + (fast ? .008 : .08);
  }
  return c;
}
void PrimitiveCosts::apply_json(const Json& profile) {
  if (!profile.is_object()) throw std::invalid_argument("Cost profile must be object");
  for (auto& [key, value] : profile.items()) {
    if (key != "duration_s" && key != "mode_duration_s")
      throw std::invalid_argument("Unknown cost field: " + key);
    if (!value.is_object()) throw std::invalid_argument("Cost entries must be objects");
    for (auto& [name, v] : value.items()) {
      int a = int(parse_primitive(name));
      auto set = [&](const Json& j, int m) {
        if (!j.is_number()) throw std::invalid_argument("Duration must be numeric");
        double d = j.get<double>();
        if (!std::isfinite(d) || d < 0 || d > 3600)
          throw std::invalid_argument("Duration outside [0,3600] seconds");
        duration[a][m] = d;
      };
      if (key == "duration_s")
        for (int m = 0; m < 3; ++m) set(v, m);
      else {
        if (!v.is_object()) throw std::invalid_argument("Mode durations must be object");
        for (auto& [mode, d] : v.items()) {
          if ((a % 6 >= 4) != (mode == "jaw"))
            throw std::invalid_argument("Mode incompatible with primitive");
          set(d, mode_index(mode));
        }
      }
    }
  }
}
double PrimitiveCosts::seconds(Primitive a, const std::string& mode) const {
  return duration.at(int(a)).at(mode_index(mode));
}
Json PrimitiveCosts::json() const {
  Json out = Json::object();
  for (int i = 0; i < 12; ++i)
    if (i % 6 >= 4)
      out[names[i]] = {{"jaw", duration[i][0]}};
    else
      out[names[i]] = {
          {"whole", duration[i][0]}, {"face", duration[i][1]}, {"empty", duration[i][2]}};
  return {{"mode_duration_s", out}};
}
Json primitive_plan_json(const std::vector<Primitive>& plan, RobotState s,
                         const PrimitiveCosts& costs) {
  Json out = Json::array();
  double time = 0;
  for (auto a : plan) {
    auto t = primitive_transition(s, a);
    if (!t) throw std::invalid_argument("Illegal primitive in plan");
    double d = costs.seconds(a, t->mode);
    out.push_back({{"action", primitive_name(a)},
                   {"mode", t->mode},
                   {"move", t->move},
                   {"duration_s", d},
                   {"start_s", time},
                   {"end_s", time + d},
                   {"before", robot_state_json(s)},
                   {"after", robot_state_json(t->state)}});
    time += d;
    s = t->state;
  }
  return out;
}
std::vector<std::string> primitive_moves(const std::vector<Primitive>& plan, RobotState s) {
  std::vector<std::string> moves;
  for (auto a : plan) {
    auto t = primitive_transition(s, a);
    if (!t) throw std::invalid_argument("Illegal primitive in plan");
    if (!t->move.empty()) moves.push_back(t->move);
    s = t->state;
  }
  return moves;
}
void execute_primitives(Cube& c, const std::vector<Primitive>& plan, const RobotState& expected,
                        const Cube::Callback& cb) {
  if (!c.robot_ready) throw std::invalid_argument("Initialize grasps before primitive execution");
  if (robot_snapshot(c) != expected) throw std::invalid_argument("Robot changed since planning");
  primitive_moves(plan, expected);  // 执行前校验完整序列。
  RobotState s = expected;
  for (size_t i = 0; i < plan.size(); ++i) {
    auto a = plan[i];
    auto t = *primitive_transition(s, a);
    int h = int(a) / 6, k = int(a) % 6;
    std::string hand = h == 0 ? "A" : "B";
    c.current_action = {{"primitive", primitive_name(a)},
                        {"mode", t.mode},
                        {"move", t.move},
                        {"index", i},
                        {"total", plan.size()}};
    double began = c.data->time;
    if (k == 4) {
      c.release(hand);
      c.move_actuator(hand + "_fingers_actuator", 0, .3, cb);
      c.wait_open(hand, cb);
    } else if (k == 5) {
      c.move_actuator(hand + "_fingers_actuator", 115, .3, cb);
      c.grasp(hand, "core", cb);
    } else {
      if (t.mode == "face") {
        c.release(hand);
        c.unlock(t.move, hand);
        c.grasp(hand, "face", cb);
      }
      Action monitor{"yaw", hand, t.state.wrist[h] * pi / 2, t.mode, t.move};
      Cube::Callback observe = [&](Cube&) { c.check_clearance(monitor, cb); };
      c.move_actuator(hand + "_yaw_drive", monitor.target, k < 2 ? .85 : 1.2, observe,
                      hand + "_yaw");
      if (t.mode == "whole") c.orientation = robot_orientations()[t.state.orientation];
      if (t.mode == "face") {
        c.lock(t.move, cb, turns[k]);
        c.grasped[hand] = "core";
      }
    }
    auto [pos, angle] = c.pose_error();
    if (pos > .0002 || angle > .02) throw std::runtime_error("Primitive cube pose mismatch");
    for (int j = 0; j < mjNWARNING; ++j)
      if (c.data->warning[j].number) throw std::runtime_error("Primitive numerical warning");
    s = t.state;
    if (robot_snapshot(c) != s) throw std::runtime_error("Primitive end state mismatch");
    c.current_action["actual_duration_s"] = c.data->time - began;
    c.executed.push_back(c.current_action);
  }
}
}  // namespace rm::cube
