#include "rm/cube.hpp"
namespace rm::cube {
double Cube::open_gap(const std::string& hand) const {
  int right = id(mjOBJ_GEOM, hand + "_right_tip"), left = id(mjOBJ_GEOM, hand + "_left_tip");
  Vec delta = Eigen::Map<const Vec>(data->geom_xpos + 3 * right) -
              Eigen::Map<const Vec>(data->geom_xpos + 3 * left),
      dir = delta.normalized();
  double radii = 0;
  for (int g : {right, left}) {
    Mat r = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(data->geom_xmat + 9 * g);
    radii += (r.transpose() * dir).cwiseAbs().dot(Eigen::Map<const Vec>(model->geom_size + 3 * g));
  }
  return delta.norm() - radii;
}
Json Cube::pad_contacts(const std::string& h) const {
  Json result = Json::object();
  for (auto side : {"right", "left"}) {
    int tip = id(mjOBJ_GEOM, h + "_" + side + "_tip");
    for (int i = 0; i < data->ncon; i++) {
      auto& c = data->contact[i];
      int other = c.geom[0] == tip ? c.geom[1] : c.geom[1] == tip ? c.geom[0] : -1;
      if (other < 0 || !cube_bodies.contains(model->geom_bodyid[other])) continue;
      mjtNum force[6] = {};
      mj_contactForce(model, data, i, force);
      if (force[0] > .01) {
        if (!result.contains(side)) result[side] = Json::array();
        result[side].push_back({{"normal_force_n", force[0]}, {"body", model->geom_bodyid[other]}});
      }
    }
  }
  return result;
}
void Cube::wait_open(const std::string& h, const Callback& cb) {
  for (int i = 0; i < 100; i++) {
    if (open_gap(h) >= required_open_gap) return;
    advance(.01, cb);
  }
  throw std::runtime_error(h + ": insufficient open aperture for in-place rotation");
}
void Cube::check_clearance(const Action& a, const Callback& cb) {
  std::string open = a.mode == "empty"   ? a.hand
                     : a.mode == "whole" ? (a.hand == "A" ? "B" : "A")
                                         : "";
  if (!open.empty()) {
    double gap = open_gap(open);
    if (clearance["min_open_gap_m"].is_null() || gap < clearance["min_open_gap_m"].get<double>())
      clearance["min_open_gap_m"] = gap;
    if (gap < required_open_gap)
      throw std::runtime_error(open + ": aperture below rotation clearance");
  }
  for (int i = 0; i < data->ncon; i++) {
    auto& c = data->contact[i];
    int g1 = c.geom[0], g2 = c.geom[1];
    if (g1 < 0 || g2 < 0) continue;
    int h1 = geom_hand[g1], h2 = geom_hand[g2], which = open == "A" ? 0 : 1;
    bool bad =
        (h1 >= 0 && h2 >= 0 && h1 != h2) ||
        (!open.empty() && ((h1 == which && geom_cube[g2]) || (h2 == which && geom_cube[g1])));
    if (rx && a.mode == "face" && !active_face.empty()) {
      const auto [axis, sign] = face_axis(active_face[0]);
      auto moving = [&](int geom) {
        int body = model->geom_bodyid[geom];
        if (body == id(mjOBJ_BODY, "center_" + active_face)) return true;
        auto it = std::find(piece_ids.begin(), piece_ids.end(), body);
        return it != piece_ids.end() && slots[it - piece_ids.begin()][axis] == sign;
      };
      int hand = h1 >= 0 && geom_cube[g2] ? h1 : h2 >= 0 && geom_cube[g1] ? h2 : -1;
      int cube_geom = h1 >= 0 && geom_cube[g2] ? g2 : g1;
      if (hand >= 0 && ((hand == (a.hand == "A" ? 0 : 1)) != moving(cube_geom))) {
        mjtNum force[6] = {};
        mj_contactForce(model, data, i, force);
        bad = bad || force[0] > .01;
      }
    }
    // RX 的 margin 可在几何尚未穿透时传力；禁触也必须覆盖这种承载。
    mjtNum forbidden_force[6] = {};
    if (rx && bad) mj_contactForce(model, data, i, forbidden_force);
    if (bad && (c.dist < 0 || (rx && forbidden_force[0] > .01))) {
      clearance["forbidden_contacts"] = clearance["forbidden_contacts"].get<int>() + 1;
      throw std::runtime_error("In-place rotation collision between geoms " + std::to_string(g1) +
                               " and " + std::to_string(g2));
    }
  }
  clearance["rotation_steps_checked"] = clearance["rotation_steps_checked"].get<int>() + 1;
  if (cb) cb(*this);
}
void Cube::move_actuator(const std::string& name, double target, double duration,
                         const Callback& cb, const std::string& joint) {
  if (!std::isfinite(target) || !std::isfinite(duration) || duration <= 0)
    throw std::invalid_argument("Invalid actuator target or duration");
  if (rx) {
    if (joint.empty()) {
      if (target != 0 && target != 115) throw std::invalid_argument("RX jaw expects open/close");
      target = target == 0 ? -2.25 : -1.20;
    }
    duration = joint.empty() ? 2. : 3.;
  }
  if (fast && joint.empty()) {
    fast_jaw(name, target, duration, cb);
    return;
  }
  int a = id(mjOBJ_ACTUATOR, name);
  double start = data->ctrl[a], gain = model->actuator_gainprm[10 * a],
         damping = -model->actuator_biasprm[10 * a + 2];
  duration /= joint.empty() ? jaw_speed : wrist_speed;
  int steps = std::max(1L, std::lround(duration / model->opt.timestep));
  duration = steps * model->opt.timestep;
  double began = data->time, peakv = 0, peakf = 0;
  Json gap0 = joint.empty() ? Json(open_gap(name.substr(0, 1))) : Json(nullptr);
  int j = joint.empty() ? -1 : id(mjOBJ_JOINT, joint);
  for (int k = 1; k <= steps; k++) {
    double u = double(k) / steps, smooth = u * u * u * (10 + u * (-15 + 6 * u)),
           v = 30 * u * u * (1 - u) * (1 - u) / duration,
           ff = j >= 0 && gain ? damping / gain * (target - start) * v : 0;
    data->ctrl[a] = start + (target - start) * smooth + ff;
    step(cb);
    peakf = std::max(peakf, std::abs(data->actuator_force[a]));
    if (j >= 0) peakv = std::max(peakv, std::abs(data->qvel[model->jnt_dofadr[j]]));
  }
  double actual = j >= 0 ? data->qpos[model->jnt_qposadr[j]] : open_gap(name.substr(0, 1));
  motion_checks.push_back({{"actuator", name},
                           {"start_time_s", began},
                           {"duration_s", data->time - began},
                           {"target", target},
                           {"actual_at_ramp_end", actual},
                           {"peak_joint_velocity_rad_s", j >= 0 ? Json(peakv) : Json(nullptr)},
                           {"peak_actuator_force", peakf},
                           {"initial_aperture_m", gap0}});
  data->ctrl[a] = target;
  advance(rx ? .5 : .15, cb);
  if (j >= 0 &&
      std::abs(data->qpos[model->jnt_qposadr[j]] - target) > (joint.ends_with("yaw") ? .006 : .001))
    throw std::runtime_error(joint + " failed tracking");
}
void Cube::fast_jaw(const std::string& name, double target, double duration, const Callback& cb) {
  int a = id(mjOBJ_ACTUATOR, name);
  std::string h = name.substr(0, 1);
  double ratio = .8 / 255, kp = 100 * jaw_speed * jaw_speed, kv = 10 * jaw_speed;
  model->actuator_gainprm[10 * a] = kp * ratio;
  model->actuator_biasprm[10 * a + 1] = -kp;
  model->actuator_biasprm[10 * a + 2] = -kv;
  model->actuator_forcerange[2 * a] = -40;
  model->actuator_forcerange[2 * a + 1] = 40;
  double initial = data->actuator_length[a], goal = target == 0 ? 0 : .332,
         dt = model->opt.timestep;
  duration = std::max(dt, std::round(duration / jaw_speed / dt) * dt);
  double began = data->time, gap0 = open_gap(h), previous = gap0, peakv = 0, peakf = 0, gap = gap0;
  Json first = nullptr, ramp = nullptr;
  bool contact = false, finished = false;
  int stable = 0;
  for (int k = 0; k < std::lround((duration + .1) / dt); k++) {
    double t = (k + 1) * dt, u = std::min(1., t / duration),
           p = u * u * u * (10 + u * (-15 + 6 * u)),
           v = u < 1 ? 30 * u * u * (1 - u) * (1 - u) / duration : 0,
           reference = initial + (goal - initial) * p + kv / kp * (goal - initial) * v;
    if (target && (contact || u == 1)) {
      model->actuator_forcerange[2 * a] = -5;
      model->actuator_forcerange[2 * a + 1] = 5;
      reference = .345;
    }
    data->ctrl[a] = reference / ratio;
    step(cb);
    gap = open_gap(h);
    if (ramp.is_null() && t >= duration) ramp = gap;
    peakv = std::max(peakv, std::abs(gap - previous) / dt);
    peakf = std::max(peakf, std::abs(data->actuator_force[a]));
    previous = gap;
    Json pads = target ? pad_contacts(h) : Json::object();
    contact = contact || !pads.empty();
    bool ready = target ? pads.size() == 2 : gap >= required_open_gap;
    if (ready && first.is_null()) first = data->time - began;
    stable = ready ? stable + 1 : 0;
    if (t >= duration && stable * dt >= .004) {
      finished = true;
      break;
    }
  }
  if (!finished) throw std::runtime_error(h + ": fast jaw failed contact/clearance feedback");
  motion_checks.push_back({{"actuator", name},
                           {"start_time_s", began},
                           {"duration_s", data->time - began},
                           {"reference_duration_s", duration},
                           {"target", target},
                           {"actual_at_ramp_end", ramp},
                           {"final_aperture_m", gap},
                           {"stable_reached_s", data->time - began - .004 + dt},
                           {"initial_aperture_m", gap0},
                           {"first_reached_s", first},
                           {"peak_aperture_velocity_m_s", peakv},
                           {"peak_actuator_force", peakf},
                           {"peak_joint_velocity_rad_s", nullptr}});
}
void Cube::grasp(const std::string& h, const std::string& mode, const Callback& cb) {
  if (h != "A" && h != "B") throw std::invalid_argument("Unknown hand");
  if (mode != "core" && mode != "face") throw std::invalid_argument("Unknown grasp mode");
  int stable = 0;
  for (int i = 0; i < 100; i++) {
    stable = pad_contacts(h).size() == 2 ? stable + 1 : 0;
    if (stable >= 4) break;
    advance(fast ? .002 : .01, cb);
  }
  auto contacts = pad_contacts(h);
  if (stable < 4) throw std::runtime_error(h + ": both fingertips must contact before grasp");
  grasped[h] = mode;
  grasp_checks.push_back(
      {{"hand", h}, {"mode", mode}, {"contacts", contacts}, {"time_s", data->time}});
  advance(fast ? .002 : .05, cb);
}
void Cube::release(const std::string& h) {
  if (h != "A" && h != "B") throw std::invalid_argument("Unknown hand");
  std::string other = h == "A" ? "B" : "A";
  bool fixture = data->eq_active[id(mjOBJ_EQUALITY, "loading_fixture")];
  if (grasped[other] != "core" && !fixture)
    throw std::runtime_error("Refusing release without independent support");
  grasped[h] = "";
}
void Cube::initialize_grasps(const Callback& cb) {
  if (!dual) throw std::runtime_error("Grippers require dual scene");
  if (robot_ready) throw std::runtime_error("Already initialized");
  for (char face : faces) {
    std::string f(1, face);
    int eq = id(mjOBJ_EQUALITY, "lock_" + f), a = id(mjOBJ_ACTUATOR, "drive_" + f);
    model->eq_data[mjNEQDATA * eq] = targets[face];
    data->eq_active[eq] = 1;
    std::fill(model->actuator_gainprm + 10 * a, model->actuator_gainprm + 10 * a + 10, 0);
    std::fill(model->actuator_biasprm + 10 * a, model->actuator_biasprm + 10 * a + 10, 0);
    data->ctrl[a] = 0;
  }
  for (auto h : {"A", "B"}) {
    current_action = Action{"jaw", h, 115}.json();
    move_actuator(std::string(h) + "_fingers_actuator", 115, .35, cb);
    grasp(h, "core", cb);
  }
  data->eq_active[id(mjOBJ_EQUALITY, "loading_fixture")] = 0;
  robot_ready = true;
  advance(.15, cb);
}
void Cube::unlock(const std::string& move, const std::string& hand) {
  if (hand != "A" && hand != "B") throw std::invalid_argument("Unknown hand");
  if (!active_face.empty()) throw std::runtime_error("Layer already unlocked");
  auto [f, c] = parse_move(move);
  auto [axis, sign] = face_axis(f);
  Vec normal = body_rotation(id(mjOBJ_BODY, "core")) * Vec::Unit(axis) * sign;
  if ((normal - (hand == "A" ? Vec(1, 0, 0) : Vec(0, -1, 0))).norm() > .025)
    throw std::runtime_error("Face not aligned with selected wrist");
  active_face = std::string(1, f);
  active_hand = hand;
  for (int i = 0; i < 20; i++)
    if (slots[i][axis] == sign) attach(i, "center_" + active_face);
  data->eq_active[id(mjOBJ_EQUALITY, "lock_" + active_face)] = 0;
}
void Cube::lock(const std::string& move, const Callback& cb, int wrist_quarters) {
  auto [f, c] = parse_move(move);
  if (active_face != std::string(1, f)) throw std::runtime_error("Wrong layer lock");
  auto [axis, sign] = face_axis(f);
  if (wrist_quarters != 0 && (std::abs(wrist_quarters) > 2 || (wrist_quarters + c) % 4 != 0))
    throw std::invalid_argument("Wrist turn disagrees with face move");
  double expected = targets[f] + (wrist_quarters ? wrist_quarters : -c) * pi / 2;
  int j = id(mjOBJ_JOINT, "hinge_" + std::string(1, f));
  if (std::abs(data->qpos[model->jnt_qposadr[j]] - expected) > .02)
    throw std::runtime_error(move + ": physical layer did not turn");
  std::vector<int> selected;
  for (int i = 0; i < 20; i++)
    if (slots[i][axis] == sign) selected.push_back(i);
  Mat r = rotation(axis, -sign * c * pi / 2).array().round();
  if (rx) {
    const int core = id(mjOBJ_BODY, "core");
    const Mat core_rotation = body_rotation(core);
    double position_error = 0, angle_error = 0;
    for (int i : selected) {
      const Vec physical =
          core_rotation.transpose() * (body_position(piece_ids[i]) - body_position(core));
      const Mat physical_rotation = core_rotation.transpose() * body_rotation(piece_ids[i]);
      const Mat desired_rotation = r * orientations[i];
      position_error = std::max(position_error, (physical - cube_pitch * r * slots[i]).norm());
      angle_error = std::max(
          angle_error,
          std::acos(std::clamp(((desired_rotation.transpose() * physical_rotation).trace() - 1) / 2,
                               -1., 1.)));
    }
    motion_checks.push_back({{"check", "before_layer_lock"},
                             {"move", move},
                             {"position_error_m", position_error},
                             {"orientation_error_rad", angle_error}});
    if (position_error > .02 * cube_pitch || angle_error > .02)
      throw std::runtime_error(move + ": physical pieces not aligned before layer lock");
  }
  for (int i : selected) {
    slots[i] = r * slots[i];
    orientations[i] = r * orientations[i];
  }
  targets[f] = expected;
  int eq = id(mjOBJ_EQUALITY, "lock_" + std::string(1, f));
  model->eq_data[mjNEQDATA * eq] = expected;
  data->eq_active[eq] = 1;
  for (int i : selected) attach(i, "core", true);
  advance(.15, cb);
  active_face.clear();
  active_hand.clear();
  history.push_back(move);
}
void Cube::execute(const Plan& plan, const Callback& cb) {
  if (!robot_ready) throw std::runtime_error("Initialize grasps before execution");
  replay_plan(plan, orientation);
  for (size_t i = 0; i < plan.size(); i++) {
    auto& a = plan[i];
    current_action = a.json();
    current_action["index"] = i;
    current_action["total"] = plan.size();
    auto h = a.hand;
    if (a.kind == "release")
      release(h);
    else if (a.kind == "jaw") {
      move_actuator(h + "_fingers_actuator", a.target, .3, cb);
      if (a.target == 0) wait_open(h, cb);
    } else if (a.kind == "grasp")
      grasp(h, a.mode, cb);
    else if (a.kind == "yaw") {
      double old = data->ctrl[id(mjOBJ_ACTUATOR, h + "_yaw_drive")];
      Callback observe = [&](Cube&) { check_clearance(a, cb); };
      move_actuator(h + "_yaw_drive", a.target, std::abs(a.target - old) > 2 ? 1.2 : .85, observe,
                    h + "_yaw");
      if (a.mode == "whole") orientation = wrist_rotation(h, a.target - old) * orientation;
    } else if (a.kind == "layer_unlock")
      unlock(a.move, a.hand);
    else if (a.kind == "layer_lock")
      lock(a.move, cb);
    else if (a.kind == "checkpoint") {
      auto [p, angle] = pose_error();
      if (p > .0002 || angle > .02) throw std::runtime_error("Cube pose mismatch at checkpoint");
      for (int j = 0; j < mjNWARNING; j++)
        if (data->warning[j].number) throw std::runtime_error("MuJoCo numerical warning");
    }
    executed.push_back(a.json());
  }
}
}  // namespace rm::cube
