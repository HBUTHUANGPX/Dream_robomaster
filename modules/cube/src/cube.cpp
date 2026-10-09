#include "rm/cube.hpp"

#include <unistd.h>

#include <fstream>
#include <sstream>

#include "rm/robot_search.hpp"
#include "rm/rx_gripper.hpp"
extern "C" {
#include "cubiecube.h"
#include "facecube.h"
#include "search.h"
}
namespace rm::cube {
static std::string name(int i) {
  return "piece_" + (i < 10 ? std::string("0") : std::string()) + std::to_string(i);
}
Cube::Cube(const std::filesystem::path& root, bool isdual, double s, std::optional<double> ws,
           std::optional<double> js, const std::filesystem::path& rx_bundle)
    : dual(isdual),
      fast(rx_bundle.empty() && js.has_value() && *js > 1),
      rx(!rx_bundle.empty()),
      cube_pitch(rx ? .055 / 3 : pitch),
      speed(s),
      wrist_speed(ws.value_or(s)),
      jaw_speed(js.value_or(s)) {
  for (double v : {speed, wrist_speed, jaw_speed})
    if (!std::isfinite(v) || v <= 0 || v > 1000)
      throw std::invalid_argument("Speeds must be finite, positive, at most 1000");
  if (rx && !dual) throw std::invalid_argument("RX grippers require --dual");
  if (rx) required_open_gap = std::sqrt(2.) * (3 * cube_pitch + .0002) + .004;
  xml = build_scene(root, dual, fast, rx_bundle);
  auto cache = root / ".cache/native-cube";
  std::filesystem::create_directories(cache);
  std::string pattern = (cache / "scene-XXXXXX.xml").string();
  std::vector<char> temp(pattern.begin(), pattern.end());
  temp.push_back(0);
  int fd = mkstemps(temp.data(), 4);
  if (fd < 0) throw std::runtime_error("Cannot create cube XML");
  close(fd);
  auto path = std::filesystem::path(temp.data());
  try {
    std::ofstream(path) << xml;
    sim = std::make_unique<Simulation>(path);
    std::filesystem::remove(path);
  } catch (...) {
    std::filesystem::remove(path);
    throw;
  }
  model = sim->model;
  data = sim->data;
  if (rx) initialize_rx_grippers(model, data);
  for (int x = -1; x <= 1; x++)
    for (int y = -1; y <= 1; y++)
      for (int z = -1; z <= 1; z++)
        if ((x != 0) + (y != 0) + (z != 0) >= 2) {
          piece_ids.push_back(id(mjOBJ_BODY, name(initial_slots.size())));
          initial_slots.emplace_back(x, y, z);
          orientations.push_back(Mat::Identity());
        }
  slots = initial_slots;
  for (char f : faces) targets[f] = 0;
  mj_forward(model, data);
  clearance = {{"rotation_steps_checked", 0},
               {"forbidden_contacts", 0},
               {"min_open_gap_m", nullptr},
               {"required_open_gap_m", required_open_gap}};
  if (dual) {
    cube_bodies.insert(piece_ids.begin(), piece_ids.end());
    for (char f : faces) cube_bodies.insert(id(mjOBJ_BODY, "center_" + std::string(1, f)));
    for (int i = 0; i < model->ngeom; i++) {
      const char* n = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[i]);
      std::string bn = n ? n : "";
      geom_hand.push_back(bn.starts_with("A_") ? 0 : bn.starts_with("B_") ? 1 : -1);
      geom_cube.push_back(cube_bodies.contains(model->geom_bodyid[i]));
    }
    advance(.3);
  }
}
int Cube::id(mjtObj type, const std::string& n) const {
  int value = mj_name2id(model, type, n.c_str());
  if (value < 0) throw std::invalid_argument("Missing model object: " + n);
  return value;
}
Mat Cube::body_rotation(int b) const {
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(data->xmat + 9 * b);
}
Vec Cube::body_position(int b) const { return Eigen::Map<const Vec>(data->xpos + 3 * b); }
void Cube::attach(int i, const std::string& parent, bool ideal) {
  int eq = id(mjOBJ_EQUALITY, name(i) + "_to_" + parent), p = id(mjOBJ_BODY, parent),
      b = piece_ids.at(i), core = id(mjOBJ_BODY, "core");
  Mat r = body_rotation(p),
      ori = ideal ? (body_rotation(core) * orientations[i]).eval() : body_rotation(b);
  Vec pos = ideal ? (body_position(core) + body_rotation(core) * (cube_pitch * slots[i])).eval()
                  : body_position(b);
  Vec rel = r.transpose() * (pos - body_position(p));
  Mat rr = r.transpose() * ori;
  Eigen::Quaterniond q(rr);
  double* e = model->eq_data + eq * mjNEQDATA;
  std::fill(e, e + 3, 0);
  Eigen::Map<Vec>(e + 3) = rel;
  e[6] = q.w();
  e[7] = q.x();
  e[8] = q.y();
  e[9] = q.z();
  int start = id(mjOBJ_EQUALITY, name(i) + "_to_core");
  std::fill(data->eq_active + start, data->eq_active + start + 7, 0);
  data->eq_active[eq] = 1;
}
void Cube::step(const Callback& cb) {
  mj_step(model, data);
  mj_forward(model, data);
  if (rx) {
    for (int i = 0; i < model->nq; ++i)
      if (!std::isfinite(data->qpos[i])) throw std::runtime_error("RX non-finite position");
    for (auto hand : {"A", "B"}) {
      int a = id(mjOBJ_ACTUATOR, std::string(hand) + "_fingers_actuator");
      rx_peak_motor_torque = std::max(rx_peak_motor_torque, std::abs(data->actuator_force[a]));
    }
    for (int i = 0; robot_ready && i < 6; ++i)
      rx_peak_cube_motor_force =
          std::max(rx_peak_cube_motor_force, std::abs(data->actuator_force[i]));
    for (int i = 0; i < data->ncon; ++i) {
      const auto& contact = data->contact[i];
      if (contact.geom[0] < 0 || contact.geom[1] < 0) continue;
      int g1 = contact.geom[0], g2 = contact.geom[1];
      int h1 = geom_hand[g1], h2 = geom_hand[g2];
      if ((h1 >= 0 && geom_cube[g2]) || (h2 >= 0 && geom_cube[g1]))
        rx_max_contact_penetration = std::max(rx_max_contact_penetration, -contact.dist);
      auto extension = [&](int geom) {
        const char* name = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[geom]);
        return name && std::string_view(name).ends_with("_extension");
      };
      if (h1 >= 0 && h2 >= 0 && (h1 != h2 || extension(g1) || extension(g2))) {
        mjtNum force[6] = {};
        mj_contactForce(model, data, i, force);
        if (contact.dist < -.00001 || force[0] > .01) {
          clearance["forbidden_contacts"] = clearance["forbidden_contacts"].get<int>() + 1;
          throw std::runtime_error("RX fingertip self/inter-hand collision");
        }
      }
    }
    ++rx_contact_steps;
    if (robot_ready && !active_face.empty()) check_clearance(Action{"yaw", active_hand, 0, "face"});
  }
  if (cb) cb(*this);
}
void Cube::advance(double sec, const Callback& cb) {
  if (!std::isfinite(sec) || sec < 0) throw std::invalid_argument("Invalid duration");
  for (int i = 0, n = std::lround(sec / model->opt.timestep); i < n; i++) step(cb);
}
void Cube::turn(const std::string& move, const Callback& cb) {
  auto [f, c] = parse_move(move);
  if (robot_ready) throw std::runtime_error("Use gripper plan after initialization");
  if (!active_face.empty()) throw std::runtime_error("Face already active");
  auto [axis, sign] = face_axis(f);
  std::vector<int> selected;
  for (int i = 0; i < 20; i++)
    if (slots[i][axis] == sign) selected.push_back(i);
  active_face = std::string(1, f);
  for (int i : selected) attach(i, "center_" + active_face);
  int act = id(mjOBJ_ACTUATOR, "drive_" + active_face);
  double start = targets[f], end = start - c * pi / 2;
  int steps = std::lround((std::abs(c) == 1 ? .75 : 1.05) / model->opt.timestep);
  for (int k = 1; k <= steps; k++) {
    progress = double(k) / steps;
    double u = progress, smooth = u * u * u * (10 + u * (-15 + 6 * u));
    data->ctrl[act] = start + (end - start) * smooth;
    step(cb);
  }
  targets[f] = end;
  data->ctrl[act] = end;
  advance(.15, cb);
  int hinge = id(mjOBJ_JOINT, "hinge_" + active_face);
  if (std::abs(data->qpos[model->jnt_qposadr[hinge]] - end) > .002)
    throw std::runtime_error("Turn did not reach alignment; refusing layer reassignment");
  Mat r = rotation(axis, -sign * c * pi / 2).array().round().matrix();
  for (int i : selected) {
    slots[i] = r * slots[i];
    orientations[i] = r * orientations[i];
    attach(i, "core", true);
  }
  advance(.1, cb);
  active_face.clear();
  progress = 0;
  history.push_back(move);
}
std::pair<double, double> Cube::pose_error(bool goal) const {
  int core = id(mjOBJ_BODY, "core");
  Mat r = body_rotation(core);
  double pmax = 0, amax = 0;
  for (int i = 0; i < 20; i++) {
    Vec p = r.transpose() * (body_position(piece_ids[i]) - body_position(core));
    Mat m = r.transpose() * body_rotation(piece_ids[i]),
        want = goal ? Mat::Identity() : orientations[i];
    pmax = std::max(pmax, (p - cube_pitch * (goal ? initial_slots[i] : slots[i])).norm());
    double tr = (m.array() * want.array()).sum();
    amax = std::max(amax, std::acos(std::clamp((tr - 1) / 2, -1., 1.)));
  }
  return {pmax, amax};
}
bool Cube::is_solved() const {
  for (int i = 0; i < 20; i++)
    if (slots[i] != initial_slots[i] || orientations[i] != Mat::Identity()) return false;
  return true;
}
void validate_facelets(const std::string& s) {
  if (s.size() != 54) throw std::invalid_argument("Expected 54 URFDLB facelets");
  for (int i = 0; i < 6; i++)
    if (std::count(s.begin(), s.end(), order[i]) != 9 || s[9 * i + 4] != order[i])
      throw std::invalid_argument("Need nine colors each and URFDLB centers");
  std::string input = s;
  auto fc = get_facecube_fromstring(input.data());
  auto cc = toCubieCube(fc);
  int result = verify(cc);
  free(fc);
  free(cc);
  if (result != 0)
    throw std::invalid_argument("Physically impossible cube (edge/corner orientation or parity)");
}
std::string encode_facelets(const std::vector<Vec>& initial, const std::vector<Vec>& slots,
                            const std::vector<Mat>& orientations) {
  if (initial.size() != 20 || slots.size() != 20 || orientations.size() != 20)
    throw std::invalid_argument("Expected 20 pieces");
  std::string s(54, '?');
  for (int i = 0; i < 6; i++) s[9 * i + 4] = order[i];
  auto face_of = [](const Vec& n) {
    for (char f : order) {
      auto [a, sgn] = face_axis(f);
      if (n == Vec::Unit(a) * sgn) return f;
    }
    throw std::invalid_argument("Off-grid face normal");
  };
  std::array<Vec, 6> right = {Vec(1, 0, 0), Vec(0, 1, 0),  Vec(1, 0, 0),
                              Vec(1, 0, 0), Vec(0, -1, 0), Vec(-1, 0, 0)};
  std::array<Vec, 6> down = {Vec(0, -1, 0), Vec(0, 0, -1), Vec(0, 0, -1),
                             Vec(0, 1, 0),  Vec(0, 0, -1), Vec(0, 0, -1)};
  for (int i = 0; i < 20; i++)
    for (int a = 0; a < 3; a++)
      if (initial[i][a]) {
        Vec original = Vec::Unit(a) * initial[i][a];
        char color = face_of(original), f = face_of(orientations[i] * original);
        int fi = order.find(f), col = std::lround(slots[i].dot(right[fi])) + 1,
            row = std::lround(slots[i].dot(down[fi])) + 1;
        if (row < 0 || row > 2 || col < 0 || col > 2 || s[9 * fi + 3 * row + col] != '?')
          throw std::invalid_argument("Overlapping or off-grid facelets");
        s[9 * fi + 3 * row + col] = color;
      }
  validate_facelets(s);
  return s;
}
std::string Cube::facelets() const {
  int core = id(mjOBJ_BODY, "core");
  Mat r = body_rotation(core);
  std::vector<Vec> measured;
  std::vector<Mat> rots;
  for (int i = 0; i < 20; i++) {
    Vec p = r.transpose() * (body_position(piece_ids[i]) - body_position(core)) / cube_pitch;
    Mat m = r.transpose() * body_rotation(piece_ids[i]);
    Vec rounded = p.array().round();
    Mat mr = m.array().round();
    if ((p - rounded).cwiseAbs().maxCoeff() > .02 || (m - mr).cwiseAbs().maxCoeff() > .02)
      throw std::invalid_argument("Cube not aligned; finish current turn");
    measured.push_back(rounded);
    rots.push_back(mr);
  }
  return encode_facelets(initial_slots, measured, rots);
}
std::vector<std::string> solve_facelets(const std::string& state,
                                        const std::filesystem::path& root) {
  validate_facelets(state);
  if (state == solved) return {};
  std::string input = state;
  auto f = get_facecube_fromstring(input.data());
  auto c = toCubieCube(f);
  free(f);
  if (verify(c) != 0) {
    free(c);
    throw std::invalid_argument("Physically impossible cube (edge/corner orientation or parity)");
  }
  free(c);
  auto cache = (root / "modules/cube/third_party/kociemba/cprunetables").string();
  char* answer = solution(input.data(), 24, 30, 0, cache.c_str());
  if (!answer) throw std::runtime_error("Kociemba found no solution within 30 seconds");
  std::string text(answer);
  free(answer);
  auto moves = split_moves(text);
  if (replay_facelet_moves(state, moves) != solved)
    throw std::runtime_error("Solver failed independent facelet replay");
  return moves;
}
std::string replay_facelet_moves(const std::string& state, const std::vector<std::string>& moves) {
  validate_facelets(state);
  // Independent facelet geometry replay; no use of the solver's cubie move tables.
  std::string check = state;
  std::array<Vec, 6> right = {Vec(1, 0, 0), Vec(0, 1, 0),  Vec(1, 0, 0),
                              Vec(1, 0, 0), Vec(0, -1, 0), Vec(-1, 0, 0)},
                     down = {Vec(0, -1, 0), Vec(0, 0, -1), Vec(0, 0, -1),
                             Vec(0, 1, 0),  Vec(0, 0, -1), Vec(0, 0, -1)};
  for (auto& move : moves) {
    auto [face, count] = parse_move(move);
    auto [axis, sign] = face_axis(face);
    Mat rot = rotation(axis, -sign * count * pi / 2).array().round();
    std::string next(54, '?');
    for (int fi = 0; fi < 6; fi++) {
      auto [a, sg] = face_axis(order[fi]);
      for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++) {
          Vec normal = Vec::Unit(a) * sg,
              pos = normal + right[fi] * (col - 1) + down[fi] * (row - 1);
          if (pos[axis] == sign) {
            pos = rot * pos;
            normal = rot * normal;
          }
          int target = -1;
          for (int ti = 0; ti < 6; ti++) {
            auto [ta, ts] = face_axis(order[ti]);
            if (normal == Vec::Unit(ta) * ts) target = ti;
          }
          if (target < 0) throw std::runtime_error("Invalid solver replay normal");
          int rr = std::lround(pos.dot(down[target])) + 1,
              cc = std::lround(pos.dot(right[target])) + 1;
          next[9 * target + 3 * rr + cc] = check[9 * fi + 3 * row + col];
        }
    }
    check = next;
  }
  return check;
}
Json Cube::report() const {
  auto [p, a] = pose_error(true);
  Json warnings = Json::array();
  for (int i = 0; i < mjNWARNING; i++) warnings.push_back(data->warning[i].number);
  Json r = {{"solved", is_solved()},
            {"gripper", dual ? (rx ? "rx_narrow_tip" : "robotiq_2f85") : "none"},
            {"cube_side_m", 3 * cube_pitch},
            {"simulation_time_s", data->time},
            {"moves", history},
            {"max_solved_position_error_m", p},
            {"max_solved_orientation_error_rad", a},
            {"warnings", warnings},
            {"mujoco_version", mj_versionString()},
            {"active_face", active_face},
            {"progress", progress}};
  try {
    r["facelets"] = facelets();
  } catch (const std::invalid_argument&) {
    r["facelets"] = nullptr;
  }
  if (dual) {
    double force = 0, primitive_time = 0;
    for (const auto& action : executed)
      if (action.contains("primitive")) primitive_time += action.value("actual_duration_s", 0.);
    for (int i = 0; i < 6; i++) force = std::max(force, std::abs(data->actuator_force[i]));
    Vec cp = body_position(id(mjOBJ_BODY, "core"));
    r.update({{"orientation", matrix_json(orientation)},
              {"grasped", grasped},
              {"robot_ready", robot_ready},
              {"primitive_actions", executed.size()},
              {"primitive_execution_s", primitive_time},
              {"grasp_checks", grasp_checks.size()},
              {"cube_motor_force_max", force},
              {"core_position_m", {cp.x(), cp.y(), cp.z()}},
              {"core_orientation", matrix_json(body_rotation(id(mjOBJ_BODY, "core")))},
              {"motion_speed_requested", speed},
              {"wrist_speed_requested", wrist_speed},
              {"jaw_speed_requested", jaw_speed},
              {"fast_parallel_fingers", fast},
              {"motion_checks", motion_checks},
              {"current_action", current_action},
              {"grasp_model", "actuator-driven friction-only contact; no grasp weld"},
              {"rotation_clearance", clearance}});
    if (rx)
      r["rx_checks"] = {{"contact_steps", rx_contact_steps},
                        {"max_contact_penetration_m", rx_max_contact_penetration},
                        {"peak_gripper_motor_torque_nm", rx_peak_motor_torque},
                        {"peak_cube_motor_force_during_execution", rx_peak_cube_motor_force}};
  }
  return r;
}
}  // namespace rm::cube
