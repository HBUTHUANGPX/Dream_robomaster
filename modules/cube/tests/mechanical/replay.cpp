#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rm/cube.hpp"
using namespace rm;
using namespace rm::cube;

// 仅供故障复现：在独立进程初始化时恢复快照，不能把本程序输出拼成完整物理还原录像。
int main(int argc, char** argv) {
  try {
    if (argc < 4 || argc > 5)
      throw std::invalid_argument("用法: replay 场景XML 状态JSON 输出目录 [参数JSON]");
    Json saved;
    std::ifstream(argv[2]) >> saved;
    Json parameters = saved.at("parameters");
    if (!parameters.is_object())
      throw std::invalid_argument("Snapshot parameters must be an object");
    const std::set<std::string> allowed{"turn_duration_s",           "timestep_s",
                                        "rx_jaw_torque_nm",          "rx_detent_torque_nm",
                                        "settle_duration_s",         "tip_torsional_friction_m",
                                        "internal_contact_impedance"};
    if (argc == 5) {
      Json custom;
      std::ifstream(argv[4]) >> custom;
      if (!custom.is_object()) throw std::invalid_argument("Replay overrides must be an object");
      for (const auto& [key, value] : custom.items())
        if (!allowed.contains(key))
          throw std::invalid_argument("Unsupported replay override: " + key);
      parameters.update(custom);
    }
    for (const auto& key : allowed) {
      if (!parameters.contains(key)) continue;
      const auto& item = parameters.at(key);
      if (!item.is_number() || !std::isfinite(item.get<double>()))
        throw std::invalid_argument("Invalid replay parameter: " + key);
      const double value = item.get<double>();
      if (value < 0 || ((key == "turn_duration_s" || key == "timestep_s") && value == 0) ||
          (key == "timestep_s" && value > .001) ||
          (key == "internal_contact_impedance" && (value <= 0 || value >= 1)))
        throw std::invalid_argument("Invalid replay parameter: " + key);
    }
    for (const auto* key :
         {"turn_duration_s", "timestep_s", "rx_jaw_torque_nm", "rx_detent_torque_nm"})
      if (!parameters.contains(key))
        throw std::invalid_argument("Missing replay parameter: " + std::string(key));
    const double timestep = parameters.at("timestep_s");
    const double ramp_steps =
        std::max(1., std::round(parameters.at("turn_duration_s").get<double>() / timestep));
    const double settle_steps = std::round(parameters.value("settle_duration_s", .15) / timestep);
    const double total_steps = ramp_steps + settle_steps;
    if (!std::isfinite(total_steps) || total_steps > std::numeric_limits<int>::max() - 1.)
      throw std::invalid_argument("Replay duration/timestep exceeds step limit");
    std::filesystem::path output = argv[3];
    std::filesystem::create_directories(output);
    Simulation sim(argv[1]);
    auto* m = sim.model;
    auto* d = sim.data;
    auto state = saved.at("state").get<std::vector<mjtNum>>();
    int spec = saved.at("state_spec");
    if (state.size() != size_t(mj_stateSize(m, spec)))
      throw std::runtime_error("State/model size mismatch");
    mj_setState(m, d, state.data(), spec);
    m->opt.timestep = parameters.at("timestep_s").get<double>();
    if (parameters.contains("internal_contact_impedance"))
      for (int g = 0; g < m->ngeom; ++g) {
        const char* body = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
        if (body && (std::string_view(body).starts_with("piece_") ||
                     std::string_view(body) == "mechanical_core")) {
          m->geom_solimp[5 * g] = parameters.at("internal_contact_impedance");
          m->geom_solimp[5 * g + 1] = parameters.at("internal_contact_impedance");
        }
      }
    if (parameters.contains("tip_torsional_friction_m"))
      for (int p = 0; p < m->npair; ++p) {
        const char* n = mj_id2name(m, mjOBJ_GEOM, m->pair_geom1[p]);
        const char* other = mj_id2name(m, mjOBJ_GEOM, m->pair_geom2[p]);
        if ((n && std::string_view(n).ends_with("_tip")) ||
            (other && std::string_view(other).ends_with("_tip")))
          m->pair_friction[5 * p + 2] = parameters.at("tip_torsional_friction_m");
      }
    for (auto h : {"A", "B"}) {
      int a = mj_name2id(m, mjOBJ_ACTUATOR, (std::string(h) + "_fingers_actuator").c_str());
      m->actuator_forcerange[2 * a] = -parameters.at("rx_jaw_torque_nm").get<double>();
      m->actuator_forcerange[2 * a + 1] = -m->actuator_forcerange[2 * a];
    }
    mj_forward(m, d);
    int core = mj_name2id(m, mjOBJ_BODY, "mechanical_core");
    auto position = [&]() -> Vec { return Eigen::Map<const Vec>(d->xpos + 3 * core); };
    auto rotation_of = [&](int b) -> Mat {
      return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d->xmat + 9 * b);
    };
    auto action = saved.at("action");
    std::string hand = action.at("hand"), mode = action.at("mode");
    bool jaw = action.at("kind") == "jaw";
    if ((hand != "A" && hand != "B") || (!jaw && action.at("kind") != "yaw"))
      throw std::invalid_argument("Replay requires a jaw or yaw action for A/B");
    double target = action.at("target");
    if (jaw) {
      if (target != 0 && target != 115) throw std::invalid_argument("Invalid jaw command");
      target = target == 115 ? -1.20 : -2.25;
    }
    int actuator =
        mj_name2id(m, mjOBJ_ACTUATOR, (hand + (jaw ? "_fingers_actuator" : "_yaw_drive")).c_str());
    if (actuator < 0 || core < 0) throw std::invalid_argument("Replay requires an RX cube scene");
    double start = d->ctrl[actuator], began = d->time;
    Mat expected_core;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) expected_core(i, j) = saved["robot_orientation"][i][j];
    if (mode == "whole") expected_core = wrist_rotation(hand, target - start) * expected_core;
    struct Piece {
      int body, site;
      Vec slot;
      Mat rotation;
    };
    std::vector<Piece> pieces;
    for (int b = 1; b < m->nbody; ++b) {
      const char* n = mj_id2name(m, mjOBJ_BODY, b);
      if (!n || !std::string_view(n).starts_with("piece_")) continue;
      int site = mj_name2id(m, mjOBJ_SITE, (std::string(n) + "_center").c_str());
      Vec slot = (rotation_of(core).transpose() *
                  (Eigen::Map<const Vec>(d->site_xpos + 3 * site) - position()) / (.055 / 3))
                     .array()
                     .round();
      Mat orientation = (rotation_of(core).transpose() * rotation_of(b)).array().round();
      if (mode == "face") {
        auto [f, c] = parse_move(action.at("move"));
        auto [axis, sign] = face_axis(f);
        if (slot[axis] == sign) {
          Mat r = rotation(axis, -sign * c * pi / 2).array().round();
          slot = r * slot;
          orientation = r * orientation;
        }
      }
      pieces.push_back({b, site, slot, orientation});
    }
    const int ramp = static_cast<int>(ramp_steps);
    Json report = {{"mode", "diagnostic_replay"},
                   {"acceptance", "pose_only"},
                   {"action", action},
                   {"parameters", parameters},
                   {"trace", Json::array()}};
    report["initial_tip_contacts"] = Json::array();
    for (int i = 0; i < d->ncon; ++i) {
      const auto& c = d->contact[i];
      for (int side = 0; side < 2; ++side) {
        const char* tip = mj_id2name(m, mjOBJ_GEOM, c.geom[side]);
        if (!tip || !std::string_view(tip).ends_with("_tip")) continue;
        int body = m->geom_bodyid[c.geom[1 - side]];
        mjtNum f[6];
        mj_contactForce(m, d, i, f);
        report["initial_tip_contacts"].push_back(
            {{"tip", tip}, {"body", mj_id2name(m, mjOBJ_BODY, body)}, {"normal_n", f[0]}});
      }
    }
    double peak = 0;
    bool escaped = false;
    auto contact_name = [&](int g) {
      const char* body = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
      const char* geom = mj_id2name(m, mjOBJ_GEOM, g);
      const char* mesh =
          m->geom_type[g] == mjGEOM_MESH ? mj_id2name(m, mjOBJ_MESH, m->geom_dataid[g]) : nullptr;
      return Json{
          {"body", body ? body : ""}, {"geom", geom ? geom : ""}, {"mesh", mesh ? mesh : ""}};
    };
    for (int k = 1; k <= static_cast<int>(total_steps); ++k) {
      double u = std::min(1., double(k) / ramp);
      d->ctrl[actuator] = start + (target - start) * u * u * u * (10 + u * (-15 + 6 * u));
      for (char f : faces) {
        int j = mj_name2id(m, mjOBJ_JOINT, ("hinge_" + std::string(1, f)).c_str()),
            v = m->jnt_dofadr[j];
        d->qfrc_applied[v] = -parameters.at("rx_detent_torque_nm").get<double>() *
                                 std::sin(4 * d->qpos[m->jnt_qposadr[j]]) -
                             .0005 * d->qvel[v];
      }
      mj_step(m, d);
      double min_distance = 0, force = 0;
      int worst = -1;
      for (int i = 0; i < d->ncon; ++i) {
        mjtNum f[6];
        mj_contactForce(m, d, i, f);
        if (f[0] > force) {
          force = f[0];
          worst = i;
        }
        min_distance = std::min(min_distance, d->contact[i].dist);
      }
      if (force > peak) {
        peak = force;
        report["peak_contact"] = {{"time_s", d->time},
                                  {"normal_n", force},
                                  {"geom0", contact_name(d->contact[worst].geom[0])},
                                  {"geom1", contact_name(d->contact[worst].geom[1])}};
      }
      if (k % 20 == 0 || force > 100)
        report["trace"].push_back(
            {{"time_s", d->time},
             {"contacts", d->ncon},
             {"max_normal_n", force},
             {"min_distance_m", min_distance},
             {"core_position", {position().x(), position().y(), position().z()}}});
      escaped = (position() - Vec(0, 0, .1)).norm() > .005;
      if (escaped || d->ncon > 1000) {
        report["stopped_early"] = true;
        break;
      }
      for (int i = 0; i < mjNWARNING; ++i)
        if (d->warning[i].number) throw std::runtime_error("MuJoCo warning");
    }
    mj_forward(m, d);
    double pos_error = 0, angle_error = 0;
    report["pieces"] = Json::array();
    for (const auto& p : pieces) {
      double ep = (rotation_of(core).transpose() *
                       (Eigen::Map<const Vec>(d->site_xpos + 3 * p.site) - position()) -
                   (.055 / 3) * p.slot)
                      .norm();
      double er = std::acos(std::clamp(
          ((p.rotation.transpose() * rotation_of(core).transpose() * rotation_of(p.body)).trace() -
           1) /
              2,
          -1., 1.));
      pos_error = std::max(pos_error, ep);
      angle_error = std::max(angle_error, er);
      report["pieces"].push_back({{"name", mj_id2name(m, mjOBJ_BODY, p.body)},
                                  {"position_error_m", ep},
                                  {"angle_error_rad", er}});
    }
    double core_angle = std::acos(
        std::clamp(((expected_core.transpose() * rotation_of(core)).trace() - 1) / 2, -1., 1.));
    report["position_error_m"] = pos_error;
    report["orientation_error_rad"] = angle_error;
    report["core_orientation_error_rad"] = core_angle;
    auto core_error = Eigen::AngleAxisd(rotation_of(core) * expected_core.transpose());
    Vec error_vector = core_error.axis() * core_error.angle();
    report["core_rotation_error_vector"] = {error_vector.x(), error_vector.y(), error_vector.z()};
    report["core_position_m"] = {position().x(), position().y(), position().z()};
    report["duration_s"] = d->time - began;
    report["pose_passed"] = !report.value("stopped_early", false) && pos_error < .0005 &&
                            angle_error < .02 && core_angle < .02 &&
                            (position() - Vec(0, 0, .1)).norm() < .001;
    std::ofstream file(output / "report.json");
    file << report.dump(2);
    file.close();
    if (!file) throw std::runtime_error("Cannot write replay report");
    // 仅供后续独立诊断；不作为连续录像的续录点。
    mj_getState(m, d, state.data(), spec);
    saved["state"] = state;
    saved["parameters"] = parameters;
    saved["robot_orientation"] = matrix_json(expected_core);
    saved["source"] = "diagnostic_replay_not_continuous_recording";
    std::ofstream end_state(output / "end_state.json");
    end_state << saved.dump();
    end_state.close();
    if (!end_state) throw std::runtime_error("Cannot write diagnostic end state");
    Renderer renderer(m);
    mjvCamera camera;
    mjv_defaultCamera(&camera);
    camera.lookat[2] = .1;
    camera.distance = .4;
    camera.azimuth = 130;
    camera.elevation = -25;
    cv::Mat rgb = renderer.render(d, camera), bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    if (!cv::imwrite((output / "final.png").string(), bgr))
      throw std::runtime_error("Cannot write replay image");
    std::cout << Json{{"acceptance", "pose_only"},
                      {"pose_passed", report["pose_passed"]},
                      {"position_error_m", pos_error},
                      {"angle_error_rad", angle_error},
                      {"peak_contact", report["peak_contact"]}}
              << '\n';
    return report["pose_passed"].get<bool>() ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
