#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include "rm/robot_search.hpp"

namespace {
using namespace rm::cube;
using Clock = std::chrono::steady_clock;
struct Case {
  std::string name;
  std::vector<Primitive> actions;
  bool fast_scramble = false;
};

double elapsed(Clock::time_point began) {
  return std::chrono::duration<double>(Clock::now() - began).count();
}

void check_physics(Cube& c) {
  if (c.model->nu < 6) throw std::runtime_error("Missing internal cube motors");
  if (c.rx_peak_cube_motor_force != 0)
    throw std::runtime_error("Internal cube motor peak force is nonzero: " +
                             std::to_string(c.rx_peak_cube_motor_force));
  for (int i = 0; i < 6; ++i)
    if (c.data->actuator_force[i] != 0)
      throw std::runtime_error("Internal cube motor used: " + std::to_string(i));
  if (c.clearance.at("forbidden_contacts") != 0) throw std::runtime_error("Forbidden collision");
  for (int i = 0; i < mjNWARNING; ++i)
    if (c.data->warning[i].number != 0)
      throw std::runtime_error("MuJoCo warning: " + std::to_string(i));
}

void diagnose(Cube& c) {
  // 分别收集诊断，避免色块读取失败掩盖接触或朝向信息。
  try {
    std::cerr << "report: " << c.report().dump(2) << '\n';
  } catch (const std::exception& e) {
    std::cerr << "report unavailable: " << e.what() << '\n';
  }
  std::cerr << "current_action: " << c.current_action.dump() << '\n';
  std::cerr << "rx_peak_cube_motor_force: " << c.rx_peak_cube_motor_force << '\n';
  std::cerr << "grasp_checks: " << c.grasp_checks.dump() << '\n';
  for (const auto* hand : {"A", "B"}) {
    try {
      std::cerr << hand << " contacts: " << c.pad_contacts(hand).dump() << '\n';
    } catch (const std::exception& e) {
      std::cerr << hand << " contacts unavailable: " << e.what() << '\n';
    }
  }
  try {
    const Mat physical = c.body_rotation(c.id(mjOBJ_BODY, "core"));
    const Mat drift = c.orientation.transpose() * physical;
    const double angle = std::acos(std::clamp((drift.trace() - 1.) / 2., -1., 1.));
    std::cerr << "core_physical_orientation: " << matrix_json(physical).dump() << '\n'
              << "core_logical_orientation: " << matrix_json(c.orientation).dump() << '\n'
              << "core_orientation_drift_rad: " << angle << '\n'
              << "core_orientation_drift_deg: " << angle * 180. / pi << '\n';
  } catch (const std::exception& e) {
    std::cerr << "core orientation unavailable: " << e.what() << '\n';
  }
}

bool run_case(const std::filesystem::path& root, const std::filesystem::path& bundle,
              const Case& test, double wrist_speed, double jaw_speed) {
  const auto began = Clock::now();
  std::unique_ptr<Cube> cube;
  size_t step = 0;
  std::string phase = "construct", expected = solved;
  try {
    cube = std::make_unique<Cube>(root, true, test.fast_scramble ? 4 : 1, wrist_speed, jaw_speed,
                                  bundle);
    auto& c = *cube;
    if (test.fast_scramble) {
      phase = "fast_scramble";
      const auto moves = split_moves("R U F' L2 D B R' U2 F D' L B2 U' R2 F2 D L' U B' R");
      const double start = c.data->time;
      for (const auto& move : moves) c.turn(move);
      expected = replay_facelet_moves(solved, moves);
      if (c.data->time - start > 10 ||
          (c.body_position(c.id(mjOBJ_BODY, "core")) - Vec(0, 0, .22)).norm() > .00005)
        throw std::runtime_error("Fast scramble too slow or loading fixture oscillating");
    }
    phase = "initialize_grasps";
    c.initialize_grasps(check_physics);
    check_physics(c);
    if (c.facelets() != expected) throw std::runtime_error("Initial physical cube state mismatch");
    for (auto action : test.actions) {
      ++step;
      phase = primitive_name(action);
      std::cout << test.name << " step " << step << '/' << test.actions.size() << ' ' << phase
                << '\n'
                << std::flush;
      const auto start = robot_snapshot(c);
      const auto transition = primitive_transition(start, action);
      if (!transition) throw std::runtime_error("Invalid test sequence transition");
      const std::vector<std::string> moves = transition->move.empty()
                                                 ? std::vector<std::string>{}
                                                 : std::vector<std::string>{transition->move};
      expected = replay_facelet_moves(expected, moves);
      execute_primitives(c, {action}, start, check_physics);
      check_physics(c);
      const auto physical = c.facelets();
      if (physical != expected)
        throw std::runtime_error("Physical facelets disagree: expected=" + expected +
                                 " physical=" + physical);
    }
    std::cout << "PASS " << test.name << " elapsed_s=" << elapsed(began) << '\n';
    return true;
  } catch (const std::exception& e) {
    std::cerr << "FAIL " << test.name << " step=" << step << '/' << test.actions.size()
              << " phase=" << phase << " elapsed_s=" << elapsed(began) << ": " << e.what()
              << "\nexpected_facelets: " << expected << '\n';
    if (cube) diagnose(*cube);
    return false;
  }
}

bool check_margin(const std::filesystem::path& root, const std::filesystem::path& bundle,
                  double wrist_speed, double jaw_speed) {
  const auto began = Clock::now();
  std::unique_ptr<Cube> cube;
  std::string phase = "construct";
  try {
    cube = std::make_unique<Cube>(root, true, 1, wrist_speed, jaw_speed, bundle);
    auto& c = *cube;
    phase = "initialize_grasps";
    c.initialize_grasps(check_physics);
    check_physics(c);
    const std::array<int, 2> tips{c.id(mjOBJ_GEOM, "A_right_tip"), c.id(mjOBJ_GEOM, "A_left_tip")};
    auto margin_contacts = [&]() {
      rm::Json contacts = rm::Json::array();
      for (int i = 0; i < c.data->ncon; ++i) {
        const auto& contact = c.data->contact[i];
        const int g1 = contact.geom[0], g2 = contact.geom[1];
        if (g1 < 0 || g2 < 0) continue;
        const int h1 = c.geom_hand[g1], h2 = c.geom_hand[g2];
        // 排除穿透触发禁触的可能，确保本例只能由正距离载荷触发。
        if (contact.dist < 0 && ((h1 >= 0 && h2 >= 0 && h1 != h2) || (h1 >= 0 && c.geom_cube[g2]) ||
                                 (h2 >= 0 && c.geom_cube[g1])))
          throw std::runtime_error("Margin regression requires nonpenetrating gripper contacts");
        for (int tip : tips) {
          const int other = g1 == tip ? g2 : g2 == tip ? g1 : -1;
          if (other < 0 || !c.geom_cube[other]) continue;
          const int body = c.model->geom_bodyid[other];
          const auto piece = std::find(c.piece_ids.begin(), c.piece_ids.end(), body);
          const bool right_layer =
              body == c.id(mjOBJ_BODY, "center_R") ||
              (piece != c.piece_ids.end() && c.slots[piece - c.piece_ids.begin()][0] == 1);
          mjtNum force[6] = {};
          mj_contactForce(c.model, c.data, i, force);
          if (right_layer && contact.dist > 0 && force[0] > .01)
            contacts.push_back({{"contact", i},
                                {"tip", tip},
                                {"cube_geom", other},
                                {"distance_m", contact.dist},
                                {"normal_force_n", force[0]}});
        }
      }
      if (contacts.empty())
        throw std::runtime_error("No positive-distance A-tip/R-layer contact carrying >0.01 N");
      return contacts;
    };
    phase = "confirm_loaded_margin";
    std::cout << "initialized_margin_contacts: " << margin_contacts().dump() << '\n';
    phase = "unlock_R_A";
    c.unlock("R", "A");
    std::cout << "unlocked_margin_contacts: " << margin_contacts().dump() << '\n';
    // 正确身份应允许现有承载；错误身份将 A 判为不应承载 R 层的支撑手。
    phase = "correct_hand_monitor";
    c.check_clearance(Action{"yaw", "A", 0, "face", "R"});
    check_physics(c);
    const int before = c.clearance.at("forbidden_contacts").get<int>();
    phase = "wrong_hand_monitor";
    std::string rejection;
    try {
      c.check_clearance(Action{"yaw", "B", 0, "face", "R"});
    } catch (const std::runtime_error& e) {
      rejection = e.what();
    }
    const int after = c.clearance.at("forbidden_contacts").get<int>();
    if (!rejection.starts_with("In-place rotation collision between geoms ") || after <= before)
      throw std::runtime_error("Wrong-layer margin load was not rejected with a collision count");
    std::cout << "PASS check-margin forbidden_contacts=" << after << " rejection=" << rejection
              << " elapsed_s=" << elapsed(began) << '\n';
    return true;
  } catch (const std::exception& e) {
    std::cerr << "FAIL check-margin phase=" << phase << " elapsed_s=" << elapsed(began) << ": "
              << e.what() << '\n';
    if (cube) diagnose(*cube);
    return false;
  }
}

std::vector<Case> default_cases() {
  std::vector<Case> tests;
  for (int hand = 0; hand < 2; ++hand) {
    const int base = hand * 6, other = (1 - hand) * 6;
    const auto open = Primitive(base + 4), close = Primitive(base + 5);
    const auto other_open = Primitive(other + 4), other_close = Primitive(other + 5);
    tests.push_back({primitive_name(open) + "_close", {open, close}});
    for (int turn = 0; turn < 4; ++turn) {
      const auto forward = Primitive(base + turn), inverse = Primitive(base + (turn ^ 1));
      tests.push_back({primitive_name(forward) + "_face_inverse", {forward, inverse}});
      // 另一手张开后整块旋转；换手支撑后，原旋转手空返到零。
      tests.push_back({primitive_name(forward) + "_whole_handoff_empty",
                       {other_open, forward, other_close, open, inverse, close}});
    }
    // 支撑腕的正负半转对应不同机构姿态，分别覆盖另一腕的四种转角。
    for (int support_turn : {2, 3}) {
      const auto support = Primitive(base + support_turn);
      const auto restore = Primitive(base + (support_turn ^ 1));
      for (int turn = 0; turn < 4; ++turn) {
        const auto forward = Primitive(other + turn), inverse = Primitive(other + (turn ^ 1));
        tests.push_back({primitive_name(support) + "_support_" + primitive_name(forward),
                         {support, forward, inverse, restore}});
      }
    }
  }
  tests.push_back({"fast_scramble_handoff", {Primitive::B_N90, Primitive::B_P90}, true});
  return tests;
}
}  // namespace

int main(int argc, char** argv) {
  const auto began = Clock::now();
  try {
    if (argc < 3)
      throw std::invalid_argument(
          "Usage: test_rx_physics root bundle [--wrist-speed N] [--jaw-speed N] "
          "[--check-margin | primitive names...]");
    const std::filesystem::path root = argv[1], bundle = argv[2];
    if (bundle.empty() || !std::filesystem::exists(bundle))
      throw std::invalid_argument("RX bundle does not exist: " + bundle.string());
    double wrist_speed = 1, jaw_speed = 1;
    bool margin = false;
    Case custom{"custom", {}};
    for (int i = 3; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--wrist-speed" || arg == "--jaw-speed") {
        if (++i == argc) throw std::invalid_argument("Missing speed value");
        const std::string value = argv[i];
        size_t used = 0;
        const double speed = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(speed) || speed <= 0 || speed > 1000)
          throw std::invalid_argument("Speed must be finite and in (0,1000]");
        (arg == "--wrist-speed" ? wrist_speed : jaw_speed) = speed;
      } else if (arg == "--check-margin") {
        margin = true;
      } else {
        custom.actions.push_back(parse_primitive(arg));
      }
    }
    std::cout << "wrist_speed=" << wrist_speed << " jaw_speed=" << jaw_speed << '\n';
    if (margin) {
      if (!custom.actions.empty())
        throw std::invalid_argument("--check-margin does not accept primitive names");
      return check_margin(root, bundle, wrist_speed, jaw_speed) ? 0 : 1;
    }
    std::vector<Case> tests;
    if (!custom.actions.empty()) {
      tests.push_back(std::move(custom));
    } else {
      tests = default_cases();
    }
    size_t failures = 0;
    for (const auto& test : tests)
      if (!run_case(root, bundle, test, wrist_speed, jaw_speed)) ++failures;
    std::cout << "cases=" << tests.size() << " failures=" << failures
              << " elapsed_s=" << elapsed(began) << '\n';
    return failures ? 1 : 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << " elapsed_s=" << elapsed(began) << '\n';
    return 1;
  }
}
