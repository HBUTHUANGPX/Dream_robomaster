#include <cstdlib>
#include <iostream>

#include "rm/cube.hpp"
using namespace rm::cube;
void require(bool condition, const std::string& why) {
  if (!condition) throw std::runtime_error(why);
}
template <class F>
void rejects(F f, const char* why) {
  try {
    f();
  } catch (const std::exception&) {
    return;
  }
  throw std::runtime_error(why);
}
int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("Need repository root");
    std::filesystem::path root = argv[1];
    for (double bad : {0., -1., std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
      rejects([&] { Cube c(root, true, bad); }, "Invalid speed accepted");
      rejects([&] { Cube c(root, true, 1, bad); }, "Invalid wrist speed accepted");
      rejects([&] { Cube c(root, true, 1, 1, bad); }, "Invalid jaw speed accepted");
    }
    for (auto m : {"", "X", "R3", "RR", "U2'"})
      rejects([&] { parse_move(m); }, "Invalid move accepted");
    for (auto s : {"", "UUUU", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"})
      rejects([&] { solve_facelets(s, root); }, "Invalid facelets accepted");
    std::string impossible = solved;
    std::swap(impossible[5], impossible[10]);
    rejects([&] { solve_facelets(impossible, root); }, "Impossible flipped edge accepted");
    auto moves = split_moves("U R2 F' D B L2");
    auto raw = compile_moves(moves), optimized = optimize_plan(raw);
    require(optimized.size() < raw.size(), "Optimizer didn't reduce actions");
    require(replay_plan(raw) == replay_plan(optimized), "Optimizer changed result");
    require(optimize_plan(optimized) == optimized, "Optimizer not idempotent");
    Plan yaw1, yaw2;
    for (auto& a : raw)
      if (a.kind == "yaw") yaw1.push_back(a);
    for (auto& a : optimized)
      if (a.kind == "yaw") yaw2.push_back(a);
    require(yaw1 == yaw2, "Optimizer changed wrist sequence");
    rejects([&] { replay_plan({{"release", "A"}, {"release", "B"}}); }, "Lost support allowed");
    rejects([&] { replay_plan({{"slide", "A", .1}}); }, "Translation allowed");
    std::vector<Mat> orientations{Mat::Identity()};
    for (size_t i = 0; i < orientations.size(); i++)
      for (auto h : {"A", "B"}) {
        Mat q = wrist_rotation(h, pi / 2) * orientations[i];
        bool seen = false;
        for (auto& p : orientations)
          if (p == q) seen = true;
        if (!seen) orientations.push_back(q);
      }
    require(orientations.size() == 24, "Incorrect cube orientation group");
    const Plan direct_f{{"release", "B"},
                        {"layer_unlock", "B", 0, "", "F"},
                        {"grasp", "B", 0, "face", "F"},
                        {"yaw", "B", -pi / 2, "face", "F"},
                        {"layer_lock", "B", 0, "", "F"},
                        {"release", "B"},
                        {"jaw", "B", 0},
                        {"yaw", "B", 0, "empty"},
                        {"jaw", "B", 115},
                        {"grasp", "B", 0, "core"},
                        {"checkpoint", "", 0, "", "F"}};
    require(compile_moves({"F"}, Mat::Identity(), true) == direct_f,
            "Initial F must turn B directly without whole rotations");
    auto count_whole = [](const Plan& plan) {
      return std::count_if(plan.begin(), plan.end(),
                           [](const Action& a) { return a.kind == "yaw" && a.mode == "whole"; });
    };
    const auto prefix = split_moves("R U F2");
    const auto prefix_b = compile_moves(prefix, Mat::Identity(), true);
    require(count_whole(compile_moves(prefix)) == count_whole(prefix_b) + 2,
            "B face turn must remove two whole rotations from R U F2");
    require(replay_plan(prefix_b)["moves"] == prefix, "B prefix replay mismatch");
    for (auto& q : orientations) {
      auto p = compile_moves(moves, q);
      require(replay_plan(p, q) == replay_plan(optimize_plan(p, q), q),
              "Orientation optimizer mismatch");
      require(p == compile_moves(moves, q, false), "Default planner behavior changed");
      auto mixed = replay_plan(compile_moves(moves, q, true), q);
      require(mixed["moves"] == moves && mixed["yaw_rad"]["A"] == 0 && mixed["yaw_rad"]["B"] == 0,
              "Mixed-hand sequence replay or wrist reset mismatch");
      for (char face : faces)
        for (const auto* suffix : {"", "'", "2"}) {
          const std::vector<std::string> single{std::string(1, face) + suffix};
          const auto legacy = compile_moves(single, q);
          const auto plan = compile_moves(single, q, true);
          const auto replay = replay_plan(plan, q);
          require(legacy == compile_moves(single, q, false), "Disabled B option changed plan");
          require(replay["moves"] == single, "24 orientations x 18 moves replay mismatch");
          require(replay["yaw_rad"]["A"] == 0 && replay["yaw_rad"]["B"] == 0,
                  "B-enabled plan did not reset both wrists");
          require(replay == replay_plan(optimize_plan(plan, q), q),
                  "B-enabled optimizer changed replay");
          auto [axis, sign] = face_axis(face);
          if (q * Vec::Unit(axis) * sign == Vec(0, -1, 0)) {
            require(count_whole(plan) == 0 && replay["orientation"] == matrix_json(q),
                    "Presented B face unnecessarily reoriented cube");
            for (const auto& a : plan)
              require(a.kind == "checkpoint" || a.hand == "B", "Direct B plan commands wrong hand");
          } else {
            require(plan == legacy, "B option changed a face not already presented to B");
          }
        }
    }
    std::array<Vec, 6> initial = {Vec(1, -1, 1),  Vec(-1, 1, -1), Vec(1, -1, 1),
                                  Vec(-1, 1, -1), Vec(1, -1, 1),  Vec(-1, 1, -1)},
                       expected = {Vec(1, 1, 1),    Vec(-1, 1, 1),  Vec(-1, -1, 1),
                                   Vec(-1, -1, -1), Vec(1, -1, -1), Vec(1, 1, -1)};
    int fi = 0;
    for (char f : faces) {
      Cube c(root);
      require(c.facelets() == solved, "Initial facelets not solved");
      c.turn(std::string(1, f));
      int i = 0;
      while (c.initial_slots[i] != initial[fi]) i++;
      require((c.body_position(c.piece_ids[i]) - (Vec(0, 0, .105) + pitch * expected[fi])).norm() <
                  2e-5,
              "Physical corner target mismatch");
      require(c.pose_error().first < 2e-5 && c.pose_error().second < .002,
              "Single turn pose error");
      for (int k = 0; k < 3; k++) c.turn(std::string(1, f));
      require(c.is_solved() && c.facelets() == solved, "Four turns don't solve");
      fi++;
    }
    {
      Cube c(root);
      for (int i = 0; i < c.model->nu; i++) {
        c.model->actuator_gainprm[10 * i] = 0;
        c.model->actuator_biasprm[10 * i + 1] = 0;
        c.model->actuator_biasprm[10 * i + 2] = 0;
      }
      rejects([&] { c.turn("R"); }, "Motor-disabled turn succeeded");
      require(c.history.empty(), "Failed turn recorded");
    }
    {
      Cube c(root);
      for (auto& m : split_moves("R U F' L2 D B R' U2 F D'")) c.turn(m);
      c.history.clear();
      auto solution = solve_facelets(c.facelets(), root);
      require(!solution.empty(), "Scramble solution empty");
      for (auto& m : solution) c.turn(m);
      require(c.facelets() == solved && c.pose_error(true).first < 2e-5 &&
                  c.pose_error(true).second < .002,
              "General solver physical restore failed");
    }
    {
      Cube c(root);
      c.turn("R");
      auto measured = c.facelets();
      c.slots = c.initial_slots;
      std::fill(c.orientations.begin(), c.orientations.end(), Mat::Identity());
      require(measured != solved && c.facelets() == measured, "Facelet reader uses logical cache");
    }
    {
      Cube c(root, true);
      c.initialize_grasps();
      int a = c.id(mjOBJ_ACTUATOR, "A_yaw_drive");
      c.model->actuator_gainprm[10 * a] = 0;
      c.model->actuator_biasprm[10 * a + 1] = 0;
      c.model->actuator_biasprm[10 * a + 2] = 0;
      rejects([&] { c.execute(compile_moves({"R"})); }, "Unpowered wrist falsely completes turn");
      require(c.history.empty(), "Failed wrist turn recorded");
    }
    for (bool fast : {false, true}) {
      Cube c(root, true, 1, fast ? 8 : 1, fast ? 32 : 1);
      for (int i = 0; i < c.model->neq; i++)
        if (c.model->eq_type[i] == mjEQ_WELD || c.model->eq_type[i] == mjEQ_CONNECT) {
          auto ishand = [&](int b) {
            const char* p = mj_id2name(c.model, mjOBJ_BODY, b);
            std::string n = p ? p : "";
            return n.starts_with("A_") || n.starts_with("B_");
          };
          require(ishand(c.model->eq_obj1id[i]) == ishand(c.model->eq_obj2id[i]),
                  "External grasp constraint");
        }
      for (int i = 0; i < c.model->njnt; i++)
        require(c.model->jnt_type[i] != mjJNT_SLIDE, "Unexpected translation joint");
      for (auto& m : split_moves("R U F' L2")) c.turn(m);
      auto solution = solve_facelets(c.facelets(), root);
      c.history.clear();
      c.initialize_grasps();
      require(!c.data->eq_active[c.id(mjOBJ_EQUALITY, "loading_fixture")], "Fixture not released");
      c.execute(optimize_plan(compile_moves(solution)));
      require(c.facelets() == solved, "Dual gripper solution failed");
      for (auto& check : c.motion_checks) {
        std::string a = check["actuator"];
        double force = check["peak_actuator_force"];
        require(force <= (a.ends_with("yaw_drive") ? 15.0001
                          : fast                   ? 40.0001
                                                   : 5.0001),
                "Actuator exceeded force limit");
        if (fast && a.ends_with("yaw_drive")) {
          require(check["peak_joint_velocity_rad_s"].get<double>() > 25,
                  "Fast wrist did not reach requested rate");
          double duration = check["duration_s"];
          require(std::abs(duration - .85 / 8) < .0001 || std::abs(duration - 1.2 / 8) < .0001,
                  "Wrong independent wrist duration");
        }
      }

      require(c.pose_error().first < .0002 && c.pose_error().second < .02, "Dual pose mismatch");
      require(c.clearance["forbidden_contacts"] == 0, "Forbidden contacts");
      require(c.clearance["rotation_steps_checked"].get<int>() > 1000, "No rotation monitoring");
      for (int i = 0; i < 6; i++)
        require(c.data->actuator_force[i] == 0, "Internal motor powered during gripper solve");
      for (int i = 0; i < mjNWARNING; i++)
        require(c.data->warning[i].number == 0, "Numerical warning");
    }
    std::array<double, 2> drops;
    for (int zero = 0; zero < 2; zero++) {
      Cube c(root, true);
      c.initialize_grasps();
      c.release("A");
      c.move_actuator("A_fingers_actuator", 0, .3);
      c.wait_open("A");
      c.move_actuator("B_yaw_drive", pi / 2, 1.2, {}, "B_yaw");
      c.advance(.5);
      double height = c.body_position(c.id(mjOBJ_BODY, "core")).z();
      if (zero) std::fill(c.model->geom_friction, c.model->geom_friction + c.model->ngeom * 3, 0);
      c.advance(1);
      drops[zero] = height - c.body_position(c.id(mjOBJ_BODY, "core")).z();
    }
    require(std::abs(drops[0]) < .001 && drops[1] > .01, "Friction loss control failed");
    std::cout << "Cube behavior passed: six physical faces, general solver without history, all 24 "
                 "orientations, optimized support, baseline/8x32x friction solves and "
                 "zero-friction slip.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
