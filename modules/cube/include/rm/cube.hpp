#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "rm/sim.hpp"
namespace rm::cube {
using Mat = Eigen::Matrix3d;
using Vec = Eigen::Vector3d;
constexpr double pitch = .020, pi = 3.14159265358979323846;
inline const std::string faces = "RLUDFB", order = "URFDLB";
inline const std::string solved = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";
std::pair<int, int> face_axis(char face);
std::pair<char, int> parse_move(const std::string& move);
std::vector<std::string> split_moves(const std::string& moves);
Mat rotation(int axis, double angle);
Mat wrist_rotation(const std::string& hand, double angle);
Json matrix_json(const Mat& q);
struct Action {
  std::string kind, hand;
  double target = 0;
  std::string mode, move;
  Json json() const;
  bool operator==(const Action&) const = default;
};
using Plan = std::vector<Action>;
Json replay_plan(const Plan& plan, Mat orientation = Mat::Identity());
Plan compile_moves(const std::vector<std::string>& moves, Mat orientation = Mat::Identity());
Plan optimize_plan(const Plan& plan, Mat orientation = Mat::Identity());
Json plan_json(const Plan& plan);
std::string build_scene(const std::filesystem::path& root, bool dual, bool fast = false);
std::string encode_facelets(const std::vector<Vec>& initial, const std::vector<Vec>& slots,
                            const std::vector<Mat>& orientations);
void validate_facelets(const std::string& state);
std::vector<std::string> solve_facelets(const std::string& state,
                                        const std::filesystem::path& root);
struct Cube {
  std::string xml;
  std::unique_ptr<Simulation> sim;
  mjModel* model;
  mjData* data;
  std::vector<Vec> initial_slots, slots;
  std::vector<Mat> orientations;
  std::vector<int> piece_ids;
  std::map<char, double> targets;
  std::string active_face;
  std::vector<std::string> history;
  bool dual, fast, robot_ready = false;
  double speed, wrist_speed, jaw_speed, progress = 0;
  Mat orientation = Mat::Identity();
  std::map<std::string, std::string> grasped{{"A", ""}, {"B", ""}};
  Json current_action = nullptr, executed = Json::array(), motion_checks = Json::array(),
       grasp_checks = Json::array(), clearance;
  std::set<int> cube_bodies;
  std::vector<int> geom_hand;
  std::vector<bool> geom_cube;
  double required_open_gap = std::sqrt(2.) * (3 * pitch + .0002) + .004;
  using Callback = std::function<void(Cube&)>;
  Cube(const std::filesystem::path& root, bool dual = false, double speed = 1,
       std::optional<double> wrist_speed = std::nullopt,
       std::optional<double> jaw_speed = std::nullopt);
  int id(mjtObj type, const std::string& name) const;
  Mat body_rotation(int body) const;
  Vec body_position(int body) const;
  void attach(int index, const std::string& parent, bool ideal = false);
  void step(const Callback& callback = {});
  void advance(double seconds, const Callback& callback = {});
  void turn(const std::string& move, const Callback& callback = {});
  std::pair<double, double> pose_error(bool solved_pose = false) const;
  std::string facelets() const;
  bool is_solved() const;
  Json report() const;
  double open_gap(const std::string& hand) const;
  Json pad_contacts(const std::string& hand) const;
  void wait_open(const std::string& hand, const Callback& callback = {});
  void check_clearance(const Action& action, const Callback& callback = {});
  void move_actuator(const std::string& actuator, double target, double duration,
                     const Callback& callback = {}, const std::string& joint = "");
  void fast_jaw(const std::string& actuator, double target, double duration,
                const Callback& callback = {});
  void grasp(const std::string& hand, const std::string& mode, const Callback& callback = {});
  void release(const std::string& hand);
  void initialize_grasps(const Callback& callback = {});
  void unlock(const std::string& move);
  void lock(const std::string& move, const Callback& callback = {});
  void execute(const Plan& plan, const Callback& callback = {});
};
int interactive_viewer(Cube& cube, const std::filesystem::path& root,
                       const std::vector<std::string>& scramble);
}  // namespace rm::cube
