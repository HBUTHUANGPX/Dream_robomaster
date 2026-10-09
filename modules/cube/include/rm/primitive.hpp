#pragma once
#include "rm/cube.hpp"
namespace rm::cube {
enum class Primitive {
  A_P90,
  A_N90,
  A_P180,
  A_N180,
  A_OPEN,
  A_CLOSE,
  B_P90,
  B_N90,
  B_P180,
  B_N180,
  B_OPEN,
  B_CLOSE
};
std::string primitive_name(Primitive action);
Primitive parse_primitive(const std::string& name);
struct RobotState {
  int orientation = 0;
  std::array<int, 2> wrist{0, 0};
  std::array<bool, 2> closed{true, true};
  bool operator==(const RobotState&) const = default;
};
const std::vector<Mat>& robot_orientations();
int orientation_index(const Mat& q);
int robot_state_id(const RobotState& state);
RobotState robot_state_from_id(int id);
Json robot_state_json(const RobotState& state);
RobotState robot_snapshot(const Cube& cube);
struct PrimitiveTransition {
  RobotState state;
  std::string mode, move;
};
std::optional<PrimitiveTransition> primitive_transition(const RobotState& state, Primitive action);
struct PrimitiveCosts {
  // 列为整块、单层、空转；开合使用第零列。
  std::array<std::array<double, 3>, 12> duration{};
  static PrimitiveCosts defaults(double wrist_speed, double jaw_speed, bool fast);
  void apply_json(const Json& profile);
  double seconds(Primitive action, const std::string& mode) const;
  Json json() const;
};
Json primitive_plan_json(const std::vector<Primitive>& plan, RobotState state,
                         const PrimitiveCosts& costs);
std::vector<std::string> primitive_moves(const std::vector<Primitive>& plan, RobotState state);
void execute_primitives(Cube& cube, const std::vector<Primitive>& plan,
                        const RobotState& expected_start, const Cube::Callback& callback = {});
}  // namespace rm::cube
