#include <iostream>

#include "rm/primitive.hpp"
using namespace rm::cube;
void need(bool v, const char* text) {
  if (!v) throw std::runtime_error(text);
}
int main() {
  try {
    RobotState start;
    need(robot_orientations().size() == 24, "orientation group");
    for (int i = 0; i < 12; ++i)
      need(parse_primitive(primitive_name(Primitive(i))) == Primitive(i), "names");
    auto a = primitive_transition(start, Primitive::A_P90);
    need(a && a->move == "R'" && a->state.wrist[0] == 1, "relative A face");
    need(!primitive_transition(a->state, Primitive::B_N90), "cross-wrist collision");
    auto b = primitive_transition(start, Primitive::B_N90);
    need(b && b->move == "F" && b->state.wrist[1] == -1, "relative B face");
    auto half = primitive_transition(start, Primitive::A_P180);
    need(half && !primitive_transition(half->state, Primitive::A_P90), "wrist range");
    need(primitive_transition(half->state, Primitive::A_N180)->state.wrist[0] == 0,
         "relative return");
    auto opened = primitive_transition(start, Primitive::A_OPEN);
    need(opened && !primitive_transition(opened->state, Primitive::B_OPEN), "lost support");
    need(!primitive_transition(opened->state, Primitive::A_OPEN), "redundant open");
    auto whole = primitive_transition(opened->state, Primitive::B_P90);
    need(whole && whole->mode == "whole" && whole->move.empty() && whole->state.orientation != 0,
         "whole cube");
    auto empty = primitive_transition(opened->state, Primitive::A_P90);
    need(empty && empty->mode == "empty" && empty->state.orientation == 0, "empty wrist");
    need(primitive_transition(empty->state, Primitive::A_CLOSE).has_value(), "nonzero close");
    auto cost = PrimitiveCosts::defaults(8, 32, true);
    need(cost.seconds(Primitive::A_P90, "face") > cost.seconds(Primitive::A_P90, "empty"),
         "face overhead");
    cost.apply_json({{"duration_s", {{"A_P90", 2.0}}}});
    need(cost.seconds(Primitive::A_P90, "face") == 2, "custom duration");
    std::cout << "Primitive symbolic tests passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
