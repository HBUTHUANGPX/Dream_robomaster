#include <iostream>

#include "rm/robot_search.hpp"
using namespace rm::cube;
int main(int argc, char** argv) {
  try {
    if (argc < 2) throw std::invalid_argument("Need repository root");
    std::filesystem::path root = argv[1];
    std::vector<std::vector<Primitive>> sequences;
    if (argc > 2) {
      std::vector<Primitive> seq;
      for (int i = 2; i < argc; ++i) seq.push_back(parse_primitive(argv[i]));
      sequences.push_back(seq);
    } else {
      for (int h = 0; h < 2; ++h)
        for (int k = 0; k < 4; ++k)
          sequences.push_back({Primitive(h * 6 + k), Primitive(h * 6 + (k ^ 1))});
      sequences.push_back(
          {Primitive::A_N180, Primitive::B_N90, Primitive::B_P90, Primitive::A_P180});
      sequences.push_back({Primitive::A_OPEN, Primitive::B_P90, Primitive::A_CLOSE,
                           Primitive::B_OPEN, Primitive::B_N90, Primitive::B_CLOSE});
    }
    for (auto& seq : sequences) {
      Cube c(root, true, 1, 8, 32);
      c.initialize_grasps();
      std::string expected = solved;
      for (auto a : seq) {
        std::cout << primitive_name(a) << ' ' << std::flush;
        auto start = robot_snapshot(c);
        auto t = primitive_transition(start, a);
        if (!t) throw std::runtime_error("Invalid test sequence");
        if (!t->move.empty()) expected = replay_facelet_moves(expected, {t->move});
        execute_primitives(c, {a}, start);
        if (c.facelets() != expected)
          throw std::runtime_error("Intermediate physical facelets disagree");
      }

      if (c.clearance["forbidden_contacts"] != 0) throw std::runtime_error("Forbidden collision");
      for (int i = 0; i < 6; ++i)
        if (c.data->actuator_force[i] != 0) throw std::runtime_error("Internal cube motor used");
      std::cout << "passed\n";
    }
    if (argc == 2) {
      Cube slow(root, true);
      slow.initialize_grasps();
      execute_primitives(slow, {Primitive::B_P90}, robot_snapshot(slow));
      if (slow.facelets() != replay_facelet_moves(solved, {"F\'"}))
        throw std::runtime_error("Slow B wrist direction");
    }
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
