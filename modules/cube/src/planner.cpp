#include <sstream>

#include "rm/cube.hpp"
namespace rm::cube {
std::pair<int, int> face_axis(char f) {
  switch (f) {
    case 'R':
      return {0, 1};
    case 'L':
      return {0, -1};
    case 'U':
      return {2, 1};
    case 'D':
      return {2, -1};
    case 'F':
      return {1, -1};
    case 'B':
      return {1, 1};
    default:
      throw std::invalid_argument("Unknown cube face");
  }
}
std::pair<char, int> parse_move(const std::string& m) {
  if (m.empty() || m.size() > 2 || faces.find(m[0]) == std::string::npos ||
      (m.size() == 2 && m[1] != '\'' && m[1] != '2'))
    throw std::invalid_argument("Invalid face move: " + m);
  return {m[0], m.size() == 1 ? 1 : m[1] == '2' ? 2 : -1};
}
std::vector<std::string> split_moves(const std::string& s) {
  std::istringstream in(s);
  std::vector<std::string> r;
  std::string m;
  while (in >> m) {
    parse_move(m);
    r.push_back(m);
  }
  if (r.size() > 1000) throw std::invalid_argument("At most 1000 moves per request");
  return r;
}
Mat rotation(int axis, double angle) {
  return Eigen::AngleAxisd(angle, Vec::Unit(axis)).toRotationMatrix();
}
static int hand_index(const std::string& h) {
  if (h == "A") return 0;
  if (h == "B") return 1;
  throw std::invalid_argument("Unknown gripper");
}
Mat wrist_rotation(const std::string& h, double angle) {
  int a = hand_index(h);
  if (!std::isfinite(angle) || std::abs(angle / (pi / 2) - std::round(angle / (pi / 2))) > 1e-7)
    throw std::invalid_argument("Rotation must use complete quarter turns");
  return rotation(a, (a == 0 ? 1 : -1) * angle).array().round().matrix();
}
Json matrix_json(const Mat& q) {
  Json out = Json::array();
  for (int i = 0; i < 3; i++) out.push_back({q(i, 0), q(i, 1), q(i, 2)});
  return out;
}
static void checked(const Mat& q) {
  if (!q.allFinite() || !(q.transpose() * q).isApprox(Mat::Identity(), 1e-10) ||
      std::abs(q.determinant() - 1) > 1e-10 || (q - q.array().round().matrix()).norm() > 1e-10)
    throw std::invalid_argument("Expected proper integer cube orientation");
}
Json Action::json() const {
  return {{"kind", kind}, {"hand", hand}, {"target", target}, {"mode", mode}, {"move", move}};
}
Json plan_json(const Plan& p) {
  Json r = Json::array();
  for (auto& a : p) r.push_back(a.json());
  return r;
}
Json replay_plan(const Plan& p, Mat q) {
  checked(q);
  std::array<std::string, 2> g = {"core", "core"};
  std::array<double, 2> jaw = {115, 115}, yaw = {0, 0};
  std::string unlocked;
  bool turned = false;
  std::vector<std::string> moves;
  auto need = [](bool v, const char* why) {
    if (!v) throw std::invalid_argument(why);
  };
  for (auto& a : p) {
    int h = a.hand.empty() ? -1 : hand_index(a.hand), other = h == 0 ? 1 : 0;
    need(std::isfinite(a.target), "Nonfinite action target");
    if (a.kind != "checkpoint") need(h >= 0, "Action requires hand");
    if (a.kind == "release") {
      need(g[other] == "core", "Release removes support");
      g[h] = "";
    } else if (a.kind == "jaw") {
      need(a.target == 0 || a.target == 115, "Jaw target must be 0 or 115");
      need(a.target != 0 || g[h].empty(), "Release before opening");
      jaw[h] = a.target;
    } else if (a.kind == "grasp") {
      need(jaw[h] == 115, "Grasp needs closed jaws");
      need(a.mode == "core" || a.mode == "face", "Invalid grasp mode");
      need(a.mode != "face" || (unlocked == a.move && g[other] == "core"),
           "Face grasp needs support and unlocked layer");
      g[h] = a.mode;
    } else if (a.kind == "layer_unlock") {
      auto [f, c] = parse_move(a.move);
      auto [axis, sign] = face_axis(f);
      need(unlocked.empty() && g[h].empty() && g[other] == "core",
           "Layer release needs independent support");
      Vec expected = h == 0 ? Vec(1, 0, 0) : Vec(0, -1, 0);
      need((q * Vec::Unit(axis) * sign - expected).norm() < 1e-9, "Face not presented to wrist");
      unlocked = a.move;
      turned = false;
    } else if (a.kind == "yaw") {
      double delta = a.target - yaw[h];
      if (a.mode == "whole") {
        need(g[h] == "core" && g[other].empty() && jaw[other] == 0 && unlocked.empty(),
             "Whole rotation needs sole support and clearance");
        q = wrist_rotation(a.hand, delta) * q;
      } else if (a.mode == "face") {
        auto [f, c] = parse_move(a.move);
        need(g[h] == "face" && g[other] == "core" && unlocked == a.move &&
                 std::abs(delta + c * pi / 2) < 1e-8,
             "Incorrect face rotation");
        turned = true;
      } else if (a.mode == "empty")
        need(g[h].empty() && jaw[h] == 0, "Empty reset needs open jaws");
      else
        throw std::invalid_argument("Unknown yaw mode");
      yaw[h] = a.target;
    } else if (a.kind == "layer_lock") {
      need(unlocked == a.move && turned, "Cannot lock unexecuted turn");
      moves.push_back(a.move);
      unlocked.clear();
    } else if (a.kind == "checkpoint")
      need((g[0] == "core" || g[1] == "core") && yaw[0] == 0 && yaw[1] == 0 && unlocked.empty(),
           "Invalid checkpoint support or wrist state");
    else
      throw std::invalid_argument("Unknown action: " + a.kind);
  }
  need(unlocked.empty(), "Plan ends with unlocked layer");
  Json hands = Json::array();
  for (int h = 0; h < 2; h++)
    if (!g[h].empty()) hands.push_back(h == 0 ? "A" : "B");
  return {{"moves", moves},
          {"orientation", matrix_json(q)},
          {"grasped", hands},
          {"yaw_rad", {{"A", yaw[0]}, {"B", yaw[1]}}},
          {"jaw_command", {{"A", jaw[0]}, {"B", jaw[1]}}}};
}
Plan compile_moves(const std::vector<std::string>& moves, Mat orientation) {
  checked(orientation);
  Mat q = orientation;
  Plan p;
  auto clear = [&](std::string h) {
    p.push_back({"release", h});
    p.push_back({"jaw", h, 0});
  };
  auto engage = [&](std::string h) {
    p.push_back({"jaw", h, 115});
    p.push_back({"grasp", h, 0, "core"});
  };
  auto reorient = [&](std::string h, int quarters) {
    std::string other = h == "A" ? "B" : "A";
    clear(other);
    p.push_back({"yaw", h, quarters * pi / 2, "whole"});
    engage(other);
    clear(h);
    p.push_back({"yaw", h, 0, "empty"});
    engage(h);
    q = wrist_rotation(h, quarters * pi / 2) * q;
  };
  for (auto& m : moves) {
    auto [f, c] = parse_move(m);
    auto [axis, sign] = face_axis(f);
    Vec n = q * Vec::Unit(axis) * sign;
    if (n.x() < -0.5)
      reorient("B", 2);
    else if (n.z() > .5)
      reorient("B", -1);
    else if (n.z() < -.5)
      reorient("B", 1);
    else if (n.y() > .5) {
      reorient("A", 1);
      reorient("B", -1);
    } else if (n.y() < -.5) {
      reorient("A", 1);
      reorient("B", 1);
    }
    p.push_back({"release", "A"});
    p.push_back({"layer_unlock", "A", 0, "", m});
    p.push_back({"grasp", "A", 0, "face", m});
    p.push_back({"yaw", "A", -c * pi / 2, "face", m});
    p.push_back({"layer_lock", "A", 0, "", m});
    clear("A");
    p.push_back({"yaw", "A", 0, "empty"});
    engage("A");
    p.push_back({"checkpoint", "", 0, "", m});
  }
  replay_plan(p, orientation);
  return p;
}
Plan optimize_plan(const Plan& input, Mat orientation) {
  auto expected = replay_plan(input, orientation);
  Plan result = input;
  for (;;) {
    bool changed = false;
    for (size_t i = 0; i < result.size(); i++) {
      size_t start = i;
      bool close = result[i].kind == "jaw" && result[i].target == 115;
      if (close) start++;
      if (start >= result.size()) continue;
      auto grasp = result[start];
      if (grasp.kind != "grasp" || grasp.mode != "core" || (close && grasp.hand != result[i].hand))
        continue;
      size_t j = start + 1;
      while (j < result.size() && result[j].kind == "checkpoint") j++;
      if (j >= result.size() || result[j].kind != "release" || result[j].hand != grasp.hand)
        continue;
      size_t end = j + 1;
      if (close) {
        if (end >= result.size() || result[end].kind != "jaw" || result[end].hand != grasp.hand ||
            result[end].target != 0)
          continue;
        end++;
      }
      Plan candidate(result.begin(), result.begin() + i);
      candidate.insert(candidate.end(), result.begin() + start + 1, result.begin() + j);
      candidate.insert(candidate.end(), result.begin() + end, result.end());
      try {
        if (replay_plan(candidate, orientation) == expected) {
          result = std::move(candidate);
          changed = true;
          break;
        }
      } catch (const std::invalid_argument&) {
      }
    }
    if (!changed) break;
  }
  std::map<std::string, double> jaw{{"A", 115}, {"B", 115}};
  Plan compact;
  for (auto& a : result) {
    if (a.kind == "jaw") {
      if (jaw[a.hand] == a.target) continue;
      jaw[a.hand] = a.target;
    }
    compact.push_back(a);
  }
  if (replay_plan(compact, orientation) != expected)
    throw std::runtime_error("Optimization changed cube state");
  return compact;
}
}  // namespace rm::cube
