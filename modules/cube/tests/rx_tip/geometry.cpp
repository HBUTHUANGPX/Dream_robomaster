#include "common.hpp"
using namespace rx_tip;
int main(int argc, char** argv) {
  try {
    if (argc != 4 && argc != 5)
      throw std::invalid_argument("用法: geometry BUNDLE_MJCF_DIR OUTPUT_DIR A|B [PAD_MM=16]");
    const int active = parse_hand(argv[3]);
    const double padmm = argc == 5 ? finite_number(argv[4]) : default_pad_mm;
    if (padmm <= 0) throw std::invalid_argument("PAD_MM 必须大于零");
    const auto src = std::filesystem::canonical(directory(argv[1], false));
    if (std::string(argv[2]).empty()) throw std::invalid_argument("输出目录不能为空");
    const auto requested = std::filesystem::weakly_canonical(std::filesystem::absolute(argv[2]));
    const auto relative = requested.lexically_relative(src);
    if (relative.empty() || *relative.begin() != "..")
      throw std::invalid_argument("输出目录不能位于只读源 bundle 内");
    const auto out = directory(requested.string(), true);
    const auto scene = active ? "dual55B.xml" : "dual55.xml";
    for (const auto& path :
         {out / scene, out / (active ? "dual55B-scan.json" : "dual55-scan.json")})
      if (std::filesystem::is_symlink(path)) throw std::invalid_argument("输出文件不能是符号链接");
    std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)> owner(
        xmlReadFile((src / "free_sweep.xml").c_str(), nullptr, XML_PARSE_NONET), xmlFreeDoc);
    if (!owner) throw std::runtime_error("XML 加载失败: " + (src / "free_sweep.xml").string());
    auto doc = owner.get();
    auto root = xmlDocGetRootElement(doc), world = child(root, "worldbody");
    set(child(root, "compiler"), "inertiafromgeom", "auto");
    walk(child(root, "asset")->children, [&](N n) {
      if (xmlStrEqual(n->name, BAD_CAST "mesh") && !get(n, "file").empty())
        set(n, "file", (src / get(n, "file")).string());
    });
    std::vector<N> remove;
    walk(world->children, [&](N n) {
      if (get(n, "name") == "test_pull_slide") remove.push_back(n);
    });
    walk(child(root, "actuator")->children, [&](N n) {
      if (get(n, "name") == "test_pull") remove.push_back(n);
    });
    for (auto n : remove) {
      xmlUnlinkNode(n);
      xmlFreeNode(n);
    }
    double side = .055, pitch = side / 3, half = (side - .0007 * 55 / 60) / 2,
           cubelet = half - pitch, motor = motor_for_gap(2 * half),
           cx = -.00189107 + (rz(angle(motor)) * (B - A) - (B - A)).x();
    M ra = V(-1, 1, -1).asDiagonal(), rb;
    rb.col(0) = V(0, 1, 0);
    rb.col(1) = V(1, 0, 0);
    rb.col(2) = V(0, 0, -1);
    std::vector<N> bodies;
    for (auto n = world->children; n; n = n->next)
      if (n->type == XML_ELEMENT_NODE) bodies.push_back(n);
    for (auto n : bodies) {
      const auto name = get(n, "name");
      if (!xmlStrEqual(n->name, BAD_CAST "body") ||
          (name != "l_gripper_link" && name != "r_gripper_link"))
        throw std::runtime_error("源 worldbody 只允许两个夹爪根 body");
      bool left = name == "l_gripper_link";
      xmlUnlinkNode(n);
      auto wrist = add(world, "body", {{"name", left ? "A_wrist" : "B_wrist"}, {"pos", "0 0 .22"}});
      add(wrist, "joint",
          {{"name", left ? "A_yaw" : "B_yaw"},
           {"axis", left ? "1 0 0" : "0 -1 0"},
           {"limited", "true"},
           {"range", "-3.2 3.2"},
           {"damping", ".2"},
           {"armature", ".01"}});
      set(n, "pos",
          vec(left ? V(pitch + cx + default_shift, 0, 0) : V(0, -pitch - cx - default_shift, 0)));
      set(n, "quat", quat(left ? ra : rb));
      xmlAddChild(wrist, n);
      add(child(root, "actuator"), "position",
          {{"name", left ? "A_yaw_drive" : "B_yaw_drive"},
           {"joint", left ? "A_yaw" : "B_yaw"},
           {"kp", "20"},
           {"kv", "1"},
           {"forcerange", "-15 15"}});
    }
    std::vector<N> fingers;
    walk(world->children, [&](N n) {
      auto name = get(n, "name");
      if (xmlStrEqual(n->name, BAD_CAST "body") &&
          (name.ends_with("_finger1_link") || name.ends_with("_finger2_link")))
        fingers.push_back(n);
    });
    for (auto finger : fingers) {
      auto name = get(finger, "name");
      double sign = name.ends_with("_finger1_link") ? -1 : 1;
      auto tip = add(finger, "body", {{"name", name + "_extension"}});
      add(tip, "geom",
          {{"name", name + "_neck"},
           {"type", "box"},
           {"pos", vec(V(.027, sign * .01789246, 0))},
           {"size", ".009 .003 .004"},
           {"mass", ".001"},
           {"contype", "1"},
           {"conaffinity", "15"},
           {"friction", "0 0 0"},
           {"condim", "1"},
           {"rgba", ".2 .7 .8 1"},
           {"solref", ".004 1"},
           {"solimp", ".99 .999 .001 .5 2"},
           {"margin", ".0005"}});
      add(tip, "geom",
          {{"name", name + "_pad"},
           {"type", "box"},
           {"pos", vec(V(.040, sign * .01689246, 0))},
           {"size", vec(V(padmm * .0005, .003, padmm * .0005))},
           {"mass", ".001"},
           {"contype", "1"},
           {"conaffinity", "15"},
           {"friction", "0 0 0"},
           {"condim", "1"},
           {"rgba", ".1 .15 .2 1"},
           {"solref", ".004 1"},
           {"solimp", ".99 .999 .001 .5 2"},
           {"margin", ".0005"}});
    }
    auto cube = add(world, "body", {{"name", "probe_cube"}, {"pos", "0 0 .22"}});
    add(cube, "freejoint", {{"name", "probe_free"}});
    auto layer = add(cube, "body", {{"name", "probe_layer"}});
    add(layer, "joint",
        {{"name", "probe_layer_hinge"},
         {"axis", active ? "0 -1 0" : "1 0 0"},
         {"damping", ".001"}});
    int idx = 0;
    J slots = J::array();
    for (int x = -1; x <= 1; x++)
      for (int y = -1; y <= 1; y++)
        for (int z = -1; z <= 1; z++) {
          if (x == 0 && y == 0 && z == 0) continue;
          add((active ? y == -1 : x == 1) ? layer : cube, "geom",
              {{"name", "piece_" + std::to_string(idx++)},
               {"type", "box"},
               {"pos", vec(pitch * V(x, y, z))},
               {"size", vec(V::Constant(cubelet))},
               {"mass", ".006"},
               {"contype", "16"},
               {"conaffinity", "15"},
               {"friction", ".8 .002 .0001"},
               {"rgba", (active ? y == -1 : x == 1) ? ".9 .15 .1 1" : ".15 .6 .9 1"}});
          slots.push_back({x, y, z});
        }
    add(world, "light", {{"pos", ".1 -.3 .6"}, {"dir", "0 0 -1"}});
    add(world, "light", {{"pos", "-.3 .1 .4"}, {"dir", "1 0 -1"}});
    if (xmlSaveFormatFileEnc((out / scene).c_str(), doc, "UTF-8", 1) < 0)
      throw std::runtime_error("XML 输出失败");
    Model model(out / scene);
    auto* m = model.model.get();
    auto* d = model.data.get();
    const auto roots = hand_roots(m);
    std::vector<int> hand, piece;
    classify(m, roots, hand, piece);
    int wrist[2] = {id(m, mjOBJ_JOINT, "A_yaw"), id(m, mjOBJ_JOINT, "B_yaw")},
        layerj = id(m, mjOBJ_JOINT, "probe_layer_hinge");
    auto snapshot = [&]() {
      mj_forward(m, d);
      int cross = 0, self = 0;
      double depth = 0, wrongdepth = 0;
      std::string wrongpair;
      std::set<int> touch[2];
      int open_cube[2] = {};
      for (int c = 0; c < d->ncon; c++) {
        auto& k = d->contact[c];
        if (k.dist >= -1e-6) continue;
        int a = k.geom[0], b = k.geom[1];
        auto ext = [&](int g) {
          const char* n = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
          return n && std::string(n).ends_with("_extension");
        };
        if (hand[a] >= 0 && hand[a] == hand[b] && (ext(a) || ext(b))) self++;
        if (hand[a] >= 0 && hand[b] >= 0 && hand[a] != hand[b]) {
          cross++;
          depth = std::max(depth, -k.dist);
        }
        for (int h = 0; h < 2; h++) {
          int pg = hand[a] == h && piece[b] >= 0 ? b : hand[b] == h && piece[a] >= 0 ? a : -1;
          if (pg >= 0 &&
              (((h == active) &&
                !(active ? slots[piece[pg]][1] == -1 : slots[piece[pg]][0] == 1)) ||
               (h != active && (active ? slots[piece[pg]][1] == -1 : slots[piece[pg]][0] == 1))) &&
              -k.dist > wrongdepth) {
            wrongdepth = -k.dist;
            wrongpair = geom_name(m, a) + " / " + geom_name(m, b);
          }
          if (hand[a] == h && piece[b] >= 0) {
            touch[h].insert(piece[b]);
            open_cube[h]++;
          }
          if (hand[b] == h && piece[a] >= 0) {
            touch[h].insert(piece[a]);
            open_cube[h]++;
          }
        }
      }
      int wrong[2] = {};
      for (int h = 0; h < 2; h++)
        for (int p : touch[h]) {
          bool target = active ? slots[p][1] == -1 : slots[p][0] == 1;
          if ((h == active && !target) || (h != active && target)) wrong[h]++;
        }
      J t = J::array();
      for (int h = 0; h < 2; h++) {
        J s = J::array();
        for (int p : touch[h]) s.push_back(slots[p]);
        t.push_back(s);
      }
      return J{
          {"self_contacts", self},
          {"cross_contacts", cross},
          {"cross_depth_mm", depth * 1000},
          {"cube_contacts", {{"active", open_cube[active]}, {"support", open_cube[1 - active]}}},
          {"touch_slots", {{"active", t[active]}, {"support", t[1 - active]}}},
          {"wrong_layer_contacts", {{"active", wrong[active]}, {"support", wrong[1 - active]}}},
          {"wrong_depth_mm", wrongdepth * 1000},
          {"wrong_pair", wrongpair}};
    };
    J result = {{"pad_mm", padmm},
                {"mode", "single_layer_probe"},
                {"full_solve", false},
                {"active_hand", hand_name(active)},
                {"support_hand", hand_name(1 - active)},
                {"cube_side_mm", 55},
                {"actual_collision_side_mm", half * 2000},
                {"grip_motor_rad", motor},
                {"reference_center_x_mm", cx * 1000},
                {"cases", J::array()}};
    for (int roll : {90})
      for (double shift : {default_shift}) {
        if (std::abs(m->body_pos[3 * roots[0]] - (pitch + cx + shift)) > 1e-10 ||
            std::abs(m->body_pos[3 * roots[1] + 1] - (-pitch - cx - shift)) > 1e-10)
          throw std::runtime_error("生成 XML 的安装位与扫描位不一致");
        for (int direction : {-1, 1})
          for (auto kind :
               {"open_return", "open_closed_return", "both_closed", "layer_turn", "whole_turn"}) {
            mj_resetData(m, d);
            bool open =
                std::string(kind) == "open_return" || std::string(kind) == "open_closed_return";
            for (int h = 0; h < 2; h++) {
              bool empty = std::string(kind) == "open_return" ||
                           (std::string(kind) == "open_closed_return" && h == active) ||
                           (std::string(kind) == "whole_turn" && h != active);
              conf(m, d, h == 0 ? "l" : "r", empty ? -2.25 : motor_for_gap(2 * half - .0002));
            }
            d->qpos[m->jnt_qposadr[wrist[1 - active]]] = roll * pi / 180;
            J row = {{"support_wrist_deg", roll},
                     {"direction", direction},
                     {"shift_mm", shift * 1000},
                     {"kind", kind},
                     {"initial", snapshot()}};
            int selfmax = 0, crossmax = 0, openmax = 0, wrongmax = 0, supportmax = 0;
            double maxdepth = 0, wrongdepth = 0;
            J first, worst;
            int count = std::string(kind) == "both_closed" ? 0 : 180;
            for (int deg = 0; deg <= count; deg++) {
              d->qpos[m->jnt_qposadr[wrist[active]]] = direction * deg * pi / 180;
              if (std::string(kind) == "whole_turn") {
                int fj = id(m, mjOBJ_JOINT, "probe_free"), adr = m->jnt_qposadr[fj] + 3;
                d->qpos[adr] = cos(direction * deg * pi / 360);
                d->qpos[adr + 1 + active] = (active ? -1 : 1) * sin(direction * deg * pi / 360);
              } else if (!open)
                d->qpos[m->jnt_qposadr[layerj]] = direction * deg * pi / 180;
              J s = snapshot();
              int cross = s["cross_contacts"], oc = s["cube_contacts"]["active"];
              int wrong = s["wrong_layer_contacts"]["active"].get<int>() +
                          s["wrong_layer_contacts"]["support"].get<int>();
              wrongmax = std::max(wrongmax, wrong);
              if (s["wrong_depth_mm"].get<double>() > wrongdepth) {
                wrongdepth = s["wrong_depth_mm"];
                worst = {{"deg", direction * deg}, {"pair", s["wrong_pair"]}};
              }
              selfmax = std::max(selfmax, s["self_contacts"].get<int>());
              supportmax = std::max(supportmax, s["cube_contacts"]["support"].get<int>());
              crossmax = std::max(crossmax, cross);
              openmax = std::max(openmax, oc);
              maxdepth = std::max(maxdepth, s["cross_depth_mm"].get<double>());
              if (first.is_null() &&
                  (s["self_contacts"].get<int>() > 0 || cross > 0 || (open && oc > 0) ||
                   (std::string(kind) == "layer_turn" && wrong > 0) ||
                   (std::string(kind) == "whole_turn" &&
                    s["cube_contacts"]["support"].get<int>() > 0)))
                first = direction * deg;
            }
            row["sweep_max_self_contacts"] = selfmax;
            row["sweep_wrong_depth_mm"] = wrongdepth;
            row["worst_wrong"] = worst;
            row["sweep_max_wrong_layer_contacts"] = wrongmax;
            row["sweep_max_support_cube_contacts"] = supportmax;
            row["sweep_max_cross_contacts"] = crossmax;
            row["sweep_max_cross_depth_mm"] = maxdepth;
            row["sweep_max_active_cube_contacts"] = openmax;
            row["first_forbidden_deg"] = first;
            result["cases"].push_back(row);
          }
      }
    J closure = J::array();
    for (int i = 0; i <= 100; i++) {
      mj_resetData(m, d);
      d->qpos[m->jnt_qposadr[id(m, mjOBJ_JOINT, "probe_free")] + 2] = -.5;
      conf(m, d, "l", -2.25 * (1 - i / 100.));
      conf(m, d, "r", -2.25 * (1 - i / 100.));
      d->qpos[m->jnt_qposadr[wrist[1 - active]]] = pi / 2;
      auto snap = snapshot();
      closure.push_back({{"closure", i / 100.},
                         {"self_contacts", snap["self_contacts"]},
                         {"cross_contacts", snap["cross_contacts"]}});
    }
    result["closure_sweep"] = closure;
    write_json(out / (active ? "dual55B-scan.json" : "dual55-scan.json"), result);
    std::cout << result.dump(2) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "失败：" << error.what() << '\n';
    return 1;
  }
}
