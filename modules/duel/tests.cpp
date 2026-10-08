#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <thread>

#include "duel.hpp"
using namespace rm::duel;
void check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
  check(std::abs(actual - expected) < tolerance, message + ": " + std::to_string(actual));
}
Observation observation(double t, int plate = 0, std::string number = "3") {
  double angle = .9 * t + plate * pi / 2;
  return {Vec3(3 + .15 * t, .2, .25) - Vec3(.255 * std::cos(angle), .255 * std::sin(angle), 0),
          angle + pi, number, .99};
}
void alternate_geometry_test() {
  RotorTracker tracker;
  for (int i = 0; i < 400; ++i) {
    const double t = i * .05;
    const int plate = static_cast<int>(t) % 4;
    const double yaw = .8 * t + plate * pi / 2;
    const double radius = plate % 2 ? .32 : .20;
    tracker.update(
        {Observation{{3 + radius * std::cos(yaw), radius * std::sin(yaw), plate % 2 ? .33 : .25},
                     yaw,
                     "3",
                     .99}},
        t);
  }
  check((tracker.center(19.95) - Vec3(3, 0, .25)).norm() < .04,
        "alternate geometry does not create center motion");
  near((*tracker.x)[8], .20, .025, "first radius estimate");
  near(tracker.r2(), .32, .025, "alternate radius estimate");
  near(tracker.dz(), .08, .015, "alternate height estimate");
}

void label_orientation_test(const std::filesystem::path& root) {
  Duel game(root);
  rm::Renderer renderer(game.model());
  cv::Mat source = cv::imread((root / "assets/armor_labels/3.png").string(), cv::IMREAD_GRAYSCALE);
  cv::Mat reference;
  cv::resize(source, reference, {160, 160});
  reference = reference > 127;
  int camera_id = mj_name2id(game.model(), mjOBJ_CAMERA, "blue_camera");
  for (const std::string team : {"blue", "red"}) {
    for (const std::string side : {"front", "left", "rear", "right"}) {
      int body = mj_name2id(game.model(), mjOBJ_BODY, (team + "_armor_" + side).c_str());
      Mat3 rotation =
          Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(game.data()->xmat + 9 * body);
      Vec3 normal = rotation.col(0);
      Vec3 origin = Eigen::Map<Vec3>(game.data()->xpos + 3 * body) + rotation * Vec3(.0105, 0, 0);
      Vec3 camera_position = origin + normal * .5;
      std::copy(camera_position.data(), camera_position.data() + 3,
                game.data()->cam_xpos + 3 * camera_id);
      Vec3 right = (-normal).cross(Vec3::UnitZ()).normalized();
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_rotation;
      camera_rotation.col(0) = right;
      camera_rotation.col(1) = normal.cross(right);
      camera_rotation.col(2) = normal;
      std::copy(camera_rotation.data(), camera_rotation.data() + 9,
                game.data()->cam_xmat + 9 * camera_id);
      cv::Mat rgb = renderer.render_camera(game.data(), "blue_camera");
      cv::Mat crop;
      cv::cvtColor(rgb(cv::Rect(320, 220, 160, 160)), crop, cv::COLOR_RGB2GRAY);
      crop = crop > 127;
      cv::Mat vertical, horizontal;
      cv::flip(reference, vertical, 0);
      cv::flip(reference, horizontal, 1);
      auto similarity = [&](const cv::Mat& expected) {
        return cv::countNonZero(crop == expected) / double(crop.total());
      };
      const double correct = similarity(reference);
      check(correct > .9 && correct > similarity(vertical) + .015 &&
                correct > similarity(horizontal) + .1,
            "upright outward label " + team + " " + side);
    }
  }
}

void watchdog_and_blank_test(const std::filesystem::path& root) {
  Duel game(root);
  game.command("drive", {{"vx", .5}});
  game.step();
  int motor = mj_name2id(game.model(), mjOBJ_ACTUATOR, "blue_drive_fl");
  check(std::abs(game.data()->ctrl[motor]) > 1, "fresh drive reaches wheel");
  std::this_thread::sleep_for(std::chrono::milliseconds(360));
  game.step();
  near(game.data()->ctrl[motor], 0, 1e-12, "wall-clock drive watchdog stops");
  game.command("pause", {{"paused", true}});
  game.command("drive", {{"vx", .5}});
  game.command("pause", {{"paused", false}});
  game.step();
  near(game.data()->ctrl[motor], 0, 1e-12, "paused drive cannot replay");
  game.command("drive", {{"vx", .5}, {"_drive_ttl_ms", 0}});
  game.step();
  near(game.data()->ctrl[motor], 0, 1e-12, "expired host drive TTL cannot replay");

  // Mutate textures before the first renderer uploads them to the GPU.
  Duel blank(root);
  std::fill(blank.model()->tex_data, blank.model()->tex_data + blank.model()->ntexdata, 0);
  for (int i = 0; i < 10; ++i) blank.step();
  check(!blank.robots[0].tracker.x, "unreadable physical labels cannot initialize tracker");
}
int main(int argc, char** argv) {
  try {
    check(argc >= 2, "root argument required");
    std::filesystem::path root = argv[1];
    bool render = argc > 2;
    if (!render) {
      alternate_geometry_test();
      Heat h;
      for (int i = 0; i < 4; ++i) check(h.fire(0, false), "four cold shots");
      check(h.value == 40 && !h.locked, "heat exact limit");
      check(!h.fire(0, true), "heat protection");
      check(h.fire(0, false) && h.locked, "overheat lock");
      check(!h.fire(0, false), "locked rejects");
      h.cool(.099);
      near(h.value, 50, 1e-8, "discrete heat");
      h.cool(.1);
      near(h.value, 48.8, 1e-8, "100ms heat");
      h.cool(4.2);
      check(h.value == 0 && !h.locked, "cool recovery");
      near(*segment_box({-1, 0, 0}, {1, 0, 0}, {.01, .07, .06}), .495, 1e-8, "box sweep");
      check(!segment_box({-1, .1, 0}, {1, .1, 0}, {.01, .07, .06}), "box miss");
      near(*segment_cylinder({-1, 0, 0}, {1, 0, 0}, .1, .2), .45, 1e-8, "cylinder sweep");
      check(!segment_cylinder({-1, .11, 0}, {1, .11, 0}, .1, .2), "cylinder miss");
      near(*segment_ellipsoid({-1, 0, 0}, {1, 0, 0}, {.1, .2, .3}), .45, 1e-8, "ellipsoid sweep");
      RotorTracker tracker;
      for (int i = 0; i < 200; ++i) {
        double t = i * .05;
        tracker.update({observation(t, int(t / 1.3) % 4)}, t);
      }
      check(tracker.ready(9.95), "tracker ready");
      check((tracker.center(9.95) - Vec3(4.4925, .2, .25)).norm() < .06, "rotor center");
      near((*tracker.x)[7], .9, .1, "rotor rate");
      check(tracker.switches >= 3, "plate switches");
      Eigen::SelfAdjointEigenSolver<RotorTracker::Covariance> eig(tracker.P);
      check(eig.eigenvalues().minCoeff() > -1e-9, "positive covariance");
      double seen = tracker.last_seen;
      tracker.update({observation(10, 0, "4")}, 10);
      check(tracker.last_seen == seen, "wrong ID rejected");
      tracker.update({}, 10.3);
      check(!tracker.ready(10.3), "dropout fire gating");
      tracker.update({}, 11);
      check(tracker.state == "LOST", "lost state");
      tracker.update({observation(20)}, 20);
      check(tracker.state == "DETECTING" && !tracker.ready(20), "long-gap reacquisition");
      NumberClassifier classifier(root);
      cv::Mat label =
          cv::imread((root / "assets/armor_labels/3.png").string(), cv::IMREAD_GRAYSCALE);
      auto classified = classifier.classify_roi(label);
      check(classified.number == "3" && classified.confidence > .8, "original ONNX label");
      check(classifier.classify_roi(cv::Mat::zeros(28, 20, CV_8U)).number.empty(),
            "blank rejected");
      Detector detector(root, true);
      cv::Mat image = cv::Mat::zeros(600, 800, CV_8UC3);
      cv::rectangle(image, {376, 290}, {379, 309}, {255, 20, 20}, -1);
      cv::rectangle(image, {421, 290}, {424, 309}, {255, 20, 20}, -1);
      auto ds = detector.detect(image);
      check(!ds.empty(), "RGB lightbar detection");
      near(cv_vector(ds[0].pose.tvec).z(), 2, .2, "calibrated IPPE range");
      check(Detector(root, false).detect(image).empty(), "team color filter");
      check(detector.detect(cv::Mat::zeros(600, 800, CV_8UC3)).empty(), "blank detection");
      Duel game(root);
      check(game.settings.at("target_speed") == 2 && game.settings.at("target_spin") == 2 &&
                game.settings.at("invulnerable") == true,
            "control defaults");
      for (const auto& [command, args] : std::vector<std::pair<std::string, rm::Json>>{
               {"settings", {{"target_speed", 4.1}}},
               {"settings", {{"target_spin", -.1}}},
               {"settings", {{"auto_aim", 1}}},
               {"settings", {{"profile", "invalid"}}},
               {"drive", {{"vx", true}}},
               {"drive", {{"wz", 2}}},
               {"drive", {{"_drive_ttl_ms", 351}}},
               {"aim", {{"yaw", 0}}},
               {"pause", {{"paused", 1}}},
               {"settings", {{"speed", std::numeric_limits<double>::infinity()}}},
               {"settings", rm::Json::array()}}) {
        bool rejected = false;
        try {
          game.command(command, args);
        } catch (const std::exception&) {
          rejected = true;
        }
        check(rejected, "invalid command rejected " + command);
      }
      game.command("settings", {{"target_speed", 4}, {"target_spin", 4}});
      game.command("pause", {{"paused", true}});
      game.command("pause", {{"paused", true}});
      check(game.state().at("paused"), "idempotent pause");
      check(!game.command("fire", rm::Json::object()).at("fired").get<bool>(), "pause blocks fire");
      for (bool invulnerable : {true, false}) {
        game.command("settings", {{"invulnerable", invulnerable}});
        int site = mj_name2id(game.model(), mjOBJ_SITE, "red_armor_face_front");
        Vec3 face = Eigen::Map<Vec3>(game.data()->site_xpos + 3 * site);
        Mat3 rotation = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            game.data()->site_xmat + 9 * site);
        Vec3 normal = rotation.col(0);
        game.projectiles = {{0, face + normal * .1, -normal * 23, 0}};
        game.robots[1].last_damage.clear();
        std::vector<double> p(game.data()->geom_xpos,
                              game.data()->geom_xpos + game.model()->ngeom * 3),
            r(game.data()->geom_xmat, game.data()->geom_xmat + game.model()->ngeom * 9);
        for (int j = 0; j < 8; ++j) game.advance_projectiles(.001, p, r);
        check(game.robots[1].hp == (invulnerable ? 200 : 180), "physical hit invulnerability");
      }
      check(game.robots[0].hits == 2, "physical hits counted");
    } else {
      label_orientation_test(root);
      watchdog_and_blank_test(root);
      Duel game(root);
      game.command("settings", {{"auto_fire", true}, {"invulnerable", false}});
      double first = -1;
      for (int i = 0; i < 70; ++i) {
        game.step();
        if (first < 0 && game.robots[0].shots > 0) first = game.time();
      }
      check(first >= 0 && first < .7, "prompt 100Hz first shot");
      check(game.robots[0].hits >= 1 && game.robots[1].hp < 200, "RGB closed-loop physical hit");
      check(game.robots[0].detection && game.robots[0].detection->classification.number == "3",
            "rendered ONNX label");
      for (auto view : {"scene", "camera", "raw", "number"}) {
        auto frame = game.command("frame", {{"view", view}});
        check(frame.at("mime") == "image/jpeg" && frame.at("data").get<std::string>().size() > 100,
              "native JPEG frame");
      }
      game.command("settings", {{"vision", false}});
      int shots = game.robots[0].shots;
      for (int i = 0; i < 12; ++i) game.step();
      check(!game.robots[0].tracker.ready(game.time()), "RGB loss expires tracker");
      check(game.robots[0].shots <= shots + 1, "no blind autofire");
      game.reset();
      check(game.robots[0].shots == 0 && !game.robots[0].tracker.x && game.projectiles.empty(),
            "reset clears estimates/projectiles");
      game.command("settings", {{"vision", true},
                                {"auto_fire", false},
                                {"enemy_motion", "strafe"},
                                {"target_speed", .45}});
      double initial = game.state()["robots"][1]["position"][1];
      for (int i = 0; i < 20; ++i) game.step();
      check(game.state()["robots"][1]["position"][1].get<double>() < initial - .25,
            "contact-driven lateral wheels");
      game.reset();
      game.command("settings", {{"enemy_motion", "spin"}, {"target_spin", .8}});
      int tracked = 0;
      std::vector<double> rates;
      for (int i = 0; i < 110; ++i) {
        game.step();
        auto& t = game.robots[0].tracker;
        if (t.ready(game.time())) {
          ++tracked;
          rates.push_back((*t.x)[7]);
        }
      }
      check(tracked > 66, "rotating RGB tracking coverage");
      check(game.robots[0].tracker.switches >= 1, "RGB plate association switches");
      check(rates.size() >= 15, "spin estimates");
      std::vector<double> last(rates.end() - 15, rates.end());
      std::sort(last.begin(), last.end());
      near(last[7], .8, .35, "RGB spin rate");
      for (int i = 0; i < mjNWARNING; ++i)
        check(game.data()->warning[i].number == 0, "MuJoCo warnings");
      std::cout << "first shot " << first << "s; rotation tracked " << tracked
                << "/110; median omega " << last[7] << '\n';
    }
    std::cout << (render ? "duel render" : "duel behavior") << " checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "duel test: " << e.what() << '\n';
    return 1;
  }
}
