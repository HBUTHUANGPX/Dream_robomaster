#include "duel.hpp"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "model.hpp"
namespace rm::duel {
namespace {
const std::array<std::string, 2> teams{"blue", "red"};
Vec3 vector_at(const double* p, int index) { return Eigen::Map<const Vec3>(p + 3 * index); }
Mat3 rotation_at(const double* p, int index) {
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(p + 9 * index);
}
Json array(const Vec3& v) { return Json::array({v.x(), v.y(), v.z()}); }
double rounded(double value, int decimals) {
  const double scale = std::pow(10., decimals);
  return std::round(value * scale) / scale;
}
double number(const Json& a, const std::string& key, double lo, double hi,
              std::optional<double> fallback = {}) {
  if (!a.contains(key)) {
    if (fallback) return *fallback;
    throw std::invalid_argument("missing " + key);
  }
  const auto& v = a.at(key);
  if (!v.is_number()) throw std::invalid_argument(key + " must be numeric");
  double n = v.get<double>();
  if (!std::isfinite(n) || n < lo || n > hi) throw std::invalid_argument(key + " out of range");
  return n;
}
}  // namespace
Duel::Duel(const std::filesystem::path& root)
    : root_(root),
      model_(build_model(root), mj_deleteModel),
      data_(mj_makeData(model()), mj_deleteData) {
  if (!data()) throw std::runtime_error("MuJoCo data allocation failed");
  settings = {{"auto_aim", true},  {"auto_fire", false},   {"enemy_fire", false},
              {"vision", true},    {"protect_heat", true}, {"enemy_motion", "static"},
              {"speed", 23.},      {"fire_rate", 5.},      {"target_speed", 2.},
              {"target_spin", 2.}, {"invulnerable", true}, {"profile", "cooling"},
              {"view", "overview"}};
  auto id = [&](mjtObj type, const std::string& name) {
    int result = mj_name2id(model(), type, name.c_str());
    if (result < 0) throw std::runtime_error("missing model object " + name);
    return result;
  };
  for (int i = 0; i < 2; ++i) {
    auto t = teams[i];
    chassis_[i] = id(mjOBJ_BODY, t + "_chassis");
    cameras_[i] = id(mjOBJ_CAMERA, t + "_camera");
    muzzles_[i] = id(mjOBJ_SITE, t + "_muzzle");
    int k = 0;
    for (auto n : {"fl", "fr", "rl", "rr"}) drives_[i][k++] = id(mjOBJ_ACTUATOR, t + "_drive_" + n);
    k = 0;
    for (auto a : {"yaw", "pitch"}) {
      servos_[i][k] = id(mjOBJ_ACTUATOR, t + "_" + a + "_servo");
      joints_[i][k++] = model()->jnt_qposadr[id(mjOBJ_JOINT, t + "_" + a + "_joint")];
    }
  }
  for (int gid = 0; gid < model()->ngeom; ++gid) {
    const char* p = mj_id2name(model(), mjOBJ_GEOM, gid);
    std::string name = p ? p : "";
    int kind = model()->geom_type[gid];
    if (kind != mjGEOM_BOX && kind != mjGEOM_ELLIPSOID && kind != mjGEOM_CYLINDER) continue;
    if (name.find("light_") != std::string::npos) continue;
    int body = model()->body_rootid[model()->geom_bodyid[gid]];
    for (int i = 0; i < 2; ++i)
      if (body == chassis_[i])
        colliders_[i].push_back({gid, kind, name.find("armor_collision") != std::string::npos});
  }
  reset();
}
double Duel::time() const { return data()->time - start_; }
rm::Renderer& Duel::renderer() {
  if (!renderer_) renderer_ = std::make_unique<rm::Renderer>(model(), 800, 600);
  return *renderer_;
}
void Duel::event(const std::string& text) {
  events_.push_back({{"time", rounded(time(), 2)}, {"text", text}});
  if (events_.size() > 30) events_.erase(events_.begin());
}
void Duel::reset() {
  mj_resetData(model(), data());
  for (int i = 0; i < 2; ++i) data()->qpos[joints_[i][1]] = -.05;
  mj_forward(model(), data());
  for (int j = 0; j < 300; ++j) {
    for (int i = 0; i < 2; ++i) {
      data()->ctrl[servos_[i][0]] = 0;
      data()->ctrl[servos_[i][1]] = -.05;
    }
    mj_step(model(), data());
  }
  start_ = data()->time;
  for (int i = 0; i < 2; ++i) {
    robots[i] = Robot{};
    robots[i].detector = std::make_unique<Detector>(root_, i == 0);
    robots[i].heat = Heat(settings.at("profile").get<std::string>());
  }
  projectiles.clear();
  trails_.clear();
  events_ = Json::array();
  paused_ = false;
  winner_.clear();
  drive_.setZero();
  drive_deadline_ = {};
  raw_ = {};
  annotated_ = {};
  event("新回合：蓝方对红方 · 200 HP");
}
Vec3 Duel::muzzle_velocity(int i) const {
  double v[6];
  mj_objectVelocity(model(), data(), mjOBJ_SITE, muzzles_[i], v, 0);
  return Eigen::Map<Vec3>(v + 3);
}
void Duel::overlay(mjvScene& scene) {
  auto add = [&](int type, const Vec3& size, const Vec3& position, const float* color) {
    if (scene.ngeom >= scene.maxgeom) return static_cast<mjvGeom*>(nullptr);
    auto* g = &scene.geoms[scene.ngeom++];
    double identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    mjv_initGeom(g, type, size.data(), position.data(), identity, color);
    return g;
  };
  for (auto& p : projectiles) {
    float color[] = {1, .8f, .15f, 1};
    add(mjGEOM_SPHERE, Vec3::Constant(projectile_radius), p.position, color);
  }
  for (size_t i = 0; i < trails_.size(); i += 5) {
    auto& t = trails_[i];
    float color[] = {t.owner == 0 ? .3f : 1.f, t.owner == 0 ? .7f : .5f, t.owner == 0 ? 1.f : .2f,
                     .65f};
    auto* g = add(mjGEOM_CAPSULE, Vec3::Zero(), Vec3::Zero(), color);
    if (g) mjv_connector(g, mjGEOM_CAPSULE, .003, t.a.data(), t.b.data());
  }
}
cv::Mat Duel::render_camera(int i) {
  mjvCamera camera;
  mjv_defaultCamera(&camera);
  camera.type = mjCAMERA_FIXED;
  camera.fixedcamid = cameras_[i];
  auto& r = renderer();
  r.update(data(), camera);
  overlay(r.scene());
  return r.read();
}
void Duel::perceive(int i) {
  auto& r = robots[i];
  raw_[i] = render_camera(i);
  std::vector<Detection> candidates;
  if (settings.at("vision").get<bool>() && !r.heat.locked) candidates = r.detector->detect(raw_[i]);
  Vec3 cam = vector_at(data()->cam_xpos, cameras_[i]);
  Mat3 R = rotation_at(data()->cam_xmat, cameras_[i]) * Vec3(1, -1, -1).asDiagonal();
  std::vector<Observation> observations;
  for (size_t j = 0; j < candidates.size(); ++j) {
    auto& d = candidates[j];
    if (d.classification.number.empty()) continue;
    double best = 1e100;
    bool selected = false;
    Observation chosen;
    for (auto& pose : d.poses) {
      cv::Mat cvR;
      cv::Rodrigues(pose.rvec, cvR);
      Vec3 normal = R * (-Vec3(cvR.at<double>(0, 2), cvR.at<double>(1, 2), cvR.at<double>(2, 2)));
      Vec3 tv = cv_vector(pose.tvec), position = cam + R * tv;
      Vec3 variance(std::pow(std::max(.003, tv.z() * .0015), 2),
                    std::pow(std::max(.003, tv.z() * .0015), 2),
                    std::pow(std::max(.015, .008 * tv.z() * tv.z()), 2));
      Observation obs{position,
                      std::atan2(normal.y(), normal.x()),
                      d.classification.number,
                      d.classification.confidence,
                      R * variance.asDiagonal() * R.transpose(),
                      .25,
                      static_cast<int>(j)};
      double cost = pose.error + std::pow((normal.z() - std::sin(pi / 12)) / .2, 2);
      if (r.tracker.x && r.tracker.state != "LOST")
        cost += r.tracker.association(obs, time()).first * .1;
      // A conflicting ID has infinite association cost. Still pass its real
      // measurement to the tracker's ID gate, without an uninitialized pose.
      if (!selected || cost < best) {
        selected = true;
        best = cost;
        chosen = obs;
        d.pose = pose;
      }
    }
    chosen.yaw = r.detector->refine_yaw(d, chosen.position, cam, R);
    observations.push_back(chosen);
  }
  r.detection.reset();
  auto accepted = r.tracker.update(observations, time());
  if (accepted) r.detection = candidates.at(accepted->detection);
  plan_aim(i);
  std::optional<Vec3> prediction;
  if (r.aim) prediction = R.transpose() * (*r.aim - cam);
  annotated_[i] = r.detector->annotate(raw_[i], r.detection, prediction);
  if (r.tracker.ready(time())) {
    auto marker = [&](const Vec3& world, cv::Scalar color, int type, int size) {
      Vec3 p = R.transpose() * (world - cam);
      if (p.z() <= .1) return;
      auto xy = r.detector->K * cv::Vec3d(p.x(), p.y(), p.z());
      double x = xy[0] / xy[2], y = xy[1] / xy[2];
      if (x >= 0 && x < 800 && y >= 0 && y < 600)
        cv::drawMarker(annotated_[i], {cvRound(x), cvRound(y)}, color, type, size, 1);
    };
    auto points = r.tracker.armor_positions(time() + r.flight);
    for (int j = 0; j < 4; ++j)
      marker(points[j], j == r.aim_plate ? cv::Scalar(255, 190, 70) : cv::Scalar(120, 170, 255),
             cv::MARKER_SQUARE, 9);
    marker(r.tracker.center(time()), {100, 220, 255}, cv::MARKER_CROSS, 16);
  }
}
void Duel::plan_aim(int i) {
  auto& r = robots[i];
  r.aim.reset();
  if (!r.tracker.ready(time())) return;
  Vec3 muzzle = vector_at(data()->site_xpos, muzzles_[i]), velocity = muzzle_velocity(i);
  double best = -1, speed = settings.at("speed");
  for (int idx = 0; idx < 4; ++idx) {
    double flight = (r.tracker.armor_positions(time())[idx] - muzzle).norm() / speed;
    Vec3 target, displacement;
    for (int j = 0; j < 8; ++j) {
      target = r.tracker.armor_positions(time() + flight)[idx];
      displacement = target - muzzle - velocity * flight + Vec3(0, 0, 4.905) * flight * flight;
      flight = displacement.norm() / speed;
    }
    double yaw = r.tracker.armor_yaws(time() + flight)[idx],
           facing = Vec3(std::cos(yaw), std::sin(yaw), 0).dot((muzzle - target).normalized()),
           score = facing + (idx == r.aim_plate ? .08 : 0);
    if (facing > .35 && flight < 2 && score > best) {
      best = score;
      r.aim_plate = idx;
      r.flight = flight;
      r.aim_facing = facing;
      r.aim = muzzle + displacement;
    }
  }
}
void Duel::control(int i) {
  auto& r = robots[i];
  bool automatic = i == 0 ? settings.at("auto_aim").get<bool>() : true;
  double t = time();
  if (automatic && r.aim) {
    Vec3 local = rotation_at(data()->xmat, chassis_[i]).transpose() *
                 (*r.aim - vector_at(data()->site_xpos, muzzles_[i]));
    Eigen::Vector2d desired(std::atan2(local.y(), local.x()),
                            std::atan2(local.z(), local.head<2>().norm())),
        velocity = Eigen::Vector2d::Zero();
    if (r.last_aim && t > r.last_aim_time) {
      Eigen::Vector2d delta = desired - *r.last_aim;
      delta[0] = wrap(delta[0]);
      if (delta.norm() < .08) velocity = (delta / (t - r.last_aim_time)).cwiseMax(-2).cwiseMin(2);
    }
    r.last_aim = desired;
    r.last_aim_time = t;
    desired += velocity * (5. / 65);
    double current = data()->qpos[joints_[i][0]];
    desired[0] = std::clamp(current + wrap(desired[0] - current), -6.28, 6.28);
    desired[1] = std::clamp(desired[1], -.45, .5);
    r.angles += (desired - r.angles).cwiseMax(-.036).cwiseMin(.036);
    r.search_origin = r.angles;
    r.search_time = t;
  } else if (automatic && t - r.tracker.last_seen > .6) {
    r.last_aim.reset();
    double elapsed = std::max(0., t - r.search_time - .6),
           amplitude = std::min(1.8, .2 + .15 * std::max(0., elapsed - 3));
    r.angles[0] = std::clamp(r.search_origin[0] + amplitude * std::sin(elapsed * 2), -6.28, 6.28);
    r.angles[1] = r.search_origin[1];
  }
  for (int a = 0; a < 2; ++a) data()->ctrl[servos_[i][a]] = r.angles[a];
  r.aim_error = 180;
  if (r.aim) {
    Vec3 axis = rotation_at(data()->site_xmat, muzzles_[i]).col(0),
         direction = (*r.aim - vector_at(data()->site_xpos, muzzles_[i])).normalized();
    r.aim_error = std::acos(std::clamp(axis.dot(direction), -1., 1.)) * 180 / pi;
  }
  r.reason = r.heat.locked ? "热量锁枪" : r.tracker.ready(t) ? "跟踪中" : "搜索装甲";
  bool wants = settings.at(i == 0 ? "auto_fire" : "enemy_fire");
  auto [reason, wait] = fire_gate(i, true);
  r.fire_reason = reason;
  r.fire_wait = wait;
  if (wants && automatic && reason.empty()) {
    shoot(i);
    r.fire_reason = "已发射";
  } else if (!wants)
    r.fire_reason = "自动开火关闭";
  else if (!automatic)
    r.fire_reason = "手动瞄准：使用单发";
}
std::pair<std::string, double> Duel::fire_gate(int i, bool automatic) const {
  const auto& r = robots[i];
  if (paused_) return {"已暂停", 0};
  if (!winner_.empty() || r.hp <= 0) return {"回合结束", 0};
  if (r.heat.locked) return {"超热锁枪", r.heat.value / r.heat.rate};
  if (settings.at("protect_heat").get<bool>() && r.heat.value + 10 > r.heat.limit + 1e-8)
    return {"热量冷却", (r.heat.value + 10 - r.heat.limit) / r.heat.rate};
  double remaining = 1 / settings.at("fire_rate").get<double>() - (time() - r.last_shot);
  if (remaining > 1e-8) return {"发射间隔", remaining};
  if (automatic) {
    if (!r.tracker.ready(time())) return {"等待有效视觉跟踪", 0};
    if (!r.aim) return {"等待装甲转入正面", 0};
    Vec3 local = rotation_at(data()->site_xmat, muzzles_[i]).transpose() *
                 (*r.aim - vector_at(data()->site_xpos, muzzles_[i]));
    if (local.x() <= 0 ||
        std::abs(local.y()) > std::max(.018, .062 * r.aim_facing - projectile_radius) ||
        std::abs(local.z()) > .052 - projectile_radius)
      return {"云台进入射击窗口中", 0};
  }
  return {"", 0};
}
bool Duel::shoot(int i) {
  auto& r = robots[i];
  if (!fire_gate(i).first.empty()) return false;
  if (!r.heat.fire(time(), settings.at("protect_heat"))) return false;
  Vec3 p = vector_at(data()->site_xpos, muzzles_[i]),
       v = rotation_at(data()->site_xmat, muzzles_[i]).col(0) * settings.at("speed").get<double>() +
           muzzle_velocity(i);
  projectiles.push_back({i, p, v, time()});
  ++r.shots;
  r.last_shot = time();
  if (r.heat.locked) event(teams[i] + " 超热：锁枪并中断第一视角，冷却至零恢复");
  return true;
}
void Duel::advance_projectiles(double dt, const std::vector<double>& old_positions,
                               const std::vector<double>& old_rotations) {
  std::vector<Projectile> alive;
  for (auto bullet : projectiles) {
    Vec3 old = bullet.position, next = old + bullet.velocity * dt + Vec3(0, 0, -4.905) * dt * dt;
    bullet.velocity.z() -= 9.81 * dt;
    double first = 2;
    int owner = 1 - bullet.owner;
    Collider hit{};
    Vec3 hit_a, hit_b;
    if ((next - vector_at(data()->xpos, chassis_[owner])).norm() < .8)
      for (auto c : colliders_[owner]) {
        Vec3 a = rotation_at(old_rotations.data(), c.gid).transpose() *
                 (old - vector_at(old_positions.data(), c.gid)),
             b = rotation_at(data()->geom_xmat, c.gid).transpose() *
                 (next - vector_at(data()->geom_xpos, c.gid)),
             half = vector_at(model()->geom_size, c.gid) + Vec3::Constant(projectile_radius);
        std::optional<double> f;
        if (c.kind == mjGEOM_BOX)
          f = segment_box(a, b, half);
        else if (c.kind == mjGEOM_ELLIPSOID)
          f = segment_ellipsoid(a, b, half);
        else
          f = segment_cylinder(a, b, half[0], half[1]);
        if (f && *f < first) {
          first = *f;
          hit = c;
          hit_a = a;
          hit_b = b;
        }
      }
    if (first <= 1) {
      Vec3 at = hit_a + (hit_b - hit_a) * first;
      double normal_speed = -(hit_b.x() - hit_a.x()) / dt;
      bool valid =
          hit.armor &&
          std::abs(at.x() - (model()->geom_size[3 * hit.gid] + projectile_radius)) < 1e-5 &&
          normal_speed > 12;
      auto& target = robots[owner];
      double last = target.last_damage.contains(hit.gid) ? target.last_damage.at(hit.gid) : -10;
      if (valid && time() - last >= .05 - 1e-8) {
        target.last_damage[hit.gid] = time();
        bool invulnerable = settings.at("invulnerable");
        if (!invulnerable) target.hp = std::max(0, target.hp - 20);
        ++robots[bullet.owner].hits;
        event(teams[bullet.owner] + " 命中 " + teams[owner] + " 装甲 · " +
              (invulnerable ? "无敌计分" : "−20 HP"));
        if (target.hp == 0) {
          winner_ = teams[bullet.owner];
          event(winner_ + " 获胜 · 重置开始下一回合");
        }
      }
      trails_.push_back({old, next, time(), bullet.owner});
      continue;
    }
    if (next.z() < projectile_radius || std::abs(next.x()) > 5.94 || std::abs(next.y()) > 3.94 ||
        time() - bullet.born > 3)
      continue;
    bullet.position = next;
    alive.push_back(bullet);
    trails_.push_back({old, next, time(), bullet.owner});
  }
  projectiles = std::move(alive);
  std::erase_if(trails_, [&](auto& t) { return time() - t.time >= .1; });
  if (trails_.size() > 300) trails_.erase(trails_.begin(), trails_.end() - 300);
}
void Duel::step() {
  if (paused_ || !winner_.empty()) return;
  auto begin = std::chrono::steady_clock::now();
  for (int i = 0; i < 2; ++i) perceive(i);
  double t = time(), v = settings.at("target_speed"), w = settings.at("target_spin");
  std::string motion = settings.at("enemy_motion");
  Vec3 enemy = Vec3::Zero();
  if (motion == "strafe")
    enemy = {0, v * std::cos(t * .8), 0};
  else if (motion == "circle")
    enemy = {v, 0, w};
  else if (motion == "spin")
    enemy = {0, 0, w};
  for (int i = 0; i < 2; ++i) {
    Vec3 velocity = i == 1                                               ? enemy
                    : std::chrono::steady_clock::now() < drive_deadline_ ? drive_
                                                                         : Vec3::Zero();
    Vec3 p = vector_at(data()->xpos, chassis_[i]);
    if (std::abs(p.x()) > 5.3 || std::abs(p.y()) > 3.3)
      velocity.head<2>() =
          rotation_at(data()->xmat, chassis_[i]).topLeftCorner<2, 2>().transpose() *
          (-p.head<2>() * .25);
    double x = velocity.x(), y = velocity.y(), z = .36 * velocity.z();
    Eigen::Vector4d wheels(x - y - z, x + y + z, x + y - z, x - y + z);
    wheels /= .076;
    wheels /= std::max(1., wheels.cwiseAbs().maxCoeff() / 85);
    for (int a = 0; a < 4; ++a) data()->ctrl[drives_[i][a]] = wheels[a];
  }
  int ticks = std::lround(.05 / model()->opt.timestep),
      control_ticks = std::max(1L, std::lround(.01 / model()->opt.timestep));
  for (int tick = 0; tick < ticks; ++tick) {
    if (tick % control_ticks == 0)
      for (int i = 0; i < 2; ++i) {
        plan_aim(i);
        control(i);
      }
    std::vector<double> positions(data()->geom_xpos, data()->geom_xpos + model()->ngeom * 3),
        rotations(data()->geom_xmat, data()->geom_xmat + model()->ngeom * 9);
    mj_step(model(), data());
    advance_projectiles(model()->opt.timestep, positions, rotations);
    for (auto& r : robots) r.heat.cool(time());
    if (!winner_.empty()) {
      mju_zero(data()->ctrl, model()->nu);
      break;
    }
  }
  compute_ms_ =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
}
Json Duel::frame(const std::string& view) {
  if (view == "scene") {
    mjvCamera camera;
    mjv_defaultCamera(&camera);
    camera.distance = 7.7;
    camera.elevation = -48;
    camera.azimuth = 90;
    camera.lookat[2] = .1;
    if (settings.at("view") == "follow") {
      Vec3 pos = vector_at(data()->xpos, chassis_[0]) + Vec3(0, 0, .15);
      std::copy(pos.data(), pos.data() + 3, camera.lookat);
      Mat3 R = rotation_at(data()->xmat, chassis_[0]);
      camera.azimuth = std::atan2(R(1, 0), R(0, 0)) * 180 / pi + 180;
      camera.distance = 2.8;
      camera.elevation = -25;
    }
    auto& r = renderer();
    r.update(data(), camera);
    overlay(r.scene());
    return jpeg_response(r.read(), 88);
  }
  if (view != "camera" && view != "raw" && view != "number")
    throw std::invalid_argument("unknown frame view");
  if (raw_[0].empty()) perceive(0);
  const auto& r = robots[0];
  cv::Mat image;
  if (view == "number")
    image = r.detection && !r.heat.locked ? r.detection->classification.roi
                                          : cv::Mat::zeros(28, 20, CV_8U);
  else {
    image = (view == "raw" ? raw_[0] : annotated_[0]);
    if (r.heat.locked) {
      image = cv::Mat::zeros(600, 800, CV_8UC3);
      cv::putText(image, "OVERHEAT - VIDEO LOCKED", {150, 300}, cv::FONT_HERSHEY_SIMPLEX, 1,
                  {255, 160, 80}, 2);
    }
  }
  return jpeg_response(image, 88);
}
Json Duel::state() const {
  Json result = Json::array();
  for (int i = 0; i < 2; ++i) {
    auto& r = robots[i];
    Mat3 R = rotation_at(data()->xmat, chassis_[i]);
    Json center = nullptr, omega = nullptr, radii = nullptr, number = nullptr, confidence = nullptr,
         range = nullptr, reprojection = nullptr, aim_plate = nullptr;
    if (r.tracker.x) {
      center = array(r.tracker.center(time()));
      omega = (*r.tracker.x)[7];
      radii = Json::array({(*r.tracker.x)[8], r.tracker.r2()});
    }
    if (r.detection) {
      number = r.detection->classification.number;
      confidence = rounded(r.detection->classification.confidence, 4);
      range = rounded(cv_vector(r.detection->pose.tvec).norm(), 2);
      reprojection = rounded(r.detection->pose.error, 3);
    }
    if (r.aim) aim_plate = r.aim_plate;
    result.push_back(
        {{"team", teams[i]},
         {"hp", r.hp},
         {"heat", rounded(r.heat.value, 2)},
         {"limit", r.heat.limit},
         {"locked", r.heat.locked},
         {"shots", r.shots},
         {"hits", r.hits},
         {"accuracy", r.shots ? rounded(100. * r.hits / r.shots, 1) : 0},
         {"position", array(vector_at(data()->xpos, chassis_[i]))},
         {"yaw", std::atan2(R(1, 0), R(0, 0))},
         {"gimbal", Json::array({data()->qpos[joints_[i][0]], data()->qpos[joints_[i][1]]})},
         {"tracking", r.tracker.ready(time())},
         {"reason", r.reason},
         {"fire_reason", paused_ ? "已暂停" : r.fire_reason},
         {"fire_wait", rounded(r.fire_wait, 2)},
         {"number", number},
         {"confidence", confidence},
         {"tracker_state", r.tracker.state},
         {"switches", r.tracker.switches},
         {"armor_index", r.tracker.index},
         {"aim_plate", aim_plate},
         {"center", center},
         {"omega", omega},
         {"radii", radii},
         {"dz", r.tracker.dz()},
         {"range", range},
         {"reprojection", reprojection},
         {"aim_error", rounded(r.aim_error, 3)},
         {"flight_ms", rounded(r.flight * 1000, 1)}});
  }
  return {{"revision", revision_},
          {"time", rounded(time(), 2)},
          {"paused", paused_},
          {"winner", winner_.empty() ? Json(nullptr) : Json(winner_)},
          {"robots", result},
          {"settings", settings},
          {"events", events_},
          {"projectiles", projectiles.size()},
          {"compute_ms", rounded(compute_ms_, 1)}};
}
Json Duel::command(const std::string& name, const Json& args) {
  if (!args.is_object()) throw std::invalid_argument("command args must be an object");
  if (name == "state") return state();
  if (name == "tick") {
    step();
    return state();
  }
  if (name == "frame") {
    if (args.contains("view") && !args.at("view").is_string())
      throw std::invalid_argument("view must be string");
    return frame(args.value("view", std::string("scene")));
  }
  Json result = {{"ok", true}};
  if (name == "reset")
    reset();
  else if (name == "pause") {
    if (args.contains("paused") && !args.at("paused").is_boolean())
      throw std::invalid_argument("paused must be boolean");
    paused_ = args.value("paused", !paused_);
    drive_.setZero();
    drive_deadline_ = {};
  } else if (name == "drive") {
    Vec3 velocity(number(args, "vx", -1, 1, 0), number(args, "vy", -1, 1, 0),
                  number(args, "wz", -1.5, 1.5, 0));
    double ttl = number(args, "_drive_ttl_ms", 0, 350, 350);
    drive_ = paused_ || !winner_.empty() ? Vec3::Zero() : velocity;
    drive_deadline_ = std::chrono::steady_clock::now() +
                      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double, std::milli>(ttl));
  } else if (name == "aim") {
    Eigen::Vector2d angles(number(args, "yaw", -6.28, 6.28), number(args, "pitch", -.45, .45));
    robots[0].angles = angles;
  } else if (name == "fire") {
    auto [reason, wait] = fire_gate(0);
    result = {{"fired", shoot(0)}, {"reason", reason}, {"wait", rounded(wait, 2)}};
  } else if (name == "settings") {
    Json candidate = settings;
    for (auto it = args.begin(); it != args.end(); ++it) {
      auto key = it.key();
      if (!settings.contains(key)) throw std::invalid_argument("unknown setting " + key);
      if (settings.at(key).is_boolean()) {
        if (!it.value().is_boolean()) throw std::invalid_argument(key + " must be boolean");
      } else if (key == "speed")
        number(args, key, 13, 25);
      else if (key == "fire_rate")
        number(args, key, 1, 20);
      else if (key == "target_speed" || key == "target_spin")
        number(args, key, 0, 4);
      else {
        if (!it.value().is_string()) throw std::invalid_argument(key + " must be string");
        std::string value = it.value();
        bool valid = key == "enemy_motion" ? (value == "static" || value == "strafe" ||
                                              value == "circle" || value == "spin")
                     : key == "profile"    ? (value == "cooling" || value == "burst")
                                           : (value == "overview" || value == "follow");
        if (!valid) throw std::invalid_argument("unknown " + key);
      }
      candidate[key] = it.value();
    }
    if (candidate.at("invulnerable").get<bool>() && !settings.at("invulnerable").get<bool>()) {
      for (auto& r : robots) r.hp = 200;
      winner_.clear();
    }
    if (candidate.at("auto_aim") != settings.at("auto_aim")) {
      auto& r = robots[0];
      r.angles = {data()->qpos[joints_[0][0]], data()->qpos[joints_[0][1]]};
      r.last_aim.reset();
      r.search_origin = r.angles;
      r.search_time = time();
    }
    bool reset_profile = candidate.at("profile") != settings.at("profile");
    settings = std::move(candidate);
    if (reset_profile) reset();
  } else
    throw std::invalid_argument("unknown command " + name);
  result["revision"] = ++revision_;
  return result;
}
}  // namespace rm::duel
