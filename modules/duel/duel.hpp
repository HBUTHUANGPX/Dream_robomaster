#pragma once
#include <chrono>
#include <map>
#include <memory>

#include "rm/sim.hpp"
#include "tracker.hpp"
#include "vision.hpp"
namespace rm::duel {
struct Robot {
  std::unique_ptr<Detector> detector;
  RotorTracker tracker;
  Heat heat{"cooling"};
  int hp = 200, shots = 0, hits = 0, aim_plate = 0;
  double last_shot = -10, flight = 0, aim_error = 180, fire_wait = 0, aim_facing = 0,
         last_aim_time = 0, search_time = 0;
  std::optional<Detection> detection;
  std::optional<Vec3> aim;
  Eigen::Vector2d angles{0, -.05}, search_origin{0, -.05};
  std::optional<Eigen::Vector2d> last_aim;
  std::map<int, double> last_damage;
  std::string reason = "等待图像", fire_reason = "自动开火关闭";
};
class Duel {
  std::filesystem::path root_;
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_;
  std::unique_ptr<rm::Renderer> renderer_;
  double start_ = 0;
  uint64_t revision_ = 0;
  double compute_ms_ = 0;
  std::chrono::steady_clock::time_point drive_deadline_;
  Vec3 drive_ = Vec3::Zero();
  std::array<cv::Mat, 2> raw_, annotated_;
  std::array<int, 2> chassis_, cameras_, muzzles_;
  std::array<std::array<int, 4>, 2> drives_;
  std::array<std::array<int, 2>, 2> servos_, joints_;
  struct Collider {
    int gid, kind;
    bool armor;
  };
  std::array<std::vector<Collider>, 2> colliders_;
  struct Trail {
    Vec3 a, b;
    double time;
    int owner;
  };
  std::vector<Trail> trails_;
  Json events_ = Json::array();
  bool paused_ = false;
  std::string winner_;
  rm::Renderer& renderer();
  void event(const std::string&);
  Vec3 muzzle_velocity(int) const;
  void perceive(int);
  void plan_aim(int);
  void control(int);
  std::pair<std::string, double> fire_gate(int, bool = false) const;
  bool shoot(int);
  void overlay(mjvScene&);
  cv::Mat render_camera(int);
  Json frame(const std::string&);

 public:
  Json settings;
  std::array<Robot, 2> robots;
  std::vector<Projectile> projectiles;
  explicit Duel(const std::filesystem::path&);
  double time() const;
  void reset();
  void step();
  Json state() const;
  Json command(const std::string&, const Json&);
  mjModel* model() const { return model_.get(); }
  mjData* data() const { return data_.get(); }
  void advance_projectiles(double, const std::vector<double>&, const std::vector<double>&);
};
}  // namespace rm::duel
