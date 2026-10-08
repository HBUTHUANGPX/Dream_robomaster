#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include "rm/sim.hpp"
namespace rm::nav {
using V2 = Eigen::Vector2d;
using V3 = Eigen::Vector3d;
using V4 = Eigen::Vector4d;
using M3 = Eigen::Matrix3d;
using Points = std::vector<V3>;
constexpr double pi = 3.14159265358979323846, res = .1;
double parse_number(const std::string& value);
void robot_viewer(Simulation&, const std::optional<V3>& requested, double duration);
double wrap(double a);
M3 rotation(double yaw);
Json json(const V3& v);
Json json(const V4& v);
Json json(const Points& v, size_t stride = 1);
struct Triangle {
  std::array<V3, 3> v;
  V3 normal;
};
std::vector<Triangle> load_stl(const std::filesystem::path&);
struct Terrain {
  int rows = 0, cols = 0;
  double width = 0, height = 0;
  std::vector<double> heights, clearance;
  std::vector<unsigned char> valid, reachable;
  V3 spawn;
  explicit Terrain(const std::filesystem::path& stl);
  Terrain(int rows, int cols);
  int cell(double x, double y) const;
  V3 point(int cell) const;
  double height_at(double x, double y) const;
  bool valid_at(int r, int c) const;
  Points plan(const V3& start, const V3& goal) const;
};
class KDTree {
  struct Node {
    int point, left = -1, right = -1, axis = 0;
  };
  std::vector<Node> nodes;
  Points points;
  int build(std::vector<int>& ids, int begin, int end, int depth);
  void search(int node, const V3& p, double& d, int& idx) const;

 public:
  explicit KDTree(const Points& p);
  std::pair<int, double> nearest(const V3& p, double maximum = 1e100) const;
};
struct Mapper {
  V4 pose;
  Points map_points, prior, normals;
  double rmse = INFINITY;
  int matched = 0;
  std::unique_ptr<KDTree> tree;
  Points target;
  bool dirty = false;
  Mapper(V4 initial, Points prior = {}, Points normals = {});
  V4 update(const Points& scan, const V4& odometry);
};
struct RawCloud {
  double stamp;
  std::string frame;
  Points points;
  M3 rotation;
  V3 translation;
  Json json() const;
};
struct Sensors {
  mjModel* model;
  std::array<int, 4> dofs;
  std::array<int, 2> sites;
  int chassis, imu_site, gyro_address, accel_address, attitude_address, frame = 0, rays;
  std::mt19937 random{36};
  V3 velocity = V3::Zero();
  M3 attitude = M3::Identity();
  std::vector<RawCloud> raw;
  explicit Sensors(mjModel*, int rays = 1600);
  std::pair<Points, V4> sample(mjData*, double dt);
  Json imu(mjData*) const;
};
struct PID {
  double kp, ki, kd, limit, tau, integral = 0, integral_before = 0, previous = 0, derivative = 0;
  bool has_previous = false;
  PID(double p, double i, double d, double l, double t = .15)
      : kp(p), ki(i), kd(d), limit(l), tau(t) {}
  void reset();
  double update(double error, double dt);
};
struct Tracker {
  std::array<PID, 3> pid{PID(1.4, .16, .06, .25), PID(1.4, .16, .06, .25), PID(1.8, .08, .06, .30)};
  V3 error = V3::Zero();
  void reset();
  V3 update(const V4&, const V3&, const V3&, double);
  void accept(const V3&, const V3&);
};
struct Drive {
  double mass, inertia, budget;
  Eigen::Matrix<double, 4, 3> J, force_to_torque;
  explicit Drive(double mass = 21.258, double budget = 120);
  V4 wheels(const V3&) const;
  double electrical(const V4& torque, const V4& omega) const;
  V4 limit_torque(const V4&, const V4&) const;
  struct Prediction {
    double power;
    V4 torque, omega;
  };
  Prediction predict(const V3&, const V3&, double, const V2& gravity = V2::Zero()) const;
  bool feasible(const V3&, const V3&, double, const V2& gravity = V2::Zero()) const;
};
struct DWA {
  Terrain& terrain;
  Drive& drive;
  double horizon = 1.;
  V2 gravity = V2::Zero();
  int candidates = 0, admissible = 0;
  DWA(Terrain& t, Drive& d, double h = 1) : terrain(t), drive(d), horizon(h) {}
  std::pair<V3, V3> window(const V3&, double) const;
  std::pair<Points, int> rollout(const V4&, const V3&) const;
  bool terrain_check(const std::vector<V2>&, double* clearance = nullptr) const;
  bool line_clear(const V2&, const V2&) const;
  bool safe(const V4&, const V3&, const Points&, double* clearance = nullptr) const;
  bool motion_valid(const V3&, const V3&, double) const;
  std::pair<V3, Points> plan(const V4&, const V3&, const Points&, const V3&, const Points&,
                             double speed = 1.2, double yaw = 0);
  std::pair<V3, bool> constrain(const V4&, const V3&, const V3&, const V3&, const Points&, double);
};
struct Task {
  std::filesystem::path root;
  std::string localization;
  Terrain terrain;
  Simulation simulation;
  Sensors sensors;
  Drive drive;
  DWA planner;
  Tracker tracker;
  std::array<PID, 3> speed_feedback{PID(1.1, .6, 0, .7), PID(1.1, .6, 0, .7), PID(1., .3, 0, .6)};
  Points prior, prior_normals, path, local, trail, scan;
  std::unique_ptr<Mapper> mapper;
  std::unique_ptr<Renderer> renderer;
  std::optional<V3> goal, reference;
  V3 velocity = V3::Zero(), measured_velocity = V3::Zero();
  V2 last_position = V2::Zero();
  std::string status;
  bool paused = false;
  int index = 0, localization_failures = 0;
  double stuck_time = 0, slip_seconds = 0, yaw_rate_request = 0, power_w = 0, power_peak_w = 0,
         energy_j = 0, mechanical_w = 0;
  Task(const std::filesystem::path&, std::string localization = "prior", double budget = 120);
  void reset();
  void sense();
  void set_goal(double x, double y);
  void set_yaw_rate(double);
  V3 stop();
  V3 control();
  void step(const std::function<void(const Json&)>& imu_callback = {});
  Json state() const;
  Json map() const;
  Json presets() const;
  Json frame();
  Json handle(const std::string&, const Json&);
};
}  // namespace rm::nav
