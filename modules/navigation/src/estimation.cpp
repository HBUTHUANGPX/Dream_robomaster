#include <algorithm>
#include <limits>
#include <numeric>
#include <set>
#include <unordered_set>

#include "rm/navigation.hpp"
namespace rm::nav {
KDTree::KDTree(const Points& p) : points(p) {
  std::vector<int> ids(p.size());
  std::iota(ids.begin(), ids.end(), 0);
  nodes.reserve(p.size());
  build(ids, 0, int(ids.size()), 0);
}
int KDTree::build(std::vector<int>& ids, int begin, int end, int depth) {
  if (begin == end) return -1;
  int mid = (begin + end) / 2, axis = depth % 3;
  std::nth_element(ids.begin() + begin, ids.begin() + mid, ids.begin() + end,
                   [&](int a, int b) { return points[a][axis] < points[b][axis]; });
  int n = int(nodes.size());
  nodes.push_back({ids[mid], -1, -1, axis});
  int left = build(ids, begin, mid, depth + 1), right = build(ids, mid + 1, end, depth + 1);
  nodes[n].left = left;
  nodes[n].right = right;
  return n;
}
void KDTree::search(int n, const V3& p, double& d, int& idx) const {
  if (n < 0) return;
  auto node = nodes[n];
  double distance = (points[node.point] - p).squaredNorm();
  if (distance < d) {
    d = distance;
    idx = node.point;
  }
  double delta = p[node.axis] - points[node.point][node.axis];
  search(delta < 0 ? node.left : node.right, p, d, idx);
  if (delta * delta < d) search(delta < 0 ? node.right : node.left, p, d, idx);
}
std::pair<int, double> KDTree::nearest(const V3& p, double maximum) const {
  double d = maximum * maximum;
  int idx = -1;
  if (!nodes.empty()) search(0, p, d, idx);
  return {idx, idx < 0 ? INFINITY : std::sqrt(d)};
}
namespace {
struct Key {
  long x, y, z;
  bool operator==(const Key&) const = default;
};
struct Hash {
  size_t operator()(const Key& k) const {
    return std::hash<long>{}(k.x) ^ (std::hash<long>{}(k.y) << 1) ^ (std::hash<long>{}(k.z) << 2);
  }
};
std::vector<size_t> voxel_indices(const Points& p) {
  std::unordered_set<Key, Hash> seen;
  std::vector<size_t> result;
  for (size_t i = 0; i < p.size(); ++i) {
    auto v = p[i];
    if (!v.allFinite()) continue;
    // Bound before division and integer conversion; huge finite rows are invalid measurements.
    constexpr double key_limit = double(std::numeric_limits<long>::max()) * .05;
    if (v.cwiseAbs().maxCoeff() > key_limit) continue;
    Key key{long(std::floor(v.x() / .1)), long(std::floor(v.y() / .1)),
            long(std::floor(v.z() / .1))};
    if (seen.insert(key).second) result.push_back(i);
  }
  return result;
}
Points voxelize(const Points& p) {
  Points result;
  for (auto i : voxel_indices(p)) result.push_back(p[i]);
  return result;
}
Points world_points(const Points& p, const V4& pose) {
  Points result;
  result.reserve(p.size());
  auto r = rotation(pose[3]);
  for (auto x : p) result.push_back(r * x + pose.head<3>());
  return result;
}
}  // namespace
Mapper::Mapper(V4 initial, Points raw, Points normal) : pose(initial) {
  if (!pose.allFinite()) throw std::invalid_argument("Nonfinite initial pose");
  if (!normal.empty() && normal.size() != raw.size())
    throw std::invalid_argument("Prior normals size mismatch");
  for (size_t i = 0; i < normal.size(); ++i)
    if (!raw[i].allFinite() || !normal[i].allFinite() || normal[i].norm() < 1e-8)
      throw std::invalid_argument("Invalid prior normals");
  for (auto i : voxel_indices(raw)) {
    prior.push_back(raw[i]);
    if (!normal.empty()) normals.push_back(normal[i].normalized());
  }
  if (!prior.empty()) {
    target = prior;
    tree = std::make_unique<KDTree>(target);
  }
}
V4 Mapper::update(const Points& raw, const V4& odom) {
  if (!odom.allFinite()) throw std::invalid_argument("Nonfinite odometry");
  Points source = voxelize(raw);
  V4 predicted = pose;
  predicted.head<3>() += rotation(pose[3]) * odom.head<3>();
  predicted[3] = wrap(predicted[3] + odom[3]);
  pose = predicted;
  rmse = INFINITY;
  matched = 0;
  if (source.size() < 30) return pose;
  if (dirty) {
    target = prior.empty() ? map_points : prior;
    tree = std::make_unique<KDTree>(target);
    dirty = false;
  }
  if (tree) {
    V4 candidate = predicted;
    double error = INFINITY;
    int count = 0;
    auto bounded = [&] {
      return (candidate.head<3>() - predicted.head<3>()).norm() <= .5 &&
             std::abs(wrap(candidate[3] - predicted[3])) <= .2;
    };
    for (int iteration = 0; iteration < 20; ++iteration) {
      auto world = world_points(source, candidate);
      struct Pair {
        int source, target;
        double distance;
      };
      std::vector<Pair> pairs;
      for (size_t i = 0; i < world.size(); ++i) {
        auto [id, d] = tree->nearest(world[i], .6);
        if (id >= 0) pairs.push_back({int(i), id, d});
      }
      if (pairs.size() < std::max(30., .3 * source.size())) return pose;
      if (!normals.empty()) {
        Eigen::MatrixXd jac(pairs.size(), 4);
        Eigen::VectorXd rhs(pairs.size());
        double sum = 0, weights = 0;
        for (size_t i = 0; i < pairs.size(); ++i) {
          auto pair = pairs[i];
          V3 n = normals[pair.target], relative = world[pair.source] - candidate.head<3>();
          double residual = (world[pair.source] - target[pair.target]).dot(n),
                 weight = std::min(1., .06 / std::max(std::abs(residual), 1e-8));
          jac.block<1, 3>(i, 0) = n.transpose() * std::sqrt(weight);
          jac(i, 3) = (-relative.y() * n.x() + relative.x() * n.y()) * std::sqrt(weight);
          rhs[i] = -residual * std::sqrt(weight);
          sum += weight * residual * residual;
          weights += weight;
        }
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(jac);
        qr.setThreshold(1e-4);
        if (qr.rank() < 4) return pose;
        V4 delta = qr.solve(rhs);
        candidate += delta;
        candidate[3] = wrap(candidate[3]);
        if (!bounded()) return pose;
        error = std::sqrt(sum / weights);
        count = int(pairs.size());
        if (delta.norm() < 1e-4) break;
      } else {
        std::sort(pairs.begin(), pairs.end(),
                  [](auto a, auto b) { return a.distance < b.distance; });
        pairs.resize(size_t(.8 * pairs.size()));
        std::set<int> unique;
        for (auto p : pairs) unique.insert(p.target);
        if (pairs.size() < 30 || unique.size() < 30) return pose;
        V3 src = V3::Zero(), dst = V3::Zero();
        for (auto p : pairs) {
          src += world[p.source];
          dst += target[p.target];
        }
        src /= pairs.size();
        dst /= pairs.size();
        M3 covariance = M3::Zero();
        double cross = 0, dot = 0;
        for (auto p : pairs) {
          V3 a = world[p.source] - src, b = target[p.target] - dst;
          covariance += a * a.transpose();
          cross += a.x() * b.y() - a.y() * b.x();
          dot += a.head<2>().dot(b.head<2>());
        }
        Eigen::SelfAdjointEigenSolver<M3> eig(covariance);
        if (std::sqrt(std::max(0., eig.eigenvalues()[1]) / pairs.size()) < .08) return pose;
        double yaw = std::atan2(cross, dot);
        M3 r = rotation(yaw);
        V4 revised = candidate;
        revised.head<3>() = r * candidate.head<3>() + dst - r * src;
        revised[3] = wrap(candidate[3] + yaw);
        double change = (revised.head<3>() - candidate.head<3>()).norm();
        candidate = revised;
        if (!bounded()) return pose;
        if (change < 1e-4 && std::abs(yaw) < 1e-4) break;
      }
    }
    if (normals.empty()) {
      auto world = world_points(source, candidate);
      std::vector<std::pair<double, int>> distances;
      for (auto p : world) {
        auto [id, d] = tree->nearest(p, .6);
        if (id >= 0) distances.push_back({d, id});
      }
      if (distances.size() < std::max(30., .3 * source.size())) return pose;
      std::sort(distances.begin(), distances.end());
      distances.resize(size_t(.8 * distances.size()));
      std::set<int> unique;
      double sum = 0;
      for (auto [d, id] : distances) {
        sum += d * d;
        unique.insert(id);
      }
      if (distances.size() < 30 || unique.size() < 30) return pose;
      error = std::sqrt(sum / distances.size());
      count = int(distances.size());
    }
    if (!std::isfinite(error) || error > (normals.empty() ? .20 : .12)) return pose;
    pose = candidate;
    rmse = error;
    matched = count;
  }
  auto combined = world_points(source, pose);
  combined.insert(combined.end(), map_points.begin(), map_points.end());
  map_points = voxelize(combined);
  if (map_points.size() > 40000) map_points.resize(40000);
  dirty = prior.empty();
  return pose;
}
Json RawCloud::json() const {
  Json r = Json::array();
  for (int i = 0; i < 3; ++i)
    r.push_back(Json::array({rotation(i, 0), rotation(i, 1), rotation(i, 2)}));
  return {{"stamp", stamp},
          {"frame", frame},
          {"points", nav::json(points)},
          {"rotation", r},
          {"translation", nav::json(translation)}};
}
Sensors::Sensors(mjModel* m, int count) : model(m), rays(count) {
  const char* names[] = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
  for (int i = 0; i < 4; ++i) {
    int id = mj_name2id(m, mjOBJ_JOINT, names[i]);
    if (id < 0) throw std::runtime_error("Missing wheel joint");
    dofs[i] = m->jnt_dofadr[id];
  }
  sites = {mj_name2id(m, mjOBJ_SITE, "mid360_top_origin"),
           mj_name2id(m, mjOBJ_SITE, "mid360_ground_origin")};
  chassis = mj_name2id(m, mjOBJ_BODY, "chassis");
  imu_site = mj_name2id(m, mjOBJ_SITE, "imu");
  if (sites[0] < 0 || sites[1] < 0 || chassis < 0 || imu_site < 0)
    throw std::runtime_error("Navigation model lacks required chassis or sensor mounting sites");
  if (rays <= 0 || rays > 1000000) throw std::invalid_argument("Invalid LiDAR ray count");
  auto address = [&](const char* name) {
    int id = mj_name2id(m, mjOBJ_SENSOR, name);
    if (id < 0) throw std::runtime_error("Missing IMU sensor");
    return m->sensor_adr[id];
  };
  gyro_address = address("gyro");
  accel_address = address("accel");
  attitude_address = address("attitude");
}
std::pair<Points, V4> Sensors::sample(mjData* d, double dt) {
  const double* q = d->sensordata + attitude_address;
  Eigen::Quaterniond quaternion(q[0], q[1], q[2], q[3]);
  if (!quaternion.coeffs().allFinite() || !std::isfinite(quaternion.norm()) ||
      quaternion.norm() < 1e-12)
    throw std::invalid_argument("Invalid AHRS quaternion");
  attitude = quaternion.normalized().toRotationMatrix();
  double heading = std::atan2(attitude(1, 0), attitude(0, 0));
  M3 leveling = rotation(-heading) * attitude;
  Points cloud;
  raw.clear();
  std::normal_distribution<double> noise(0, .008);
  mjtByte groups[6] = {1, 1, 0, 0, 0, 0};
  for (int site : sites) {
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> sensor_rotation(d->site_xmat +
                                                                                   9 * site);
    V3 origin = Eigen::Map<V3>(d->site_xpos + 3 * site) + sensor_rotation.col(2) * .008;
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> imu_rotation(d->site_xmat +
                                                                                9 * imu_site);
    RawCloud packet{
        d->time,
        mj_id2name(model, mjOBJ_SITE, site),
        {},
        imu_rotation.transpose() * sensor_rotation,
        imu_rotation.transpose() * (origin - Eigen::Map<V3>(d->site_xpos + 3 * imu_site))};
    Points directions;
    std::vector<double> world(rays * 3), ranges(rays);
    std::vector<int> ids(rays, -1);
    for (int i = 0; i < rays; ++i) {
      double k = i + double(frame) * rays, az = std::fmod(k * 2.399963229728653, 2 * pi),
             el = (-7 + 59 * std::fmod(k * .7548776662466927, 1.)) * pi / 180;
      V3 direction(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      directions.push_back(direction);
      Eigen::Map<V3>(world.data() + i * 3) = sensor_rotation * direction;
    }
    mj_multiRay(model, d, origin.data(), world.data(), groups, 1, -1, ids.data(), ranges.data(),
                nullptr, rays, 40.);
    for (int i = 0; i < rays; ++i)
      if (ranges[i] >= .2 && ranges[i] <= 40 && ids[i] >= 0 && model->geom_group[ids[i]] == 0) {
        double distance = ranges[i] + noise(random);
        packet.points.push_back(directions[i] * distance);
        V3 point = origin + Eigen::Map<V3>(world.data() + 3 * i) * distance;
        cloud.push_back(leveling * attitude.transpose() *
                        (point - Eigen::Map<V3>(d->xpos + 3 * chassis)));
      }
    raw.push_back(std::move(packet));
  }
  ++frame;
  V4 w;
  for (int i = 0; i < 4; ++i) w[i] = d->qvel[dofs[i]];
  double vx = .076 * w.sum() / 4, vy = .076 * (-w[0] + w[1] + w[2] - w[3]) / 4;
  V4 odom;
  odom.head<3>() = leveling * V3(vx, vy, 0) * dt;
  odom[3] = (leveling * Eigen::Map<V3>(d->sensordata + gyro_address)).z() * dt;
  velocity = V3(vx, vy, odom[3] / dt);
  return {cloud, odom};
}
Json Sensors::imu(mjData* d) const {
  return {{"stamp", d->time},
          {"angular_velocity", nav::json(V3(Eigen::Map<V3>(d->sensordata + gyro_address)))},
          {"acceleration", nav::json(V3(Eigen::Map<V3>(d->sensordata + accel_address)))}};
}
}  // namespace rm::nav
