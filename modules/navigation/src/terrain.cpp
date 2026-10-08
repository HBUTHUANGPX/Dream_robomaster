#include <algorithm>
#include <fstream>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <queue>

#include "rm/navigation.hpp"
namespace rm::nav {
double parse_number(const std::string& value) {
  size_t consumed = 0;
  double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result))
    throw std::invalid_argument("Expected a finite numeric argument");
  return result;
}
double wrap(double a) {
  a = std::fmod(a + pi, 2 * pi);
  if (a < 0) a += 2 * pi;
  return a - pi;
}
M3 rotation(double yaw) { return Eigen::AngleAxisd(yaw, V3::UnitZ()).toRotationMatrix(); }
Json json(const V3& v) { return Json::array({v.x(), v.y(), v.z()}); }
Json json(const V4& v) { return Json::array({v[0], v[1], v[2], v[3]}); }
Json json(const Points& v, size_t stride) {
  Json j = Json::array();
  for (size_t i = 0; i < v.size(); i += stride) j.push_back(json(v[i]));
  return j;
}
std::vector<Triangle> load_stl(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("Cannot read arena STL: " + path.string());
  char header[80];
  uint32_t count = 0;
  in.read(header, 80);
  in.read(reinterpret_cast<char*>(&count), 4);
  if (count > 10000000 || std::filesystem::file_size(path) != 84ull + 50ull * count)
    throw std::runtime_error("Expected binary STL");
  std::vector<Triangle> out;
  out.reserve(count);
  for (uint32_t i = 0; i < count; ++i) {
    float f[12];
    uint16_t attribute;
    in.read(reinterpret_cast<char*>(f), 48);
    in.read(reinterpret_cast<char*>(&attribute), 2);
    Triangle t;
    for (int j = 0; j < 3; ++j) t.v[j] = V3(f[3 + 3 * j], f[4 + 3 * j], f[5 + 3 * j]);
    t.normal = (t.v[1] - t.v[0]).cross(t.v[2] - t.v[0]);
    if (t.normal.norm() > 1e-10) {
      t.normal.normalize();
      out.push_back(t);
    }
  }
  if (!in) throw std::runtime_error("Truncated STL");
  return out;
}
Terrain::Terrain(int r, int c)
    : rows(r),
      cols(c),
      width(c * res),
      height(r * res),
      heights(r * c, .1),
      clearance(r * c, 2),
      valid(r * c, 1),
      reachable(r * c, 1),
      spawn(7.5, 4, .1) {}
Terrain::Terrain(const std::filesystem::path& stl) {
  const auto triangles = load_stl(stl);
  V3 maximum = V3::Zero();
  for (auto& t : triangles)
    for (auto& v : t.v) maximum = maximum.cwiseMax(v);
  cols = static_cast<int>(std::ceil(maximum.x() / res));
  rows = static_cast<int>(std::ceil(maximum.y() / res));
  width = cols * res;
  height = rows * res;
  std::vector<std::vector<std::pair<double, double>>> hits(rows * cols);
  for (auto& t : triangles) {
    if (std::abs(t.normal.z()) < 1e-5) continue;
    V2 lo = t.v[0].head<2>(), hi = lo;
    for (auto& v : t.v) {
      lo = lo.cwiseMin(v.head<2>());
      hi = hi.cwiseMax(v.head<2>());
    }
    V2 a = (t.v[1] - t.v[0]).head<2>(), b = (t.v[2] - t.v[0]).head<2>();
    double det = a.x() * b.y() - a.y() * b.x();
    if (std::abs(det) < 1e-10) continue;
    for (int r = std::max(0, int(std::floor(lo.y() / res - .5)));
         r <= std::min(rows - 1, int(std::ceil(hi.y() / res - .5))); ++r)
      for (int c = std::max(0, int(std::floor(lo.x() / res - .5)));
           c <= std::min(cols - 1, int(std::ceil(hi.x() / res - .5))); ++c) {
        V2 d = V2((c + .5) * res, (r + .5) * res) - t.v[0].head<2>();
        double u = (d.x() * b.y() - d.y() * b.x()) / det, v = (a.x() * d.y() - a.y() * d.x()) / det;
        if (u >= -1e-6 && v >= -1e-6 && u + v <= 1 + 1e-6)
          hits[r * cols + c].push_back(
              {t.v[0].z() + u * (t.v[1].z() - t.v[0].z()) + v * (t.v[2].z() - t.v[0].z()),
               t.normal.z()});
      }
  }
  heights.assign(rows * cols, NAN);
  clearance.resize(rows * cols);
  valid.resize(rows * cols);
  reachable.assign(rows * cols, 0);
  for (size_t i = 0; i < hits.size(); ++i) {
    auto& hs = hits[i];
    std::sort(hs.begin(), hs.end());
    for (auto [z, nz] : hs) {
      if (nz < .01 || z < .09) continue;
      auto above = std::find_if(hs.begin(), hs.end(), [&](auto h) { return h.first > z + .025; });
      if (above != hs.end() && (above->second > 0 || above->first - z < .65)) continue;
      if (!std::isfinite(heights[i]) || std::abs(heights[i] - z) > .025) heights[i] = z;
    }
  }
  cv::Mat free(rows, cols, CV_8UC1, cv::Scalar(255));
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      if (!std::isfinite(heights[r * cols + c]) || r == 0 || c == 0 || r == rows - 1 ||
          c == cols - 1)
        free.at<unsigned char>(r, c) = 0;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      for (auto [dr, dc] : std::array<std::pair<int, int>, 4>{{{0, 1}, {1, 0}, {1, 1}, {1, -1}}}) {
        int rr = (r + dr + rows) % rows, cc = (c + dc + cols) % cols;
        if (std::abs(heights[r * cols + c] - heights[rr * cols + cc]) >
            std::hypot(dr, dc) * res * std::tan(28 * pi / 180) + .008) {
          free.at<unsigned char>(r, c) = 0;
          free.at<unsigned char>(rr, cc) = 0;
        }
      }
  cv::Mat distance;
  cv::distanceTransform(free, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  for (int i = 0; i < rows * cols; ++i) {
    clearance[i] = distance.at<float>(i / cols, i % cols) * res;
    valid[i] = std::isfinite(heights[i]) && clearance[i] >= .38;
  }
  spawn = V3(7.5, 4, height_at(7.5, 4));
  std::queue<int> q;
  int s = cell(spawn.x(), spawn.y());
  if (valid[s]) {
    q.push(s);
    reachable[s] = 1;
  }
  while (!q.empty()) {
    int p = q.front();
    q.pop();
    for (auto [dr, dc] : std::array<std::pair<int, int>, 4>{{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}}) {
      int r = p / cols + dr, c = p % cols + dc;
      if (valid_at(r, c) && !reachable[r * cols + c]) {
        reachable[r * cols + c] = 1;
        q.push(r * cols + c);
      }
    }
  }
}
int Terrain::cell(double x, double y) const {
  return std::clamp(int(y / res), 0, rows - 1) * cols + std::clamp(int(x / res), 0, cols - 1);
}
V3 Terrain::point(int p) const {
  return V3((p % cols + .5) * res, (p / cols + .5) * res, heights[p]);
}
double Terrain::height_at(double x, double y) const { return heights[cell(x, y)]; }
bool Terrain::valid_at(int r, int c) const {
  return r >= 0 && r < rows && c >= 0 && c < cols && valid[r * cols + c];
}
Points Terrain::plan(const V3& start, const V3& goal) const {
  int a = cell(start.x(), start.y()), b = cell(goal.x(), goal.y());
  if (!valid[b]) throw std::invalid_argument("目标不满足底盘净空、边缘余量或坡度要求");
  if (!valid[a]) {
    int nearest = -1;
    double dist = INFINITY;
    for (int i = 0; i < rows * cols; ++i)
      if (valid[i]) {
        double d = std::hypot(i / cols - a / cols, i % cols - a % cols);
        if (d < dist) {
          dist = d;
          nearest = i;
        }
      }
    if (nearest < 0 || dist * res > .5)
      throw std::invalid_argument("当前位置偏离可通行区域，已停车");
    a = nearest;
  }
  std::vector<double> cost(rows * cols, INFINITY);
  std::vector<int> parent(rows * cols, -1);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  cost[a] = 0;
  queue.push({0, a});
  while (!queue.empty()) {
    int p = queue.top().second;
    queue.pop();
    if (p == b) break;
    for (int dr = -1; dr <= 1; ++dr)
      for (int dc = -1; dc <= 1; ++dc) {
        if (!dr && !dc) continue;
        int r = p / cols + dr, c = p % cols + dc;
        if (!valid_at(r, c) || (dr && dc && (!valid_at(r, p % cols) || !valid_at(p / cols, c))))
          continue;
        int n = r * cols + c;
        double step = std::hypot(dr, dc) * res, dz = std::abs(heights[n] - heights[p]);
        if (dz > step * std::tan(28 * pi / 180) + .008) continue;
        double value = cost[p] + step + 2 * dz + .006 / std::max(clearance[n], .1);
        if (value < cost[n]) {
          cost[n] = value;
          parent[n] = p;
          queue.push({value + std::hypot(r - b / cols, c - b % cols) * res, n});
        }
      }
  }
  if (!std::isfinite(cost[b]))
    throw std::invalid_argument("目标与机器人之间没有满足底盘约束的连续通路");
  Points result;
  for (int p = b;; p = parent[p]) {
    result.push_back(point(p));
    if (p == a) break;
  }
  std::reverse(result.begin(), result.end());
  return result;
}
}  // namespace rm::nav
