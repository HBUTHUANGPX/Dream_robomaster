#include "rm/rosbag.hpp"

#include <bit>
#include <fstream>
#include <limits>
#include <map>
namespace rm::nav {
namespace {
using Bytes = std::string;
template <class T>
Bytes scalar(T value) {
  static_assert(std::endian::native == std::endian::little,
                "ROS bag writer currently requires little-endian host");
  return Bytes(reinterpret_cast<const char*>(&value), sizeof(T));
}
Bytes u32(uint32_t n) { return scalar(n); }
Bytes u64(uint64_t n) { return scalar(n); }
Bytes str(const std::string& s) { return u32(uint32_t(s.size())) + s; }
uint64_t timestamp(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0 || seconds >= 4294967296.)
    throw std::invalid_argument("Timestamp exceeds ROS1 uint32 seconds range");
  return uint64_t(std::llround(seconds * 1e9));
}
Bytes time(uint64_t ns) { return u32(uint32_t(ns / 1000000000)) + u32(uint32_t(ns % 1000000000)); }
using Fields = std::vector<std::pair<std::string, Bytes>>;
Bytes fields(const Fields& values) {
  Bytes bytes;
  for (auto& [key, value] : values) bytes += str(key + "=" + value);
  return bytes;
}
Bytes record(unsigned char op, Fields header, const Bytes& data) {
  header.insert(header.begin(), {"op", Bytes(1, char(op))});
  return str(fields(header)) + str(data);
}
Bytes message_header(uint32_t seq, uint64_t ns, const std::string& frame) {
  return u32(seq) + time(ns) + str(frame);
}
const char* imu_definition = R"(std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
)";
const char* cloud_definition = R"(std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
================================================================================
MSG: sensor_msgs/PointField
uint8 INT8=1
uint8 UINT8=2
uint8 INT16=3
uint8 UINT16=4
uint8 INT32=5
uint8 UINT32=6
uint8 FLOAT32=7
uint8 FLOAT64=8
string name
uint32 offset
uint8 datatype
uint32 count
)";
Bytes connection(int id) {
  std::string topic = id ? "/sim/imu" : "/sim/lidar",
              type = id ? "sensor_msgs/Imu" : "sensor_msgs/PointCloud2",
              md5 = id ? "6a62c6daae103f4ff57a132d6f95cec2" : "1158d486dd51d683ce2f1be655c3c181";
  return record(7, {{"conn", u32(id)}, {"topic", topic}},
                fields({{"topic", topic},
                        {"type", type},
                        {"md5sum", md5},
                        {"message_definition", id ? imu_definition : cloud_definition}}));
}
Bytes bag_header(uint64_t index, uint32_t chunks) {
  Fields h{{"op", Bytes(1, 3)},
           {"index_pos", u64(index)},
           {"conn_count", u32(2)},
           {"chunk_count", u32(chunks)}};
  auto head = str(fields(h));
  return head + str(Bytes(4096 - 4 - head.size(), ' '));
}
}  // namespace
struct RawBag::Impl {
  struct Chunk {
    uint64_t position, start, end;
    std::array<uint32_t, 2> counts;
  };
  std::ofstream out;
  Bytes pending;
  std::array<std::vector<std::pair<uint64_t, uint32_t>>, 2> indices;
  std::vector<Chunk> chunks;
  bool closed = false;
  explicit Impl(const std::filesystem::path& path) {
    if (std::filesystem::exists(path)) throw std::invalid_argument("Bag output already exists");
    out.open(path, std::ios::binary);
    if (!out) throw std::runtime_error("Cannot create bag");
    write("#ROSBAG V2.0\n");
    write(bag_header(0, 0));
    pending = connection(0) + connection(1);
  }
  void write(const Bytes& b) {
    out.write(b.data(), std::streamsize(b.size()));
    if (!out) throw std::runtime_error("ROS bag write failed");
  }
  void flush() {
    if (pending.empty()) return;
    Chunk chunk{uint64_t(out.tellp()), UINT64_MAX, 0, {0, 0}};
    write(record(5, {{"compression", "none"}, {"size", u32(uint32_t(pending.size()))}}, pending));
    for (int id = 0; id < 2; ++id)
      if (!indices[id].empty()) {
        Bytes index;
        for (auto [stamp, offset] : indices[id]) {
          index += time(stamp) + u32(offset);
          chunk.start = std::min(chunk.start, stamp);
          chunk.end = std::max(chunk.end, stamp);
        }
        chunk.counts[id] = uint32_t(indices[id].size());
        write(record(4, {{"ver", u32(1)}, {"conn", u32(id)}, {"count", u32(chunk.counts[id])}},
                     index));
        indices[id].clear();
      }
    if (chunk.start == UINT64_MAX) chunk.start = 0;
    chunks.push_back(chunk);
    pending.clear();
  }
  void append(int id, uint64_t stamp, const Bytes& data) {
    if (closed) throw std::logic_error("Bag already closed");
    indices[id].push_back({stamp, uint32_t(pending.size())});
    pending += record(2, {{"conn", u32(id)}, {"time", time(stamp)}}, data);
    if (pending.size() > 1024 * 1024) flush();
  }
  void close() {
    if (closed) return;
    flush();
    uint64_t index = uint64_t(out.tellp());
    write(connection(0));
    write(connection(1));
    for (auto c : chunks) {
      Bytes counts;
      uint32_t active = 0;
      for (int id = 0; id < 2; ++id)
        if (c.counts[id]) {
          ++active;
          counts += u32(id) + u32(c.counts[id]);
        }
      write(record(6,
                   {{"ver", u32(1)},
                    {"chunk_pos", u64(c.position)},
                    {"start_time", time(c.start)},
                    {"end_time", time(c.end)},
                    {"count", u32(active)}},
                   counts));
    }
    out.seekp(13);
    write(bag_header(index, uint32_t(chunks.size())));
    out.close();
    closed = true;
  }
};
RawBag::RawBag(const std::filesystem::path& path) : impl(std::make_unique<Impl>(path)) {}
RawBag::~RawBag() {
  try {
    close();
  } catch (...) {
  }
}
void RawBag::close() { impl->close(); }
void RawBag::imu(const Json& p) {
  auto stamp = timestamp(p.at("stamp").get<double>());
  Bytes msg = message_header(count[1], stamp, "imu");
  for (double q : {0., 0., 0., 1.}) msg += scalar(q);
  for (int i = 0; i < 9; ++i) msg += scalar(i == 0 ? -1. : 0.);
  for (auto key : {"angular_velocity", "acceleration"}) {
    if (!p.at(key).is_array() || p.at(key).size() != 3)
      throw std::invalid_argument("IMU vector must have three components");
    for (auto v : p.at(key)) {
      double number = v.get<double>();
      if (!std::isfinite(number)) throw std::invalid_argument("Nonfinite IMU value");
      msg += scalar(number);
    }
    for (int i = 0; i < 9; ++i) msg += scalar(0.);
  }
  impl->append(1, stamp, msg);
  ++count[1];
}
void RawBag::cloud(const RawCloud& p) {
  auto stamp = timestamp(p.stamp);
  if (p.points.size() > (std::numeric_limits<uint32_t>::max() - 1024) / 16)
    throw std::invalid_argument("PointCloud2 exceeds ROS1 record size limit");
  uint32_t size = uint32_t(p.points.size());
  Bytes msg = message_header(count[0], stamp, "mid360_top");
  msg += u32(1) + u32(size) + u32(4);
  int i = 0;
  for (auto name : {"x", "y", "z", "intensity"}) {
    msg += str(name) + u32(i++ * 4) + Bytes(1, 7) + u32(1);
  }
  msg += Bytes(1, 0) + u32(16) + u32(size * 16) + u32(size * 16);
  for (auto point : p.points) {
    if (!point.allFinite() || point.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max())
      throw std::invalid_argument("Cloud value cannot be represented as finite Float32");
    for (int axis = 0; axis < 3; ++axis) msg += scalar(float(point[axis]));
    msg += scalar(1.f);
  }
  msg += Bytes(1, 1);
  impl->append(0, stamp, msg);
  ++count[0];
}
}  // namespace rm::nav
