#pragma once
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <vector>

#include "physics.hpp"
namespace rm::duel {
struct Classification {
  std::string number;
  double confidence = 0;
  cv::Mat roi;
};
class NumberClassifier {
  cv::dnn::Net net;
  std::vector<std::string> labels;

 public:
  explicit NumberClassifier(const std::filesystem::path&);
  Classification classify_roi(const cv::Mat&);
  Classification classify(const cv::Mat&, const std::vector<cv::Point2d>&);
};
struct Pose {
  cv::Mat rvec, tvec;
  double error = 0;
  std::vector<cv::Point2d> projected;
};
struct Detection {
  std::vector<cv::Point2d> corners;
  Pose pose;
  std::vector<Pose> poses;
  Classification classification;
};
class Detector {
  NumberClassifier classifier;
  bool red;

 public:
  cv::Matx33d K;
  std::vector<cv::Point3d> points;
  Detector(const std::filesystem::path&, bool enemy_red);
  std::vector<Detection> detect(const cv::Mat&);
  cv::Mat annotate(const cv::Mat&, const std::optional<Detection>&, const std::optional<Vec3>&);
  double refine_yaw(const Detection&, const Vec3&, const Vec3&, const Mat3&) const;
};
Vec3 cv_vector(const cv::Mat&);
}  // namespace rm::duel
