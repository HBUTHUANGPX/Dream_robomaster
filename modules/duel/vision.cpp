#include "vision.hpp"

#include <algorithm>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
namespace rm::duel {
Vec3 cv_vector(const cv::Mat& m) { return {m.at<double>(0), m.at<double>(1), m.at<double>(2)}; }
NumberClassifier::NumberClassifier(const std::filesystem::path& root)
    : net(cv::dnn::readNetFromONNX((root / "assets/rm_auto_aim/mlp.onnx").string())) {
  std::ifstream file(root / "assets/rm_auto_aim/label.txt");
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    labels.push_back(line);
  }
  if (labels.empty()) throw std::runtime_error("missing classifier labels");
}
Classification NumberClassifier::classify_roi(const cv::Mat& input) {
  cv::Mat gray, binary, normalized;
  cv::resize(input, gray, {20, 28}, 0, 0, cv::INTER_LINEAR);
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  binary.convertTo(normalized, CV_32F, 1. / 255);
  net.setInput(cv::dnn::blobFromImage(normalized));
  cv::Mat logits = net.forward().reshape(1, 1);
  double maximum;
  cv::Point location;
  cv::minMaxLoc(logits, nullptr, &maximum, nullptr, &location);
  double sum = 0;
  for (int i = 0; i < logits.cols; ++i) sum += std::exp(logits.at<float>(0, i) - maximum);
  double confidence = 1 / sum;
  std::string label = labels.at(location.x);
  cv::Scalar mean, stddev;
  cv::meanStdDev(gray, mean, stddev);
  if (confidence < .8 || label == "negative" || label == "1" || label == "base" || stddev[0] < 2)
    label.clear();
  return {label, confidence, binary};
}
Classification NumberClassifier::classify(const cv::Mat& rgb,
                                          const std::vector<cv::Point2d>& corners) {
  std::vector<cv::Point2f> src;
  for (int i : {1, 0, 2, 3}) src.emplace_back(corners[i]);
  std::vector<cv::Point2f> dst{{0, 19}, {0, 7}, {31, 7}, {31, 19}};
  cv::Mat warped, gray;
  cv::warpPerspective(rgb, warped, cv::getPerspectiveTransform(src, dst), {32, 28});
  cv::cvtColor(warped(cv::Rect(6, 0, 20, 28)), gray, cv::COLOR_RGB2GRAY);
  return classify_roi(gray);
}
Detector::Detector(const std::filesystem::path& root, bool enemy_red)
    : classifier(root), red(enemy_red) {
  double f = 300 / std::tan(pi / 8);
  K = cv::Matx33d(f, 0, 399.5, 0, f, 299.5, 0, 0, 1);
  points = {{-.062, -.026, 0}, {-.062, .026, 0}, {.062, -.026, 0}, {.062, .026, 0}};
}
std::vector<Detection> Detector::detect(const cv::Mat& rgb) {
  cv::Mat mask(rgb.rows, rgb.cols, CV_8U);
  for (int y = 0; y < rgb.rows; ++y)
    for (int x = 0; x < rgb.cols; ++x) {
      auto c = rgb.at<cv::Vec3b>(y, x);
      int main = c[red ? 0 : 2], other = c[red ? 2 : 0];
      mask.at<unsigned char>(y, x) = main > 100 && main - other > 65 && main - c[1] > 45 ? 255 : 0;
    }
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  struct Light {
    cv::Point2d center, top, bottom;
    double height;
  };
  std::vector<Light> lights;
  for (auto& contour : contours) {
    if (contour.size() < 4) continue;
    auto rect = cv::minAreaRect(contour);
    cv::Point2f box[4];
    rect.points(box);
    double lengths[4];
    int longest = 0;
    for (int j = 0; j < 4; ++j) {
      lengths[j] = cv::norm(box[(j + 1) % 4] - box[j]);
      if (lengths[j] > lengths[longest]) longest = j;
    }
    double height = lengths[longest] + 1, width = lengths[(longest + 1) % 4] + 1;
    cv::Point2d axis = (box[(longest + 1) % 4] - box[longest]) / std::max(lengths[longest], 1e-6);
    if (axis.y < 0) axis = -axis;
    if (height < 5 || height / width < 1.5 || std::abs(axis.y) < .7) continue;
    cv::Point2d center = rect.center;
    lights.push_back({center, center - axis * height / 2, center + axis * height / 2, height});
  }
  std::sort(lights.begin(), lights.end(), [](auto& a, auto& b) { return a.center.x < b.center.x; });
  std::vector<Detection> detections;
  for (size_t i = 0; i < lights.size(); ++i)
    for (size_t j = i + 1; j < lights.size(); ++j) {
      auto& a = lights[i];
      auto& b = lights[j];
      double height = (a.height + b.height) / 2, dx = b.center.x - a.center.x,
             dy = b.center.y - a.center.y;
      if (dx / height <= .8 || dx / height >= 3.8 || std::abs(dy) > .65 * height ||
          std::min(a.height, b.height) / std::max(a.height, b.height) < .6)
        continue;
      Detection d;
      d.corners = {a.top, a.bottom, b.top, b.bottom};
      std::vector<cv::Mat> rvecs, tvecs;
      cv::solvePnPGeneric(points, d.corners, K, cv::noArray(), rvecs, tvecs, false,
                          cv::SOLVEPNP_IPPE);
      for (size_t k = 0; k < tvecs.size(); ++k) {
        Vec3 tv = cv_vector(tvecs[k]);
        if (!tv.allFinite() || tv.z() <= .3 || tv.z() >= 15) continue;
        Pose pose{rvecs[k], tvecs[k], 0, {}};
        cv::projectPoints(points, pose.rvec, pose.tvec, K, cv::noArray(), pose.projected);
        for (int n = 0; n < 4; ++n) {
          auto delta = pose.projected[n] - d.corners[n];
          pose.error += delta.dot(delta) / 4;
        }
        pose.error = std::sqrt(pose.error);
        if (pose.error < 2.5) d.poses.push_back(pose);
      }
      if (!d.poses.empty()) {
        d.pose = d.poses.front();
        d.classification = classifier.classify(rgb, d.corners);
        detections.push_back(d);
      }
    }
  std::sort(detections.begin(), detections.end(),
            [](auto& a, auto& b) { return a.pose.error < b.pose.error; });
  return detections;
}
cv::Mat Detector::annotate(const cv::Mat& rgb, const std::optional<Detection>& d,
                           const std::optional<Vec3>& prediction) {
  auto out = rgb.clone();
  cv::drawMarker(out, {out.cols / 2, out.rows / 2}, {220, 220, 220}, cv::MARKER_CROSS, 18, 1);
  if (d) {
    std::vector<cv::Point> polygon;
    for (int i : {0, 2, 3, 1}) polygon.emplace_back(d->corners[i]);
    cv::polylines(out, polygon, true, {70, 255, 130}, 1, cv::LINE_AA);
    for (auto& p : d->pose.projected) cv::circle(out, p, 2, {255, 215, 70}, -1);
    cv::putText(
        out, cv::format("PnP %.2fm  reproj %.2fpx", cv_vector(d->pose.tvec).norm(), d->pose.error),
        {12, 25}, cv::FONT_HERSHEY_SIMPLEX, .55, {90, 255, 150}, 1, cv::LINE_AA);
    cv::putText(out,
                cv::format("ID %s  ONNX %.1f%%", d->classification.number.c_str(),
                           d->classification.confidence * 100),
                {12, 48}, cv::FONT_HERSHEY_SIMPLEX, .55, {90, 255, 150}, 1, cv::LINE_AA);
  }
  if (prediction && prediction->z() > .1) {
    auto p = K * cv::Vec3d(prediction->x(), prediction->y(), prediction->z());
    double x = p[0] / p[2], y = p[1] / p[2];
    if (std::isfinite(x) && std::isfinite(y) && x > -1000 && x < out.cols + 1000 && y > -1000 &&
        y < out.rows + 1000)
      cv::drawMarker(out, {cvRound(x), cvRound(y)}, {255, 185, 45}, cv::MARKER_DIAMOND, 14, 2);
  }
  return out;
}
double Detector::refine_yaw(const Detection& d, const Vec3& position, const Vec3& camera,
                            const Mat3& R) const {
  double bearing = std::atan2(camera.y() - position.y(), camera.x() - position.x()), best = 1e100,
         result = bearing;
  for (int i = -80; i <= 80; ++i) {
    double yaw = bearing + i * pi / 180, c = std::cos(yaw), s = std::sin(yaw);
    Vec3 horizontal(-s, c, 0),
        vertical(-std::sin(pi / 12) * c, -std::sin(pi / 12) * s, std::cos(pi / 12));
    double error = 0;
    for (int j = 0; j < 4; ++j) {
      Vec3 local =
          R.transpose() * (position + points[j].x * horizontal - points[j].y * vertical - camera);
      auto p = K * cv::Vec3d(local.x(), local.y(), local.z());
      error +=
          std::pow(p[0] / p[2] - d.corners[j].x, 2) + std::pow(p[1] / p[2] - d.corners[j].y, 2);
    }
    if (error < best) {
      best = error;
      result = yaw;
    }
  }
  return result;
}
}  // namespace rm::duel
