#include "rm/sim.hpp"

#include <cmath>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>

void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
  try {
    check(argc == 2, "root argument required");
    bool rejected = false;
    try {
      rm::Simulation missing("/definitely/missing/model.xml");
    } catch (const std::exception&) {
      rejected = true;
    }
    check(rejected, "missing model must fail cleanly");
    rm::Simulation sim(std::filesystem::path(argv[1]) / "assets/robot.xml");
    check(sim.model->nu == 4, "robot must have four actuators");
    for (int i = 0; i < 100; ++i) mj_step(sim.model, sim.data);
    check(std::isfinite(sim.data->qpos[2]), "physics must remain finite");
    rm::Renderer render(sim.model, 320, 240);
    mjvCamera camera;
    mjv_defaultCamera(&camera);
    camera.distance = 1.8;
    camera.elevation = -25;
    camera.lookat[2] = 0.15;
    auto frame = render.render(sim.data, camera);
    check(frame.rows == 240 && frame.cols == 320 && frame.type() == CV_8UC3,
          "RGB frame dimensions");
    cv::Scalar mean, deviation;
    cv::meanStdDev(frame, mean, deviation);
    check(deviation[0] > 4, "frame must contain rendered geometry");
    auto encoded = rm::jpeg_response(frame);
    check(encoded.at("mime") == "image/jpeg", "JPEG content type");
    check(encoded.at("data").get<std::string>().starts_with("/9j/"), "JPEG must be base64 encoded");
    std::cout << "MuJoCo " << mj_versionString() << ": model, physics, EGL, RGB/JPEG passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
