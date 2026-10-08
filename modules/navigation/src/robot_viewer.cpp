#include <GLFW/glfw3.h>

#include <chrono>
#include <thread>

#include "rm/navigation.hpp"
namespace rm::nav {
void robot_viewer(Simulation& simulation, const std::optional<V3>& requested, double duration) {
  if (!glfwInit())
    throw std::runtime_error("GLFW initialization failed; select --headless without a display");
  auto* window = glfwCreateWindow(
      1100, 800, "Native RoboMaster | W/S A/D Q/E | SPACE stop | P demo", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    throw std::runtime_error("Cannot create GLFW window");
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  struct Input {
    V3 velocity = V3::Zero();
    bool demo = false, left = false;
    double x = 0, y = 0;
    mjvCamera camera;
  };
  Input input;
  input.velocity = requested.value_or(V3::Zero());
  mjv_defaultCamera(&input.camera);
  input.camera.distance = 1.4;
  input.camera.elevation = -28;
  input.camera.azimuth = 135;
  glfwSetWindowUserPointer(window, &input);
  glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    auto& in = *static_cast<Input*>(glfwGetWindowUserPointer(w));
    if (key == GLFW_KEY_ESCAPE) {
      glfwSetWindowShouldClose(w, 1);
      return;
    }
    if (key == GLFW_KEY_P) {
      in.demo = !in.demo;
      return;
    }
    switch (key) {
      case GLFW_KEY_W:
        in.velocity = V3(.4, 0, 0);
        break;
      case GLFW_KEY_S:
        in.velocity = V3(-.4, 0, 0);
        break;
      case GLFW_KEY_A:
        in.velocity = V3(0, .4, 0);
        break;
      case GLFW_KEY_D:
        in.velocity = V3(0, -.4, 0);
        break;
      case GLFW_KEY_Q:
        in.velocity = V3(0, 0, .7);
        break;
      case GLFW_KEY_E:
        in.velocity = V3(0, 0, -.7);
        break;
      case GLFW_KEY_SPACE:
        in.velocity.setZero();
        break;
      default:
        return;
    }
    in.demo = false;
  });
  glfwSetMouseButtonCallback(window, [](GLFWwindow* w, int, int, int) {
    auto& in = *static_cast<Input*>(glfwGetWindowUserPointer(w));
    in.left = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    glfwGetCursorPos(w, &in.x, &in.y);
  });
  glfwSetCursorPosCallback(window, [](GLFWwindow* w, double x, double y) {
    auto& in = *static_cast<Input*>(glfwGetWindowUserPointer(w));
    if (in.left) {
      in.camera.azimuth -= (x - in.x) * .3;
      in.camera.elevation = std::clamp(in.camera.elevation - (y - in.y) * .3, -89., 89.);
    }
    in.x = x;
    in.y = y;
  });
  glfwSetScrollCallback(window, [](GLFWwindow* w, double, double y) {
    auto& in = *static_cast<Input*>(glfwGetWindowUserPointer(w));
    in.camera.distance = std::clamp(in.camera.distance * std::exp(-y * .1), .15, 15.);
  });
  mjvOption option;
  mjv_defaultOption(&option);
  mjvScene scene;
  mjv_defaultScene(&scene);
  mjrContext context;
  mjr_defaultContext(&context);
  mjv_makeScene(simulation.model, &scene, 4000);
  mjr_makeContext(simulation.model, &context, mjFONTSCALE_150);
  Drive drive;
  auto m = simulation.model;
  auto d = simulation.data;
  double start = d->time;
  auto wall_start = std::chrono::steady_clock::now();
  std::array<V3, 4> demo{V3(.4, 0, 0), V3(0, .4, 0), V3(0, 0, .7), V3::Zero()};
  while (!glfwWindowShouldClose(window) && d->time - start < duration) {
    glfwPollEvents();
    V3 command = input.demo ? demo[int((d->time - start) / 3) % 4] : input.velocity;
    V4 wheels = drive.wheels(command);
    wheels /= std::max(1., wheels.cwiseAbs().maxCoeff() / 35.);
    for (int j = 0; j < 4; ++j) d->ctrl[j] = wheels[j];
    for (int i = 0; i < int(std::round(.01 / m->opt.timestep)); ++i) mj_step(m, d);
    for (int i = 0; i < 3; ++i) input.camera.lookat[i] = d->qpos[i] + (i == 2 ? .08 : 0);
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    mjrRect viewport{0, 0, width, height};
    mjv_updateScene(m, d, &option, nullptr, &input.camera, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &context);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                "W/S forward | A/D strafe | Q/E yaw | SPACE stop | P demo", nullptr, &context);
    glfwSwapBuffers(window);
    auto deadline = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                     std::chrono::duration<double>(d->time - start));
    std::this_thread::sleep_until(deadline);
  }
  mjr_freeContext(&context);
  mjv_freeScene(&scene);
  glfwDestroyWindow(window);
  glfwTerminate();
}
}  // namespace rm::nav
