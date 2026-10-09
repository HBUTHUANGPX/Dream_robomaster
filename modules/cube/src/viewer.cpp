#include <GLFW/glfw3.h>

#include <chrono>
#include <deque>
#include <iostream>
#include <thread>

#include "rm/cube.hpp"
namespace rm::cube {
int interactive_viewer(Cube& cube, const std::filesystem::path& root,
                       const std::vector<std::string>& scramble) {
  if (!glfwInit())
    throw std::runtime_error(
        "GLFW initialization failed; use --rpc or --headless without a display");
  auto* window = glfwCreateWindow(
      1100, 800, "Native physical cube — R/L/U/D/F/B, S scramble, Z solve", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    throw std::runtime_error("Cannot create GLFW window");
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  struct Input {
    std::deque<int> keys;
    mjvCamera camera;
    bool left = false, right = false;
    double x = 0, y = 0;
  } input;
  mjv_defaultCamera(&input.camera);
  input.camera.lookat[2] = cube.dual ? .22 : .105;
  input.camera.distance = cube.rx ? .36 : cube.dual ? .69 : .24;
  input.camera.azimuth = 135;
  input.camera.elevation = -30;
  glfwSetWindowUserPointer(window, &input);
  glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int, int action, int) {
    if (action == GLFW_PRESS) {
      if (key == GLFW_KEY_ESCAPE)
        glfwSetWindowShouldClose(w, 1);
      else
        static_cast<Input*>(glfwGetWindowUserPointer(w))->keys.push_back(key);
    }
  });
  glfwSetMouseButtonCallback(window, [](GLFWwindow* w, int, int, int) {
    auto* p = static_cast<Input*>(glfwGetWindowUserPointer(w));
    p->left = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    p->right = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(w, &p->x, &p->y);
  });
  glfwSetCursorPosCallback(window, [](GLFWwindow* w, double x, double y) {
    auto* p = static_cast<Input*>(glfwGetWindowUserPointer(w));
    if (p->left) {
      p->camera.azimuth -= (x - p->x) * .3;
      p->camera.elevation = std::clamp(p->camera.elevation - (y - p->y) * .3, -89., 89.);
    }
    if (p->right) {
      p->camera.lookat[0] -= (x - p->x) * p->camera.distance * .001;
      p->camera.lookat[2] += (y - p->y) * p->camera.distance * .001;
    }
    p->x = x;
    p->y = y;
  });
  glfwSetScrollCallback(window, [](GLFWwindow* w, double, double y) {
    auto* p = static_cast<Input*>(glfwGetWindowUserPointer(w));
    p->camera.distance = std::clamp(p->camera.distance * std::exp(-y * .1), .05, 3.);
  });
  mjvOption option;
  mjv_defaultOption(&option);
  if (cube.rx) {
    option.geomgroup[3] = 0;
    option.sitegroup[4] = 0;
  }
  mjvScene scene;
  mjv_defaultScene(&scene);
  mjv_makeScene(cube.model, &scene, 4000);
  mjrContext context;
  mjr_defaultContext(&context);
  mjr_makeContext(cube.model, &context, mjFONTSCALE_150);
  auto began = std::chrono::steady_clock::now();
  double sim_start = cube.data->time, last = -1;
  bool stopped = false;
  auto draw = [&](Cube& c) {
    if (glfwWindowShouldClose(window)) throw std::runtime_error("Viewer closed during execution");
    if (c.data->time - last < 1. / 60) return;
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    mjrRect viewport = {0, 0, width, height};
    mjv_updateScene(c.model, c.data, &option, nullptr, &input.camera, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &context);
    std::string status = stopped ? "Safety stop: restart viewer to reset"
                                 : "R/L/U/D/F/B: turn | S: scramble | Z: solve | mouse: camera";
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, status.c_str(), nullptr, &context);
    glfwSwapBuffers(window);
    glfwPollEvents();
    last = c.data->time;
    auto deadline = began + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                std::chrono::duration<double>(c.data->time - sim_start));
    std::this_thread::sleep_until(deadline);
  };
  try {
    while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();
      if (stopped) {
        last = -1;
        draw(cube);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
        continue;
      }
      if (input.keys.empty()) {
        cube.advance(.016, draw);
        continue;
      }
      int key = input.keys.front();
      input.keys.pop_front();
      std::vector<std::string> sequence;
      if (key == 'S')
        sequence = scramble;
      else if (key == 'Z')
        sequence = solve_facelets(cube.facelets(), root);
      else if (faces.find(static_cast<char>(key)) != std::string::npos)
        sequence = {std::string(1, static_cast<char>(key))};
      try {
        if (key == 'Z' && cube.dual && !cube.robot_ready) cube.initialize_grasps(draw);
        if (cube.robot_ready)
          cube.execute(optimize_plan(compile_moves(sequence, cube.orientation), cube.orientation),
                       draw);
        else
          for (auto& m : sequence) cube.turn(m, draw);
      } catch (const std::exception& e) {
        std::cerr << "Viewer safety stop: " << e.what() << '\n';
        stopped = true;
      }
    }
  } catch (const std::exception& e) {
    if (!glfwWindowShouldClose(window)) {
      mjr_freeContext(&context);
      mjv_freeScene(&scene);
      glfwDestroyWindow(window);
      glfwTerminate();
      throw;
    }
  }
  mjr_freeContext(&context);
  mjv_freeScene(&scene);
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
}  // namespace rm::cube
