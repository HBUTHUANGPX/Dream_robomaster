set(RM_LOCAL_PREFIX "${PROJECT_SOURCE_DIR}/.deps/sysroot/usr")
list(PREPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/.deps/opencv" "${RM_LOCAL_PREFIX}")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.9 REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)

set(MUJOCO_ROOT "$ENV{MUJOCO_ROOT}" CACHE PATH "MuJoCo C SDK directory")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS "${MUJOCO_ROOT}/include" "${PROJECT_SOURCE_DIR}/.deps/mujoco/include")
find_library(MUJOCO_LIBRARY NAMES mujoco
  HINTS "${MUJOCO_ROOT}/lib" "${MUJOCO_ROOT}/bin" "${MUJOCO_ROOT}"
        "${PROJECT_SOURCE_DIR}/.deps/mujoco/lib" "${PROJECT_SOURCE_DIR}/.deps/mujoco")
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo C SDK missing: set MUJOCO_ROOT or run tools/bootstrap-native.sh")
endif()
add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")

# System packages or the reproducibly built local SDK; never Python bindings.
find_package(OpenCV 4.10 CONFIG QUIET COMPONENTS core imgproc imgcodecs calib3d dnn)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV >=4.10 C++ SDK missing: run tools/bootstrap-native.sh or set OpenCV_DIR")
endif()
get_filename_component(RM_MUJOCO_LIBDIR "${MUJOCO_LIBRARY}" DIRECTORY)
list(APPEND CMAKE_BUILD_RPATH "${RM_MUJOCO_LIBDIR}" "${RM_LOCAL_PREFIX}/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
