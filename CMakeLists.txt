cmake_minimum_required(VERSION 3.8)
project(ros2_vi_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "/usr/lib/cmake")
set(OpenGL_GL_PREFERENCE GLVND)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(ORB_SLAM3_DIR "~/opt/ORB_SLAM3")
set(PANGOLIN_LIB_DIR "~/opt/Pangolin/build/src/libpangolin.so")

include_directories(include)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)

add_executable(ros2_vi_slam src/ros_vi_stereo.cpp src/vi_stereo_node.cpp)

link_directories(${ORB_SLAM3_DIR})

target_link_libraries(ros2_vi_slam
    ${cv_bridge_LIBRARIES}
    ${OpenCV_LIBS}
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${PANGOLIN_LIB_DIR}
)

ament_target_dependencies(ros2_vi_slam
    rclcpp
    sensor_msgs
    cv_bridge
    nav_msgs
)

install(TARGETS
    ros2_vi_slam
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
