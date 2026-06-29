cmake_minimum_required(VERSION 3.8)
project(lidar_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# 生成可执行文件
add_executable(lidar_node src/lidar_node.cpp)

# 把所有依赖全部加在这里！！！
ament_target_dependencies(lidar_node
  rclcpp
  sensor_msgs
  OpenCV
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
)

# 链接 OpenCV 库
target_link_libraries(lidar_node ${OpenCV_LIBS})

# 安装
install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
