cmake_minimum_required(VERSION 3.8)
project(rrr_challenge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Find MuJoCo
find_library(MUJOCO_LIBRARY mujoco REQUIRED)

# Add include directories
include_directories(
  include
  ${MUJOCO_INCLUDE_DIRS}
)

# Create executable
add_executable(rrr_robot src/main.cpp)
target_link_libraries(rrr_robot ${MUJOCO_LIBRARY})
ament_target_dependencies(rrr_robot rclcpp ament_index_cpp)

# Install
install(TARGETS rrr_robot
  DESTINATION lib/${PROJECT_NAME})

# Install model files to include directory (for C++ header inclusion)
install(DIRECTORY include/
  DESTINATION include)

# Install URDF files (e.g., rrr.urdf)
# This installs files from include/rrr_challenge/urdf/ to share/rrr_challenge/urdf/
install(DIRECTORY include/rrr_challenge/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf/)


install(DIRECTORY include/rrr_challenge/urdf/mesh/
  DESTINATION share/${PROJECT_NAME}/urdf/mesh/
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  include/rrr_challenge/scripts/controller.py
  include/rrr_challenge/scripts/end_effector_listener.py
  include/rrr_challenge/scripts/master_node.py
  include/rrr_challenge/scripts/visualization_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()