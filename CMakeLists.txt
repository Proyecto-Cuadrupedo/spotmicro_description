cmake_minimum_required(VERSION 3.8)
project(spotmicro_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  DIRECTORY config launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/spotmicro_high_level_controller.py
    scripts/spotmicro_low_level_controller.py
    scripts/spotmicro_odometry_publisher.py
    scripts/spotmicro_pose_controller.py
    scripts/spotmicro_crawl_controller.py
    scripts/spotmicro_cmdvel_pub_gui.py
    scripts/spotmicro_pose_pub_gui.py
    scripts/spotmicro_joint_force_gui.py
    scripts/spotmicro_force_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/spotmicro_kinematics.py
    scripts/spotmicro_reference.py
    scripts/spotmicro_trajectory_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
