cmake_minimum_required(VERSION 3.5)
project(spot_nvblox)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python node scripts as executables (runnable as a launch Node executable).
# depth_min_clip_node: clips the hand camera's near depth (sim gripper artifact).
install(PROGRAMS
  scripts/depth_min_clip_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME depth_min_clip_node
)

# republish_rgb.py: BGR8 -> RGB8 relay, used only on the real robot path.
install(PROGRAMS
  scripts/republish_rgb.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
