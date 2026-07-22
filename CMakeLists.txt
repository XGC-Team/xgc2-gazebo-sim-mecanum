cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_mecanum)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
)
find_package(gazebo REQUIRED)

catkin_package(
  LIBRARIES gazebo_sim_mecanum_contract
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(gazebo_sim_mecanum_contract SHARED
  src/mecanum_contract_plugin.cpp
)
target_compile_features(gazebo_sim_mecanum_contract PRIVATE cxx_std_14)
add_dependencies(gazebo_sim_mecanum_contract ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_sim_mecanum_contract
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS gazebo_sim_mecanum_contract
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/check_model_ready.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES LICENSE MODEL_ASSET_NOTICE.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES process-definitions/xgc2-gazebo-sim-mecanum.json
  DESTINATION /usr/share/xgc2/process-definitions
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  roslaunch_add_file_check(launch)
  add_rostest(test/ideal_drive.test)
endif()
