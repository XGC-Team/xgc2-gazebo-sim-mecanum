cmake_minimum_required(VERSION 3.16)
project(gazebo_sim_mecanum)
find_package(PythonInterp 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  xgc2_gazebo_scene
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
find_package(gazebo REQUIRED)
find_package(XgcXrpc REQUIRED CONFIG)

catkin_package(
  LIBRARIES gazebo_sim_mecanum_contract
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros xgc2_gazebo_scene
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
target_compile_features(gazebo_sim_mecanum_contract PRIVATE cxx_std_20)
add_dependencies(gazebo_sim_mecanum_contract ${catkin_EXPORTED_TARGETS})
find_package(Threads REQUIRED)
target_link_libraries(gazebo_sim_mecanum_contract
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  Threads::Threads
  XgcXrpc::http
)

install(TARGETS gazebo_sim_mecanum_contract
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/check_model_ready.py
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

if(CATKIN_ENABLE_TESTING)
  find_package(PythonInterp 3 REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  add_test(
    NAME mecanum_optional_simple_lidar_render
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_simple_lidar_render.py
  )

endif()

# Scientific full-vehicle regression runs only against a caller-prepared world.
set(XGC2_TEST_WORLD "" CACHE FILEPATH "Prepared isolated native world")
set(XGC2_TEST_SERVICE_REF_JSON "" CACHE STRING "Explicit native simulation-v1 ServiceRef")
set(XGC2_TEST_TARGET_ID "" CACHE STRING "Native fixture local target")
if(CATKIN_ENABLE_TESTING AND XGC2_TEST_WORLD AND XGC2_TEST_SERVICE_REF_JSON AND XGC2_TEST_TARGET_ID)
  add_rostest(test/ideal_drive.test ARGS "world_name:=${XGC2_TEST_WORLD}" "simulation_service_ref_json:=${XGC2_TEST_SERVICE_REF_JSON}" "target_id:=${XGC2_TEST_TARGET_ID}")
  add_rostest(test/high_fidelity_drive.test ARGS "world_name:=${XGC2_TEST_WORLD}" "simulation_service_ref_json:=${XGC2_TEST_SERVICE_REF_JSON}" "target_id:=${XGC2_TEST_TARGET_ID}")
endif()
