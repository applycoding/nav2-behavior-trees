# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom

cmake_minimum_required(VERSION 3.22)
project(nav2_bt_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED CONFIG)
if(NOT TARGET BT::behaviortree_cpp AND NOT TARGET behaviortree_cpp::behaviortree_cpp)
  message(FATAL_ERROR "BehaviorTree.CPP not found; set CMAKE_PREFIX_PATH to /opt/btcpp-4.11 when apt is not 4.11.0")
endif()
find_package(ament_index_cpp REQUIRED)
find_package(OpenSSL REQUIRED)

if(TARGET BT::behaviortree_cpp)
  set(NAV2_BT_BTCPP_LIB BT::behaviortree_cpp)
elseif(TARGET behaviortree_cpp::behaviortree_cpp)
  set(NAV2_BT_BTCPP_LIB behaviortree_cpp::behaviortree_cpp)
else()
  message(FATAL_ERROR "BehaviorTree.CPP imported target not found after find_package")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SimStep.srv"
  "srv/GetWorldSample.srv"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>)
endif()

set(BT_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(BT_CORE_SOURCES
  ${BT_REPO_ROOT}/src/grid_sim.cpp
  ${BT_REPO_ROOT}/src/frozen_world.cpp
  ${BT_REPO_ROOT}/src/mission_state.cpp
  ${BT_REPO_ROOT}/src/scenarios.cpp
  ${BT_REPO_ROOT}/src/nodes/mission_nodes.cpp
  ${BT_REPO_ROOT}/src/runner.cpp
  ${BT_REPO_ROOT}/src/trace_writer.cpp
  ${BT_REPO_ROOT}/src/status_change_logger.cpp)

add_library(bt_core_ros STATIC ${BT_CORE_SOURCES})
set_target_properties(bt_core_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(bt_core_ros PUBLIC ${BT_REPO_ROOT}/include)
target_link_libraries(bt_core_ros PUBLIC ${NAV2_BT_BTCPP_LIB} OpenSSL::Crypto)
target_compile_options(bt_core_ros PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(nav2_bt_ros_lib STATIC
  src/ros_nav_backend.cpp
  src/mock_nav2.cpp
  src/ros_runner.cpp)
target_include_directories(nav2_bt_ros_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_bt_ros_lib
  rclcpp rclcpp_action nav2_msgs std_msgs nav_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_dependencies(nav2_bt_ros_lib ${PROJECT_NAME}__rosidl_generator_cpp)
target_link_libraries(nav2_bt_ros_lib bt_core_ros ${cpp_typesupport_target})

if(DEFINED ENV{BTCPP_INSTALL_LIB} AND NOT "$ENV{BTCPP_INSTALL_LIB}" STREQUAL "")
  set(BTCPP_RPATH "$ENV{BTCPP_INSTALL_LIB}")
else()
  set(BTCPP_RPATH "")
endif()

function(nav2_bt_set_rpath target)
  if(BTCPP_RPATH)
    set_target_properties(${target} PROPERTIES
      BUILD_RPATH "${BTCPP_RPATH}"
      INSTALL_RPATH "${BTCPP_RPATH}"
      BUILD_WITH_INSTALL_RPATH TRUE)
  endif()
endfunction()

function(nav2_bt_link_static_btcpp target)
  foreach(_prefix IN LISTS CMAKE_PREFIX_PATH)
    if(_prefix STREQUAL "")
      continue()
    endif()
    set(_bt_a "${_prefix}/lib/libbehaviortree_cpp.a")
    set(_tx_a "${_prefix}/lib/libtinyxml2.a")
    if(EXISTS "${_bt_a}" AND EXISTS "${_tx_a}")
      target_link_libraries(${target} PRIVATE "${_bt_a}" "${_tx_a}")
      return()
    endif()
  endforeach()
  if(EXISTS "/opt/btcpp-4.11/lib/libbehaviortree_cpp.a")
    target_link_libraries(${target} PRIVATE
      /opt/btcpp-4.11/lib/libbehaviortree_cpp.a
      /opt/btcpp-4.11/lib/libtinyxml2.a)
  endif()
endfunction()

add_executable(mock_nav2_servers src/mock_servers_node.cpp)
ament_target_dependencies(mock_nav2_servers rclcpp)
target_link_libraries(mock_nav2_servers nav2_bt_ros_lib)
nav2_bt_link_static_btcpp(mock_nav2_servers)

add_executable(bt_mission_node src/mission_node.cpp)
nav2_bt_set_rpath(mock_nav2_servers)
nav2_bt_set_rpath(bt_mission_node)
ament_target_dependencies(bt_mission_node rclcpp ament_index_cpp)
target_link_libraries(bt_mission_node nav2_bt_ros_lib)
nav2_bt_link_static_btcpp(bt_mission_node)

install(TARGETS mock_nav2_servers bt_mission_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${BT_REPO_ROOT}/trees
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ros_backend
    test/test_ros_backend.cpp
    ${BT_REPO_ROOT}/tests/scenario_gates.cpp)
  ament_target_dependencies(test_ros_backend rclcpp)
  target_link_libraries(test_ros_backend nav2_bt_ros_lib)
  nav2_bt_link_static_btcpp(test_ros_backend)
  nav2_bt_set_rpath(test_ros_backend)
  target_compile_definitions(test_ros_backend PRIVATE BT_SOURCE_DIR="${BT_REPO_ROOT}")
  set_tests_properties(test_ros_backend PROPERTIES TIMEOUT 3600)

  ament_add_gtest(test_readelf_no_v3 test/test_readelf_no_v3.cpp)
  target_compile_definitions(test_readelf_no_v3 PRIVATE
    "BT_MISSION_NODE_PATH=\"$<TARGET_FILE:bt_mission_node>\"")
  add_dependencies(test_readelf_no_v3 bt_mission_node)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_launch_nominal.py)
  set_tests_properties(test_test_launch_nominal.py PROPERTIES TIMEOUT 1200)
endif()

ament_package()
