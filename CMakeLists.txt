# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom

cmake_minimum_required(VERSION 3.22)
project(nav2_behavior_trees LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)
set(BTCPP_SQLITE_LOGGING OFF CACHE BOOL "" FORCE)
set(BTCPP_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BTCPP_SHARED_LIBS OFF CACHE BOOL "" FORCE)

set(_nav2_bt_project_testing ${BUILD_TESTING})

FetchContent_Declare(
  behaviortree_cpp
  GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
  GIT_TAG 8dad25dce26c3fd0787ff5c8790714bc6159dabe)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(behaviortree_cpp)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG 063de7e9578f82b369302001269680b4b1553359)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(BUILD_TESTING ${_nav2_bt_project_testing} CACHE BOOL "" FORCE)

set(BT_CORE_SOURCES
    src/grid_sim.cpp
    src/frozen_world.cpp
    src/mission_state.cpp
    src/scenarios.cpp
    src/nodes/mission_nodes.cpp
    src/runner.cpp
    src/trace_writer.cpp
    src/status_change_logger.cpp)

find_package(OpenSSL REQUIRED)

add_library(bt_core STATIC ${BT_CORE_SOURCES})
target_include_directories(bt_core PUBLIC include)
target_link_libraries(bt_core PUBLIC BT::behaviortree_cpp OpenSSL::Crypto)
target_compile_options(bt_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(bt_mission apps/bt_mission.cpp)
target_link_libraries(bt_mission PRIVATE bt_core)
target_compile_options(bt_mission PRIVATE -Wall -Wextra -Wpedantic -Werror)

function(add_mutant name def)
  add_library(bt_core_${name} STATIC ${BT_CORE_SOURCES})
  target_include_directories(bt_core_${name} PUBLIC include)
  target_link_libraries(bt_core_${name} PUBLIC BT::behaviortree_cpp OpenSSL::Crypto)
  target_compile_options(bt_core_${name} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_compile_definitions(bt_core_${name} PRIVATE ${def})
endfunction()

add_mutant(mutant_no_cancel MUTANT_NO_CANCEL)
add_mutant(mutant_reset_lane MUTANT_RESET_LANE)

enable_testing()

add_executable(bt_tests
    tests/test_registry_xml.cpp
    tests/test_gridsim.cpp
    tests/test_scenarios.cpp
    tests/test_mutants.cpp
    tests/test_determinism.cpp
    tests/test_golden.cpp
    tests/scenario_gates.cpp)
target_link_libraries(bt_tests PRIVATE bt_core GTest::gtest_main)
target_compile_options(bt_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(bt_tests PRIVATE BT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    BT_MISSION_EXECUTABLE="$<TARGET_FILE:bt_mission>")
add_dependencies(bt_tests bt_mission)

include(GoogleTest)
gtest_discover_tests(bt_tests)

add_executable(bt_tests_mutant_no_cancel tests/test_mutant_builds.cpp)
target_link_libraries(bt_tests_mutant_no_cancel PRIVATE bt_core_mutant_no_cancel GTest::gtest_main)
target_compile_options(bt_tests_mutant_no_cancel PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(bt_tests_mutant_no_cancel PRIVATE BT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
                                                                 MUTANT_NO_CANCEL)
gtest_discover_tests(bt_tests_mutant_no_cancel)

add_executable(bt_tests_mutant_reset_lane tests/test_mutant_builds.cpp)
target_link_libraries(bt_tests_mutant_reset_lane PRIVATE bt_core_mutant_reset_lane GTest::gtest_main)
target_compile_options(bt_tests_mutant_reset_lane PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(bt_tests_mutant_reset_lane PRIVATE BT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
                                                                  MUTANT_RESET_LANE)
gtest_discover_tests(bt_tests_mutant_reset_lane)
