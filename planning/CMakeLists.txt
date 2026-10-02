cmake_minimum_required(VERSION 3.10)
project(tailsitter_planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_path(NLOPT_INCLUDE_DIR nlopt.hpp REQUIRED)
find_library(NLOPT_LIBRARY nlopt REQUIRED)

set(SPLINE_TRAJECTORY_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/SplineTrajectory"
    CACHE PATH "SplineTrajectory source directory")

add_library(tailsitter_planning
    src/flat_output_trajectory.cpp
    src/time_allocation.cpp
    src/actuator_penalty_optimizer.cpp
    src/tailsitter_flatness.cpp
    src/feasibility_checker.cpp
    src/time_scaler.cpp
    src/trajectory_csv.cpp)

target_include_directories(tailsitter_planning
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${SPLINE_TRAJECTORY_ROOT}/include
    PRIVATE
        ${NLOPT_INCLUDE_DIR})
target_link_libraries(tailsitter_planning PUBLIC Eigen3::Eigen ${NLOPT_LIBRARY})
target_compile_options(tailsitter_planning PRIVATE -Wall -Wextra -Wpedantic)

add_executable(generate_simple_trajectory apps/generate_simple_trajectory.cpp)
target_link_libraries(generate_simple_trajectory PRIVATE tailsitter_planning)

add_executable(generate_stage_b_trajectory apps/generate_stage_b_trajectory.cpp)
target_link_libraries(generate_stage_b_trajectory PRIVATE tailsitter_planning)

add_executable(generate_joint_hover_trajectory apps/generate_joint_hover_trajectory.cpp)
target_link_libraries(generate_joint_hover_trajectory PRIVATE tailsitter_planning)

add_executable(generate_stage_c_trajectory apps/generate_stage_c_trajectory.cpp)
target_link_libraries(generate_stage_c_trajectory PRIVATE tailsitter_planning)

enable_testing()
add_executable(test_stage_a tests/test_stage_a.cpp)
target_link_libraries(test_stage_a PRIVATE tailsitter_planning)
add_test(NAME stage_a COMMAND test_stage_a)

add_executable(test_stage_b tests/test_stage_b.cpp)
target_link_libraries(test_stage_b PRIVATE tailsitter_planning)
add_test(NAME stage_b COMMAND test_stage_b)

add_executable(test_joint_optimizer tests/test_joint_optimizer.cpp)
target_link_libraries(test_joint_optimizer PRIVATE tailsitter_planning)
add_test(NAME joint_optimizer COMMAND test_joint_optimizer)

add_executable(test_stage_c tests/test_stage_c.cpp)
target_link_libraries(test_stage_c PRIVATE tailsitter_planning)
add_test(NAME stage_c COMMAND test_stage_c)
