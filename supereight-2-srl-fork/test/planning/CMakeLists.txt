# SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)

add_executable(planning-unittest "planning_unittest.cpp")
target_link_libraries(planning-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME planning-unittest COMMAND planning-unittest)

add_executable(collision-checker-unittest "collision_checker_unittest.cpp")
target_link_libraries(collision-checker-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME collision-checker-unittest COMMAND collision-checker-unittest)
