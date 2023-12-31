# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)
project(Supereight2 VERSION 0.11.0 LANGUAGES C CXX)

option(SE_OPENMP "Compile supereight with OpenMP" ON)
option(SE_TEST "Compile the supereight tests" ON)
option(SE_APP "Compile the supereight application" ON)

# Find dependencies
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(octomap REQUIRED)
find_package(ompl REQUIRED)

# Compiler options
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unknown-pragmas
    -Wno-unused-function
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


#set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")

# Compile the dependencies
add_subdirectory(third_party)

# Compile the supereight library
set(LIB_NAME "supereight2")
add_library(${LIB_NAME} STATIC
    "src/common/colour_utils.cpp"
    "src/common/image_utils.cpp"
    "src/common/perfstats.cpp"
    "src/common/str_utils.cpp"
    "src/common/yaml.cpp"
    "src/map/data.cpp"
    "src/map/map.cpp"
    "src/map/octant.cpp"
    "src/map/preprocessor.cpp"
    "src/map/raycaster.cpp"
    "src/sensor/leica_lidar.cpp"
    "src/planning/state.cpp"
    "src/sensor/ouster_lidar.cpp"
    "src/sensor/pinhole_camera.cpp"
    "src/sensor/sensor.cpp"
    "src/tracker/tracker.cpp"
)
# Set the C++ standard required by the library.
target_compile_features(${LIB_NAME} PUBLIC cxx_std_17)
target_include_directories(${LIB_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${OpenCV_INCLUDE_DIRS}
      ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME}
    PUBLIC
      m
      stdc++fs
      Eigen3::Eigen
      ${OpenCV_LIBS}
      ${OMPL_LIBRARIES}
      SRL::Projection
      #SRL::Utils
      #SRL::sensors_realsense
)
# Find OpenMP, warn if disabled.
if(SE_OPENMP)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        target_compile_options(${LIB_NAME} PUBLIC ${OpenMP_CXX_FLAGS})
        target_link_libraries(${LIB_NAME} PUBLIC ${OpenMP_CXX_LIBRARIES})
        message(STATUS "Compiling with OpenMP support")
    else()
        message(WARNING "OpenMP not found. Performance may be terrible.")
    endif()
else()
    message(WARNING "Building without OpenMP. Performance may be terrible.")
endif()

if(DEFINED OCTOMAP_INCLUDE_DIRS)
    target_include_directories(${LIB_NAME} PUBLIC ${OCTOMAP_INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} PUBLIC ${OCTOMAP_LIBRARIES})
    target_compile_definitions(${LIB_NAME} PUBLIC "-DSE_OCTOMAP")
    message(STATUS "Building with OctoMap support")
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # Link against the GNU C++ library even when using clang.
    target_link_libraries(${LIB_NAME} PUBLIC stdc++)
endif()
# Add an alias so that the library can be used inside the build tree, e.g. when testing.
add_library(SRL::${PROJECT_NAME} ALIAS ${LIB_NAME})
# This is required so that the exported target has the name Supereight2 and not supereight.
set_target_properties(${LIB_NAME} PROPERTIES EXPORT_NAME ${PROJECT_NAME})

# Compile the tests
if(SE_TEST)
    # enable_testing() must be called in the root CMakeLists.txt.
    enable_testing()
    add_subdirectory(test)
endif()

# Compile the app
if(SE_APP)
    add_subdirectory(app)
endif()

# Install the library
set(PACKAGE_NAME ${PROJECT_NAME})
string(TOUPPER ${PACKAGE_NAME} PACKAGE_NAME_UC)
# The directory where the .cmake files will be installed
include(GNUInstallDirs)
if(WIN32 AND NOT CYGWIN)
    set(INSTALL_CMAKEDIR "${PACKAGE_NAME}/cmake")
elseif(APPLE)
    set(INSTALL_CMAKEDIR "${PACKAGE_NAME}.framework/Resources/CMake")
else()
    set(INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PACKAGE_NAME}")
endif()
# Generate *Config.cmake and *ConfigVersion.cmake files
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/${PACKAGE_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
    INSTALL_DESTINATION ${INSTALL_CMAKEDIR})
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
    COMPATIBILITY SameMajorVersion)
# Install files
install(DIRECTORY include/se DESTINATION include)
install(TARGETS ${LIB_NAME} EXPORT ${PACKAGE_NAME}-targets DESTINATION lib)
install(EXPORT ${PACKAGE_NAME}-targets
    FILE ${PACKAGE_NAME}Targets.cmake
    NAMESPACE SRL::
    DESTINATION ${INSTALL_CMAKEDIR})
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
    DESTINATION ${INSTALL_CMAKEDIR})

# Create the uninstall target
# https://gitlab.kitware.com/cmake/community/-/wikis/FAQ#can-i-do-make-uninstall-with-cmake
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake)
endif()

