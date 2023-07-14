# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)
find_package(GLUT)
find_package(OpenGL)

# glog
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
include_directories(${glog_INCLUDE_DIRS})

# Realsense
find_package(realsense2 2.4 REQUIRED)

# Compile the reader as a library so that it isn't recompiled for each app.
set(LIB_NAME reader)
add_library(${LIB_NAME} STATIC
  "src/config.cpp"
  "src/reader.cpp"
  "src/reader_base.cpp"
  "src/reader_interiornet.cpp"
  "src/reader_newercollege.cpp"
  "src/reader_openni.cpp"
  "src/reader_raw.cpp"
  "src/reader_tum.cpp"
  "src/reader_leica.cpp"
  #"src/reader_realsense.cpp"
)
target_include_directories(${LIB_NAME} PRIVATE include)
target_link_libraries(${LIB_NAME} PRIVATE SRL::Supereight2
        #SRL::Utils
        #SRL::sensors_realsense
        )

# Compile the apps for each configuration in SE_APP_CONFIG.
set(SE_APP_CONFIG
  "TSDF Single PinholeCamera"
  "TSDF Single OusterLidar"
  "TSDF Multi PinholeCamera"
  "TSDF Multi OusterLidar"
  "TSDF Multi LeicaLidar"
  "Occupancy Multi PinholeCamera"
  "Occupancy Multi OusterLidar"
  "Occupancy Multi LeicaLidar"
)
foreach(APP_CONFIG ${SE_APP_CONFIG})
  # Replace spaces with semicolons to convert the string to a CMake list.
  string(REPLACE " " ";" APP_CONFIG ${APP_CONFIG})
  # Get the field, resolution and sensor type from the list.
  list(GET APP_CONFIG 0 FIELD)
  list(GET APP_CONFIG 1 RES)
  list(GET APP_CONFIG 2 SENSOR)
  # Convert to lower case.
  string(TOLOWER ${FIELD} FIELD_LC)
  string(TOLOWER ${RES} RES_LC)
  string(TOLOWER ${SENSOR} SENSOR_LC)

  # Add a custom command that replaces the field, resolution and sensor in
  # main.cpp using sed. The generated file is then used as the main of the
  # app.
  set(INPUT_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
  set(GENERATED_MAIN "${CMAKE_CURRENT_BINARY_DIR}/main_${FIELD_LC}_${RES_LC}_${SENSOR_LC}.cpp")
  add_custom_command(
    OUTPUT  ${GENERATED_MAIN}
    COMMAND sed -e "s/se::TSDF/se::${FIELD}/g" -e "s/se::Res::Single/se::Res::${RES}/g" -e "s/se::PinholeCamera/se::${SENSOR}/g" ${INPUT_MAIN} > ${GENERATED_MAIN}
    DEPENDS ${INPUT_MAIN}
  )

  # Compile the app using the generated file.
  set(EXE_NAME "supereight_${FIELD_LC}_${RES_LC}_${SENSOR_LC}")
  add_executable(${EXE_NAME} ${GENERATED_MAIN} "src/montage.cpp")
  target_include_directories(${EXE_NAME} BEFORE PRIVATE include)
  target_link_libraries(${EXE_NAME} PRIVATE SRL::Supereight2 ${LIB_NAME} ${realsense2_LIBRARY} ${glog_LIBRARIES})
  # Allow handling large files in 32-bit systems
  target_compile_definitions(${EXE_NAME} PRIVATE _FILE_OFFSET_BITS=64)

  # Compile with GUI support
  if(GLUT_FOUND)
    target_link_libraries(${EXE_NAME}
      PRIVATE
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
    )
    target_compile_definitions(${EXE_NAME} PRIVATE "-DSE_GLUT")
    if(NOT APPLE)
      target_link_libraries(${EXE_NAME} PRIVATE GL)
    endif()
  endif()
endforeach()

# build ray interface
add_executable(supereight_occupancy_multi_leicalidar_ray src/main_rayInterface.cpp)
target_include_directories(supereight_occupancy_multi_leicalidar_ray BEFORE PRIVATE include)
target_link_libraries(supereight_occupancy_multi_leicalidar_ray PRIVATE SRL::Supereight2 ${LIB_NAME} ${realsense2_LIBRARY} ${glog_LIBRARIES} ${Boost_LIBRARIES})
# Allow handling large files in 32-bit systems
target_compile_definitions(supereight_occupancy_multi_leicalidar_ray PRIVATE _FILE_OFFSET_BITS=64)

# build ray interface
add_executable(rayBatch_test src/main_rayBatchInterface.cpp)
target_include_directories(rayBatch_test BEFORE PRIVATE include)
target_link_libraries(rayBatch_test PRIVATE SRL::Supereight2 ${LIB_NAME} ${glog_LIBRARIES} ${Boost_LIBRARIES})
# Allow handling large files in 32-bit systems
target_compile_definitions(rayBatch_test PRIVATE _FILE_OFFSET_BITS=64)


# build realsense app
#add_executable(supereight_occupancy_multi_realsense src/main_realsense.cpp src/montage.cpp)
#target_include_directories(supereight_occupancy_multi_realsense BEFORE PRIVATE include)
#target_link_libraries(supereight_occupancy_multi_realsense PRIVATE SRL::Supereight2 ${LIB_NAME} SRL::sensors_realsense SRL::Utils ${realsense2_LIBRARY} ${glog_LIBRARIES})
## Allow handling large files in 32-bit systems
#target_compile_definitions(supereight_occupancy_multi_realsense PRIVATE _FILE_OFFSET_BITS=64)