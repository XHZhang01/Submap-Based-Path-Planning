cmake_minimum_required(VERSION 3.5)
project(okvis_multisensor_processing)

# generate the configure header file
configure_file(../cmake/okvisConfig.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/okvisConfig.hpp" @ONLY)
message(STATUS ${CMAKE_CURRENT_BINARY_DIR}/okvisConfig.hpp)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR}) 

set(LIB_SRC
  src/DatasetReader.cpp
  src/DatasetWriter.cpp
  src/LeicaDatasetReader.cpp
  src/RpgDatasetReader.cpp
  src/SubmappingInterface.cpp
  src/ThreadedSlam.cpp
  src/TrajectoryOutput.cpp
  src/ViVisualizer.cpp
)
if(HAVE_LIBREALSENSE)
  set(LIB_SRC ${LIB_SRC} src/Realsense.cpp src/RealsenseRgbd.cpp)
endif()
add_library(${PROJECT_NAME} ${LIB_SRC})
target_link_libraries(${PROJECT_NAME}
  ${GLOG_LIBRARIES}
  okvis_ceres
  okvis_common
  okvis_cv
  okvis_frontend
  okvis_kinematics
  okvis_mapping
  okvis_time
  okvis_timing
  okvis_util
)
if(HAVE_LIBREALSENSE)
  target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
endif()

# export config
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER ${CMAKE_CURRENT_BINARY_DIR}/okvisConfig.hpp)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/okvis" COMPONENT dev
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")

# testing
if(BUILD_TESTS)
  # require OpenCV
  find_package( OpenCV REQUIRED core imgproc features2d )
  include_directories(BEFORE ${OpenCV_INCLUDE_DIRS}) 
  if(APPLE)
    add_definitions(-DGTEST_HAS_TR1_TUPLE=1)
  else()
    add_definitions(-DGTEST_HAS_TR1_TUPLE=1)
    enable_testing()
    set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)
    add_executable(${PROJECT_TEST_NAME}
      test/test_main.cpp
    )
    target_link_libraries(${PROJECT_TEST_NAME} 
      ${GTEST_LIBRARY}
      ${GMOCK_LIBRARY}  
      ${PROJECT_NAME}
      ${CERES_LIBRARIES}
      ${OpenCV_LIBS}
      pthread)
    add_test(test ${PROJECT_TEST_NAME} )
  endif(APPLE)
endif()

