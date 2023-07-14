cmake_minimum_required(VERSION 3.5)
project(okvis_ceres)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR})
               
# build the library 
add_library(${PROJECT_NAME} 
  src/PoseParameterBlock.cpp
  src/SpeedAndBiasParameterBlock.cpp
  src/HomogeneousPointParameterBlock.cpp
  src/HomogeneousPointLocalParameterization.cpp
  src/PoseLocalParameterization.cpp
  src/ImuError.cpp
  src/PoseError.cpp
  src/RelativePoseError.cpp
  src/SpeedAndBiasError.cpp
  src/TwoPoseGraphError.cpp
  src/HomogeneousPointError.cpp
  src/ViSlamBackend.cpp
  src/ViGraph.cpp
  src/ViGraphEstimator.cpp
  src/ErrorInterface.cpp
  src/CeresIterationCallback.cpp
  include/okvis/ViSlamBackend.hpp
  include/okvis/ViGraph.hpp
  include/okvis/ViGraphEstimator.hpp
  include/okvis/ceres/ParameterBlock.hpp
  include/okvis/ceres/ParameterBlockSized.hpp
  include/okvis/ceres/PoseParameterBlock.hpp
  include/okvis/ceres/PoseLocalParameterization.hpp
  include/okvis/ceres/HomogeneousPointParameterBlock.hpp
  include/okvis/ceres/HomogeneousPointLocalParameterization.hpp
  include/okvis/ceres/HomogeneousPointError.hpp
  include/okvis/ceres/SpeedAndBiasParameterBlock.hpp
  include/okvis/ceres/SpeedAndBiasError.hpp
  include/okvis/ceres/ErrorInterface.hpp
  include/okvis/ceres/ImuError.hpp
  include/okvis/ceres/PoseError.hpp
  include/okvis/ceres/RelativePoseError.hpp
  include/okvis/ceres/ReprojectionError.hpp
  include/okvis/PseudoInverse.hpp
  include/okvis/ceres/TwoPoseGraphError.hpp
  include/okvis/ceres/implementation/ReprojectionError.hpp
  include/okvis/ceres/ReprojectionErrorBase.hpp
  include/okvis/ceres/CeresIterationCallback.hpp
  include/okvis/ceres/ode/ode.hpp
  # GPS error implementations
  src/GpsErrorSynchronous.cpp
  include/okvis/ceres/GpsErrorSynchronous.hpp
  src/GpsErrorAsynchronous.cpp
  include/okvis/ceres/GpsErrorAsynchronous.hpp
  # Submap Alignment error Implementations
  src/SubmapIcpError.cpp
  include/okvis/ceres/SubmapIcpError.hpp
)

# and link it
target_link_libraries(${PROJECT_NAME}
  okvis_util
  okvis_cv
  okvis_common
  okvis_timing
  ${CERES_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  SRL::Supereight2
  SRL::Projection
)
# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")

# testing
if(BUILD_TESTS)
  if(APPLE)
    add_definitions(-DGTEST_HAS_TR1_TUPLE=1)
  else()
    add_definitions(-DGTEST_HAS_TR1_TUPLE=0)
  endif(APPLE)
  enable_testing()
  set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)
  add_executable(${PROJECT_TEST_NAME}
    test/test_main.cpp
    #test/TestPoseGraphError.cpp
    #test/TestPoseError.cpp
    #test/TestHomogeneousPointError.cpp
    #test/TestReprojectionError.cpp
    #test/TestImuError.cpp
    #test/TestGpsError.cpp
    #test/TestEstimatorGpsError.cpp
    test/TestSubmapIcpError.cpp
  )
  target_link_libraries(${PROJECT_TEST_NAME} 
    ${PROJECT_NAME} 
    ${GTEST_LIBRARY}
    pthread
    ${PCL_LIBRARIES})
  add_test(test ${PROJECT_TEST_NAME})
endif()
