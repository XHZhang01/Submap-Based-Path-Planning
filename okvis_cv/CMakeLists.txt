cmake_minimum_required(VERSION 3.5)
project(okvis_cv)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR}) 

# require OpenCV
find_package( OpenCV REQUIRED COMPONENTS imgproc core features2d)
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

# build the library
add_library(${PROJECT_NAME} STATIC 
  include/okvis/cameras/PinholeCamera.hpp
  include/okvis/cameras/implementation/PinholeCamera.hpp
  include/okvis/cameras/DistortionBase.hpp
  include/okvis/cameras/NoDistortion.hpp
  include/okvis/cameras/RadialTangentialDistortion.hpp
  include/okvis/cameras/implementation/RadialTangentialDistortion.hpp
  include/okvis/cameras/EquidistantDistortion.hpp
  include/okvis/cameras/implementation/EquidistantDistortion.hpp
  include/okvis/cameras/RadialTangentialDistortion8.hpp
  include/okvis/cameras/implementation/RadialTangentialDistortion8.hpp
  include/okvis/cameras/EucmCamera.hpp
  include/okvis/cameras/implementation/EucmCamera.hpp
  include/okvis/cameras/CameraBase.hpp
  include/okvis/cameras/implementation/CameraBase.hpp
  include/okvis/cameras/NCameraSystem.hpp
  include/okvis/cameras/implementation/NCameraSystem.hpp
  include/okvis/Frame.hpp
  src/Frame.cpp
  include/okvis/MultiFrame.hpp
  include/okvis/implementation/Frame.hpp
  include/okvis/implementation/MultiFrame.hpp
  src/CameraBase.cpp
  src/NCameraSystem.cpp
)

# and link it
target_link_libraries(${PROJECT_NAME}  
  okvis_util
  okvis_kinematics 
  okvis_time
  ${OpenCV_LIBS}
  ${BRISK_LIBRARIES}
  ${TORCH_LIBRARIES}
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
    test/runTests.cpp
    test/TestPinholeCamera.cpp
    test/TestFrame.cpp
    test/TestNCameraSystem.cpp
    test/TestMultiFrame.cpp
    test/TestEucmCamera.cpp
  )
  target_link_libraries(${PROJECT_TEST_NAME} 
    ${PROJECT_NAME} 
    ${GTEST_LIBRARY}  
    ${OpenCV_LIBS}
    ${BRISK_LIBRARIES}
    ${TORCH_LIBRARIES}
    pthread)
  add_test(test ${PROJECT_TEST_NAME})
endif()
