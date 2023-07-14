cmake_minimum_required(VERSION 3.5)
project(okvis_frontend)  

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR}) 

# require OpenCV
find_package( OpenCV REQUIRED core imgproc features2d )
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

# build the library
add_library(${PROJECT_NAME} 
  src/Frontend.cpp
  src/stereo_triangulation.cpp
  src/FrameNoncentralAbsoluteAdapter.cpp
  src/LoopclosureNoncentralAbsoluteAdapter.cpp
  src/FrameRelativeAdapter.cpp
  src/FBrisk.cpp
  include/DBoW2/FBrisk.hpp
  include/okvis/Frontend.hpp
  include/okvis/triangulation/stereo_triangulation.hpp
  include/opengv/absolute_pose/FrameNoncentralAbsoluteAdapter.hpp
  include/opengv/absolute_pose/LoopclosureNoncentralAbsoluteAdapter.hpp
  include/opengv/relative_pose/FrameRelativeAdapter.hpp
  include/opengv/sac_problems/absolute_pose/FrameAbsolutePoseSacProblem.hpp
  include/opengv/sac_problems/relative_pose/FrameRelativePoseSacProblem.hpp
  include/opengv/sac_problems/relative_pose/FrameRotationOnlySacProblem.hpp)

# and link it        
target_link_libraries(${PROJECT_NAME}
  ${BRISK_LIBRARIES} 
  ${OpenGV_LIBRARIES} 
  ${CERES_LIBRARIES}
  ${OpenCV_LIBS}
  #-lopencv_calib3d # for some crazy reason, calib3d is not properly added unless specified manually
  okvis_util
  okvis_cv 
  okvis_ceres 
  okvis_timing 
  ${DBoW2_LIBRARIES}
)

# the DBoW2 pre-built vocabulary
add_custom_command(
  TARGET okvis_frontend POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          ${PROJECT_SOURCE_DIR}/../resources/small_voc.yml.gz
          ${CMAKE_CURRENT_BINARY_DIR}/../small_voc.yml.gz)

# and the network
add_custom_command(
  TARGET okvis_frontend POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          ${PROJECT_SOURCE_DIR}/../resources/fast-scnn.pt
          ${CMAKE_CURRENT_BINARY_DIR}/../fast-scnn.pt)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
