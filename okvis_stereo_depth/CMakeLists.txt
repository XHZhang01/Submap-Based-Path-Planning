cmake_minimum_required(VERSION 3.5)
project(okvis_stereo_depth)

find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
find_package(OpenCV COMPONENTS core imgproc features2d REQUIRED)
include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR} 
    ${Torch_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    include/okvis/Processor.hpp
    include/okvis/Stereo2DepthProcessor.hpp
    include/okvis/utils.hpp
    src/Stereo2DepthProcessor.cpp 
    src/Processor.cpp)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${TORCH_LIBRARIES}
    okvis_ceres
    okvis_common
    okvis_cv
    okvis_frontend
    okvis_kinematics
    okvis_multisensor_processing
    okvis_time
    okvis_timing
    okvis_util
)

