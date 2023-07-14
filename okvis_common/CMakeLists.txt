cmake_minimum_required(VERSION 3.5)
project(okvis_common)

IF(libvisensor_FOUND)
  add_definitions(-DHAVE_LIBVISENSOR)
  MESSAGE(STATUS "Found libvisensor. Setting HAVE_LIBVISENSOR flag.")
ENDIF()

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR}) 

# require OpenCV
find_package( OpenCV REQUIRED core imgproc features2d)
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

# build the library
if(librealsense_FOUND)
add_library(${PROJECT_NAME} STATIC 
  src/ViInterface.cpp
  src/ViParametersReader.cpp
  include/okvis/FrameTypedefs.hpp
  include/okvis/Measurements.hpp
  include/okvis/Parameters.hpp
  include/okvis/ViFrontendInterface.hpp
  include/okvis/ViInterface.hpp
  include/okvis/ViParametersReader.hpp
  )
else()
add_library(${PROJECT_NAME} STATIC
  src/ViInterface.cpp
  src/ViParametersReader.cpp
  include/okvis/FrameTypedefs.hpp
  include/okvis/Measurements.hpp
  include/okvis/Parameters.hpp
  include/okvis/ViFrontendInterface.hpp
  include/okvis/ViInterface.hpp
  include/okvis/ViParametersReader.hpp)
endif()

if(${librealsense_FOUND})
  target_link_libraries(${PROJECT_NAME}
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv
    ${librealsense_LIBRARIES}
  )
else()
  # and link it                     
  target_link_libraries(${PROJECT_NAME}
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv
  )
endif()

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
