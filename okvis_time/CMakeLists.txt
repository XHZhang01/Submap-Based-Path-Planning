cmake_minimum_required(VERSION 3.5)
project(okvis_time)

IF(libvisensor_FOUND)
  add_definitions(-DHAVE_LIBVISENSOR)
  MESSAGE(STATUS "Found libvisensor. Setting HAVE_LIBVISENSOR flag.")
ENDIF()

# build the library
add_library(${PROJECT_NAME}
  include/okvis/Time.hpp
  include/okvis/Duration.hpp
  include/okvis/implementation/Time.hpp
  include/okvis/implementation/Duration.hpp
  src/Time.cpp
  src/Duration.cpp)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
