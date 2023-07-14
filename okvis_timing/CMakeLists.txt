cmake_minimum_required(VERSION 3.5)
project(okvis_timing)

add_library(${PROJECT_NAME}
  src/Timer.cpp
  src/NsecTimeUtilities.cpp
  include/okvis/timing/Timer.hpp
  include/okvis/timing/NsecTimeUtilities.hpp
)

target_link_libraries(${PROJECT_NAME} okvis_util)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
