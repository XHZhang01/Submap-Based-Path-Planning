cmake_minimum_required(VERSION 3.5)
project(okvis_util)

# nothing to build for now...
add_library(${PROJECT_NAME} STATIC
    include/okvis/assert_macros.hpp
    include/okvis/source_file_pos.hpp
    include/okvis/MstGraph.hpp
    src/dependency-tracker.cc)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/okvis" COMPONENT dev
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
