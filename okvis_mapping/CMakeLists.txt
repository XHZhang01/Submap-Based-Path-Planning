cmake_minimum_required(VERSION 3.5)
project(okvis_mapping)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR})

#find_package( Supereight2 REQUIRED)

# build the library
add_library(${PROJECT_NAME}
        src/OusterMappingBackend.cpp
        include/okvis/OusterMappingBackend.hpp
        src/LidarMappingBackend.cpp
        include/okvis/LidarMappingBackend.hpp
        )

# and link it
target_link_libraries(${PROJECT_NAME}
        SRL::Supereight2
        SRL::Projection
        )

# installation if required ToDo
install(TARGETS ${PROJECT_NAME}
        EXPORT okvisTargets
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
        )
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")

