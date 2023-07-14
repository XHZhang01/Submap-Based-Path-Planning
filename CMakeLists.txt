cmake_minimum_required(VERSION 3.5)
cmake_policy(VERSION 3.5)

project(okvis)
# The version number.
set(OKVIS_MAJOR_VERSION 2)
set(OKVIS_MINOR_VERSION 0)
set(OKVIS_PATCH_VERSION 0)
set(OKVIS_VERSION
  ${OKVIS_MAJOR_VERSION}.${OKVIS_MINOR_VERSION}.${OKVIS_PATCH_VERSION})

# options for build configuration
option (USE_SYSTEM_BRISK
        "Use brisk via find_package rather than downloading it as part of okvis" OFF)
option (USE_SYSTEM_DBOW2
         "Use DBoW2 via find_package rather than downloading it as part of okvis" OFF)
option (USE_SYSTEM_CERES
        "Use ceres via find_package rather than downloading it as part of okvis" OFF)
option (BUILD_APPS
        "Builds a demo app (which require boost and OpenCV highgui)" ON)
option (BUILD_TESTS
        "Builds all gtests" OFF)
option (HAVE_LIBREALSENSE
        "Use realsense as part of okvis" ON)
option (USE_NN
        "Use keypoint classification as part of okvis, requires torch" ON)
option (USE_GPU
        "Use keypoint classification with GPU inference, requires torch with Cuda" OFF)
option (DO_TIMING
        "Enable timing" ON)
option (EXTERNAL_PROJECTS_INHERIT_BUILD_TYPE
        "External builds inheriting build type, e.g. Debug" OFF)
option (LIDAR_MAPPING
        "Enabling LiDAR Mapping Backend" ON)

cmake_host_system_information(RESULT N_CORES QUERY NUMBER_OF_PHYSICAL_CORES)
message(STATUS "CMake ${CMAKE_VERSION} using ${N_CORES} threads")
add_definitions(-DOKVIS_N_CORES=${N_CORES})

# CXX version
# default to C++14.
if (NOT DEFAULT_CXX_STANDARD)
  set(DEFAULT_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD ${DEFAULT_CXX_STANDARD} CACHE STRING
  "C++ standard (minimum 14)" FORCE)
message(STATUS "Using C++ standard ${CMAKE_CXX_STANDARD}")

# avoid visibility warnings
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# additional definitions
if(APPLE)
  # The clang compiler (on osx) is somehow much more strict
  # than the compilers on ubuntu and so this does not seem
  # possible on OSX just yet.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC")
  set(CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS "${CMAKE_CXX_FLAGS} -Wall -fPIC")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -Wno-deprecated-copy-with-user-provided-copy -Wno-deprecated-copy -Werror") # use for code cleaning
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC")
  set(CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS "${CMAKE_CXX_FLAGS} -Wall -fPIC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -Wextra")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -Wextra -Werror") # use for code cleaning
endif()
#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

# Find OpenMP
find_package(OpenMP)
if(${OpenMP_FOUND})
  add_definitions(-DUSE_OPENMP)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS_CERES "${CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS} ${OpenMP_CXX_FLAGS}")
else()
  message(WARNING "OpenMP not found. Performance might be terrible.")
endif()

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/okvis)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# make sure we use Release and warn otherwise
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(WARNING "CMAKE_BUILD_TYPE not set to 'Release'. Performance may be terrible.")
else ()
  message(STATUS "Building with build type '${CMAKE_BUILD_TYPE}'")
endif ()
if (EXTERNAL_PROJECTS_INHERIT_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE_EXTERNAL_PROJECTS ${CMAKE_BUILD_TYPE})
else ()
    set(CMAKE_BUILD_TYPE_EXTERNAL_PROJECTS Release)
endif ()

# handle network:
if(USE_NN)
  add_definitions(-DOKVIS_USE_NN)
  if(USE_GPU)
    add_definitions(-DOKVIS_USE_GPU)
  endif()
else()
  message(WARNING "Network keypoint classification deactivated")
endif()

# these shoud only be needed by the project:
if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm*")
  message(STATUS "ARM processor detected, will attempt to use NEON.")
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__ARM_NEON__")
  add_definitions(-D__ARM_NEON__ -flax-vector-conversions) # for BRISK
else()
  message(STATUS "Assuming SSE instructions available.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
endif()

# could manually switch on timing here
if(NOT DO_TIMING)
  add_definitions(-DDEACTIVATE_TIMERS)
  message(STATUS "Deactivating timers.")
endif()

get_directory_property(defs COMPILE_DEFINITIONS)

# EXTERNAL DEPENDENCIES
include(ExternalProject)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include_directories(BEFORE ${CMAKE_BINARY_DIR}/include)
link_directories(BEFORE ${CMAKE_BINARY_DIR}/lib)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR})

if(USE_NN)
  # require Torch
  find_package(Torch REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
  add_definitions(-DC10_USE_GLOG)
endif()

# require GeographicLib
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

# require DBoW2
if(${USE_SYSTEM_DBOW2})
  find_package( DBoW2 REQUIRED )
  include_directories(BEFORE ${DBoW2_INCLUDE_DIRS})
  message(STATUS "Using system DBow2. Found at ${DBoW2_INCLUDE_DIRS}.")
else()
  option(BUILD_Demo OFF)
  ExternalProject_Add(DBoW2_external
      GIT_REPOSITORY https://github.com/dorian3d/DBoW2.git
      UPDATE_COMMAND ""
      INSTALL_DIR ${CMAKE_BINARY_DIR}
      CMAKE_ARGS
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE_EXTERNAL_PROJECTS}
        -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS}
      PREFIX ${CMAKE_CURRENT_BINARY_DIR}/DBoW2
      BUILD_IN_SOURCE 0
      BUILD_COMMAND make -j${N_CORES}
      INSTALL_COMMAND make install
    )
  add_library(DBoW2 SHARED IMPORTED)
  add_dependencies(DBoW2 DBoW2_external)
  set(DBoW2_LIBRARIES DBoW2)
  if(APPLE)
    set_target_properties(DBoW2 PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libDBoW2.dylib)
    # install it once built
    install(FILES ${CMAKE_BINARY_DIR}/DBoW2/lib/libDBoW2.dylib
      DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
    install(DIRECTORY ${CMAKE_BINARY_DIR}/include/DBoW2/
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include/DBoW2/)
  else()
   set_target_properties(DBoW2 PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libDBoW2.so)
   # install it once built
   install(FILES ${CMAKE_BINARY_DIR}/DBoW2/lib/libDBoW2.so
     DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
   install(DIRECTORY ${CMAKE_BINARY_DIR}/include/DBoW2/
     DESTINATION ${CMAKE_INSTALL_PREFIX}/include/DBoW2/)
  endif()
endif()

# require OpenCV
if(BUILD_APPS)
  # add highgui only here
  find_package( OpenCV REQUIRED core highgui imgproc imgcodecs features2d)
else()
  find_package( OpenCV REQUIRED core imgproc imgcodecs features2d)
endif()
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

# This is optional until the first release of librealsense as a ROS package
if(${HAVE_LIBREALSENSE})
  message(STATUS "With librealsense2")
  FIND_PACKAGE(realsense2 2.4 REQUIRED)
  SET(CAMERA_DRIVER "realsense")
endif()

# glog
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/include/glog")
find_package(Glog REQUIRED)
include_directories(BEFORE ${GLOG_INCLUDE_DIRS})

# brisk
if(${USE_SYSTEM_BRISK})
  find_package( brisk 2 REQUIRED )
  include_directories(BEFORE ${BRISK_INCLUDE_DIRS})
  message(STATUS "Using system brisk. Found at ${BRISK_INCLUDE_DIRS}.")
else()
  ExternalProject_Add(brisk_external
    GIT_REPOSITORY git@bitbucket.org:smartroboticslab/brisk.git
    GIT_TAG task/cameraaware
    UPDATE_COMMAND ""
    INSTALL_DIR ${CMAKE_BINARY_DIR}
    CMAKE_ARGS
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE_EXTERNAL_PROJECTS}
      -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS}
      -DBRISK_BUILD_DEMO=OFF
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/brisk
    BUILD_IN_SOURCE 0
    BUILD_COMMAND make -j${N_CORES}
    INSTALL_COMMAND make install
  )
  add_library(brisk STATIC IMPORTED)
  add_library(agast STATIC IMPORTED)
  set(BRISK_LIBRARIES brisk agast)
  set_target_properties(brisk PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libbrisk.a)
  set_target_properties(agast PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libagast.a)
  add_dependencies(brisk brisk_external)
  add_dependencies(agast brisk_external)

  # install it once built
  install(FILES ${CMAKE_BINARY_DIR}/lib/libbrisk.a ${CMAKE_BINARY_DIR}/lib/libagast.a
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
  install(DIRECTORY ${CMAKE_BINARY_DIR}/include/brisk/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/brisk/)
  install(DIRECTORY ${CMAKE_BINARY_DIR}/lib/CMake/brisk/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/CMake/brisk/)
endif()

# ceres
find_package(CUDA QUIET)
if(${CUDA_FOUND})
  enable_language(CUDA)
endif()
find_package(SuiteSparse REQUIRED QUIET) #will be needed in any case...
find_package(CXSparse QUIET) #optional
if(${USE_SYSTEM_CERES})
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
  find_package( Ceres 2.1 REQUIRED )
  include_directories(BEFORE ${CERES_INCLUDE_DIRS})
  message(STATUS "Using system Ceres.")
else()
  set(CERES_VERSION 2.1.0)
  set(CERES_TAG 783637a6ed285abc6c57b16ccd968c594a91b35f) #version 2.1.0
  if(${OpenMP_FOUND})
    set(CERES_THREADS OPENMP)
  else()
    set(CERES_THREADS CXX_THREADS)
  endif()
  ExternalProject_Add(ceres_external
    GIT_REPOSITORY https://ceres-solver.googlesource.com/ceres-solver
    GIT_TAG ${CERES_TAG}
    UPDATE_COMMAND ""
    INSTALL_DIR ${CMAKE_BINARY_DIR}
    CMAKE_ARGS
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE_EXTERNAL_PROJECTS}
      -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS_CERES}
      -DBUILD_EXAMPLES:BOOL=OFF
      -DBUILD_TESTING:BOOL=OFF
      -DCERES_THREADING_MODEL:STRING=${CERES_THREADS}
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ceres
    BUILD_IN_SOURCE 0
    BUILD_COMMAND make -j${N_CORES}
    INSTALL_COMMAND make install
  )
  add_library(ceres UNKNOWN IMPORTED)
  # Heuristic for determining LIB_SUFFIX from ceres
  set(CERES_LIB_PREFIX "lib")
  if (CMAKE_SYSTEM_NAME MATCHES "Linux" AND
      NOT DEFINED LIB_SUFFIX AND
      NOT CMAKE_CROSSCOMPILING AND
      CMAKE_SIZEOF_VOID_P EQUAL "8" AND
      NOT EXISTS "/etc/debian_version" AND
      NOT EXISTS "/etc/arch-release")
    set(CERES_LIB_PREFIX "lib64")
  endif ()

  # to find it from 3rd party software, since not added to registry:
  set(OKVIS_CERES_CONFIG "${CMAKE_BINARY_DIR}/share/Ceres/")
  set_property(TARGET ceres PROPERTY IMPORTED_LOCATION
               ${CMAKE_BINARY_DIR}/${CERES_LIB_PREFIX}/libceres.a )
  set_property(TARGET ceres PROPERTY CXX_VISIBILITY_PRESET hidden)
  set_property(TARGET ceres PROPERTY C_VISIBILITY_PRESET hidden)
  set_property(TARGET ceres PROPERTY VISIBILITY_INLINES_HIDDEN ON)

  #ceres won't export these
  set(CERES_LIBRARIES ceres 
    ${SUITESPARSE_LIBRARIES} ${CXSPARSE_LIBRARIES} ${Eigen_LIBS} ${GLOG_LIBRARY})
  if(${OpenMP_FOUND})
    set(CERES_LIBRARIES ${CERES_LIBRARIES} ${OpenMP_libomp_LIBRARY})
  endif()
  if(${CUDA_FOUND})
    set(CERES_LIBRARIES ${CERES_LIBRARIES}
        ${CUDA_LIBRARIES} ${CUDA_cusolver_LIBRARY} ${CUDA_cusparse_LIBRARY} ${CUDA_cublas_LIBRARY})
  endif()
  add_dependencies(ceres ceres_external)

  # install it once built
  install(FILES ${CMAKE_BINARY_DIR}/${CERES_LIB_PREFIX}/libceres.a
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
  install(DIRECTORY ${CMAKE_BINARY_DIR}/include/ceres/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ceres/)
  install(DIRECTORY ${CMAKE_BINARY_DIR}/share/Ceres/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/Ceres/)
  include_directories(BEFORE ${CMAKE_INSTALL_PREFIX}/include/)
endif()

# OpenGV
set(CMAKE_CXX_FLAGS_WSUPPRESS
      "${CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS} -Wno-unused-parameter -Wno-maybe-uninitialized -Wno-sign-compare -Wno-unused-but-set-variable -Wno-unused-variable -Wno-pedantic")
message(STATUS "NOTE: Suppressing some warnings when compiling OpenGV.")
ExternalProject_Add(opengv_external
    GIT_REPOSITORY https://github.com/laurentkneip/opengv
    GIT_TAG 306a54e6c6b94e2048f820cdf77ef5281d4b48ad
    UPDATE_COMMAND ""
    PATCH_COMMAND # forcing static libs
        COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/cmake/opengv/CMakeLists.txt ${CMAKE_CURRENT_BINARY_DIR}/opengv/src/opengv/CMakeLists.txt
        COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/cmake/opengv/opengvConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/opengv/src/opengv/opengvConfig.cmake.in
        COMMENT "Forcing our own CMakeLists.txt to build OpenGV (static library support)."
    CMAKE_ARGS
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE_EXTERNAL_PROJECTS}
      -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS} # suppress warnings...
      #-DBUILD_TESTS:BOOL=OFF
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/opengv
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opengv/src/opengv
    INSTALL_DIR ${CMAKE_BINARY_DIR}
    BUILD_IN_SOURCE 0
    BUILD_COMMAND make
    INSTALL_COMMAND make install
)
add_library(opengv STATIC IMPORTED)
set(OpenGV_LIBRARIES opengv)
set_target_properties(opengv PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libopengv.a)
add_dependencies(${OpenGV_LIBRARIES} opengv_external)

# install it once built
install(FILES ${CMAKE_BINARY_DIR}/lib/libopengv.a
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(DIRECTORY ${CMAKE_BINARY_DIR}/include/opengv/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/opengv/)
install(DIRECTORY ${CMAKE_BINARY_DIR}/lib/CMake/opengv/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/CMake/opengv/)

# the absolute path to the target template image
set(OKVIS_TARGET_IMAGE_PATH \"${PROJECT_SOURCE_DIR}/config\")
add_definitions(-DOKVIS_TARGET_IMAGE_PATH=${OKVIS_TARGET_IMAGE_PATH})

if(${BUILD_TESTS})
  # gmock
  # http://stackoverflow.com/questions/9689183/cmake-googletest
  ExternalProject_Add(
      googlemock
      URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip
      PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gmock
      CMAKE_ARGS
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS}
        -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DCMAKE_POLICY_DEFAULT_CMP0063=NEW
        -DCMAKE_C_VISIBILITY_PRESET=hidden
        -DCMAKE_CXX_VISIBILITY_PRESET=hidden
        -DCMAKE_VISIBILITY_INLINES_HIDDEN=ON
      BUILD_COMMAND make -j${N_CORES}
      INSTALL_COMMAND ""
  )
  ExternalProject_Get_Property(googlemock source_dir)

  set(GMOCK_INCLUDE_DIR ${source_dir}/googlemock/include)
  set(GTEST_INCLUDE_DIR ${source_dir}/googletest/include)
  include_directories(BEFORE ${GMOCK_INCLUDE_DIR})
  include_directories(BEFORE ${GTEST_INCLUDE_DIR})
  ExternalProject_Get_Property(googlemock binary_dir)
  message(STATUS ${binary_dir})
  set(GMOCK_LIBRARY_PATH ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gmock.a)
  set(GMOCK_LIBRARY gmock)
  add_library(${GMOCK_LIBRARY} UNKNOWN IMPORTED)
  set_property(TARGET ${GMOCK_LIBRARY} PROPERTY IMPORTED_LOCATION
                ${GMOCK_LIBRARY_PATH} )
  add_dependencies(${GMOCK_LIBRARY} googlemock)
  set(GTEST_LIBRARY_PATH ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a)
  set(GTEST_LIBRARY gtestlocal)
  add_library(${GTEST_LIBRARY} UNKNOWN IMPORTED)
  set_property(TARGET ${GTEST_LIBRARY} PROPERTY IMPORTED_LOCATION
                ${GTEST_LIBRARY_PATH})
  add_dependencies(${GTEST_LIBRARY} googlemock)
endif()

# BUILD LOCAL DEPENDENCIES
include_directories(okvis_util/include)
add_subdirectory(okvis_util)

include_directories(okvis_kinematics/include)
add_subdirectory(okvis_kinematics)
add_dependencies(okvis_kinematics okvis_util)

include_directories(okvis_time/include)
add_subdirectory(okvis_time)
add_dependencies(okvis_time okvis_util)

include_directories(okvis_cv/include)
add_subdirectory(okvis_cv)
add_dependencies(okvis_cv brisk okvis_util)

include_directories(okvis_common/include)
add_subdirectory(okvis_common)

include_directories(okvis_timing/include)
add_subdirectory(okvis_timing)

include_directories(okvis_mapping/include)
add_subdirectory(okvis_mapping)

include_directories(okvis_ceres/include)
add_subdirectory(okvis_ceres)
add_dependencies(okvis_ceres ceres okvis_util okvis_timing okvis_mapping DBoW2)

include_directories(okvis_frontend/include)
add_subdirectory(okvis_frontend)
add_dependencies(okvis_frontend opengv okvis_cv okvis_util okvis_timing)

# SE2 Stuff
option(SE_TEST "Compile the supereight tests" OFF)
option(SE_APP "Compile the supereight application" OFF)
add_subdirectory(supereight-2-srl-fork)

#if(LIDAR_MAPPING)
#  include_directories(okvis_mapping/include)
#  add_subdirectory(okvis_mapping)
#  add_dependencies(okvis_ceres okvis_mapping)
  #  add_dependencies(okvis_ceres ceres okvis_util okvis_timing DBoW2)
#endif()

include_directories(okvis_multisensor_processing/include)
add_subdirectory(okvis_multisensor_processing)
add_dependencies(okvis_multisensor_processing okvis_util okvis_timing okvis_mapping)



if(BUILD_APPS)

  include_directories(okvis_apps/include)
  FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS})
  if(${HAVE_LIBREALSENSE})
    add_executable(okvis_app_realsense
      okvis_apps/src/okvis_app_realsense.cpp
    )
    target_link_libraries(okvis_app_realsense
      okvis_util
      okvis_kinematics
      okvis_time
      okvis_cv
      okvis_common
      okvis_ceres
      okvis_timing
      okvis_frontend
      okvis_multisensor_processing
      pthread
      ${Boost_LIBRARIES}
      ${TORCH_LIBRARIES}
    )
  endif()
  if(${HAVE_LIBREALSENSE})
    add_executable(okvis_app_realsense_recorder
      okvis_apps/src/okvis_app_realsense_recorder.cpp
    )
    target_link_libraries(okvis_app_realsense_recorder
      okvis_util
      okvis_kinematics
      okvis_time
      okvis_cv
      okvis_common
      okvis_ceres
      okvis_timing
      okvis_frontend
      okvis_multisensor_processing
      pthread
      ${TORCH_LIBRARIES}
      ${Boost_LIBRARIES}
    )
  endif()

  # ----- gps test - begin
  add_executable(gps_conversion_geodetic_cartesian
    okvis_apps/src/gps_conversion_geodetic_cartesian.cpp)
  target_link_libraries(gps_conversion_geodetic_cartesian
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_timing
    okvis_cv
    pthread
    ${Boost_LIBRARIES}
    ${GeographicLib_LIBRARIES}
  )


  add_executable(okvis_app_synchronous
    okvis_apps/src/okvis_app_synchronous.cpp
  )
  target_link_libraries(okvis_app_synchronous
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv
    okvis_common
    okvis_ceres
    okvis_timing
    okvis_frontend
    okvis_mapping
    okvis_multisensor_processing
    pthread
    ${Boost_LIBRARIES}
    ${TORCH_LIBRARIES}
    ${GeographicLib_LIBRARIES}
  )


  add_executable(leicaDatasetReader_test
          okvis_apps/src/leicaDatasetReader_test.cpp
          )
  target_link_libraries(leicaDatasetReader_test
          okvis_util
          okvis_kinematics
          okvis_time
          okvis_cv
          okvis_common
          okvis_ceres
          okvis_timing
          okvis_frontend
          okvis_mapping
          okvis_multisensor_processing
          pthread
          ${Boost_LIBRARIES}
          ${TORCH_LIBRARIES}
          ${GeographicLib_LIBRARIES}
  )
  #set_property(TARGET okvis_app_synchronous PROPERTY CXX_STANDARD 14)

  add_executable(okvis_app_leica_submaps
          okvis_apps/src/okvis_app_leica_submaps.cpp
          )
  target_link_libraries(okvis_app_leica_submaps
          okvis_util
          okvis_kinematics
          okvis_time
          okvis_cv
          okvis_common
          okvis_ceres
          okvis_timing
          okvis_frontend
          #okvis_mapping
          okvis_multisensor_processing
          pthread
          ${Boost_LIBRARIES}
          ${TORCH_LIBRARIES}
          ${GeographicLib_LIBRARIES}
          SRL::Supereight2
          SRL::Projection
  )
  # Compile the reader as a library so that it isn't recompiled for each app.
  set(READER_LIB reader)
  add_library(${READER_LIB} STATIC
          "./supereight-2-srl-fork/app/src/config.cpp"
          "./supereight-2-srl-fork/app/src/reader.cpp"
          "./supereight-2-srl-fork/app/src/reader_base.cpp"
          "./supereight-2-srl-fork/app/src/reader_interiornet.cpp"
          "./supereight-2-srl-fork/app/src/reader_newercollege.cpp"
          "./supereight-2-srl-fork/app/src/reader_openni.cpp"
          "./supereight-2-srl-fork/app/src/reader_raw.cpp"
          "./supereight-2-srl-fork/app/src/reader_tum.cpp"
          "./supereight-2-srl-fork/app/src/reader_leica.cpp"
          #"./supereight-2-srl-fork/app/src/reader_realsense.cpp"
          )
  target_include_directories(${READER_LIB} PRIVATE ./supereight-2-srl-fork/app/include)
  target_link_libraries(${READER_LIB} PRIVATE SRL::Supereight2)

  add_executable(okvis_app_se2_test
          okvis_apps/src/okvis_app_se2_test.cpp
          )
  target_link_libraries(okvis_app_se2_test
          okvis_util
          okvis_kinematics
          okvis_time
          okvis_cv
          okvis_common
          okvis_ceres
          okvis_timing
          okvis_frontend
          #okvis_mapping
          okvis_multisensor_processing
          pthread
          ${Boost_LIBRARIES}
          ${TORCH_LIBRARIES}
          ${GeographicLib_LIBRARIES}
          ${READER_LIB}
          SRL::Supereight2
          SRL::Projection
  )

  add_executable(dbow2_test
    okvis_apps/src/dbow2_test.cpp
  )
  target_link_libraries(dbow2_test
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv
    okvis_common
    okvis_ceres
    okvis_timing
    okvis_frontend
    okvis_multisensor_processing
    pthread
    ${DBoW2_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  if(USE_NN)
    add_executable(nn_test
      okvis_apps/src/nn_test.cpp
    )

    target_link_libraries(nn_test
      ${TORCH_LIBRARIES}
      ${OpenCV_LIBS}
      ${GLOG_LIBRARIES}
      ${Boost_LIBRARIES}
    )
  endif()

  if(${HAVE_LIBREALSENSE})
    install(TARGETS 
      okvis_app_synchronous 
      dbow2_test okvis_app_realsense 
      okvis_app_realsense_recorder
      # IMPORTANT: Add the executable to the "export-set"
      EXPORT okvisTargets
      RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
  else()
    install(TARGETS 
      okvis_app_synchronous 
      dbow2_test
      # IMPORTANT: Add the executable to the "export-set"
      EXPORT okvisTargets
      RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
  endif()
endif()

# Include stereo from depth network.
if(${USE_NN})
  include_directories(okvis_stereo_depth/include)
  add_subdirectory(okvis_stereo_depth)
endif()

# installation is invoked in the individual modules...
export (TARGETS
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv
    okvis_common
    okvis_ceres
    okvis_timing
    okvis_frontend
    supereight2
    #okvis_mapping #ToDo: fix this linking / exporting issues
    #okvis_multisensor_processing
    FILE "${PROJECT_BINARY_DIR}/okvisTargets.cmake"
)
export ( PACKAGE okvis )

# Create the okvisConfig.cmake and okvisConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/okvis_util/include"
  "${PROJECT_SOURCE_DIR}/okvis_kinematics/include"
  "${PROJECT_SOURCE_DIR}/okvis_time/include"
  "${PROJECT_SOURCE_DIR}/okvis_cv/include"
  "${PROJECT_SOURCE_DIR}/okvis_common/include"
  "${PROJECT_SOURCE_DIR}/okvis_ceres/include"
  "${PROJECT_SOURCE_DIR}/okvis_timing/include"
  "${PROJECT_SOURCE_DIR}/okvis_frontend/include"
  "${PROJECT_SOURCE_DIR}/okvis_mapping/include"
  "${PROJECT_SOURCE_DIR}/okvis_apps/include"
  "${PROJECT_SOURCE_DIR}/okvis_multisensor_processing/include"
  "${PROJECT_BINARY_DIR}"
)
configure_file(cmake/okvisConfig.cmake.in
  "${PROJECT_BINARY_DIR}/okvisConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${OKVIS_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(cmake/okvisConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/okvisConfig.cmake" @ONLY)
# ... for both
configure_file(cmake/okvisConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/okvisConfigVersion.cmake" @ONLY)

# Install the okvisConfig.cmake and okvisConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/okvisConfig.cmake"
  "${PROJECT_BINARY_DIR}/okvisConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}")

# Install the export set for use with the install-tree
install(EXPORT okvisTargets DESTINATION
  "${INSTALL_CMAKE_DIR}")
