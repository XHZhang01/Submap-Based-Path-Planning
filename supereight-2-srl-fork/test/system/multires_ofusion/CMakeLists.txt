# SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)

# PCL Stuff
find_package(PCL)
pkg_check_modules(glog REQUIRED libglog)
include_directories(${glog_INCLUDE_DIRS})

set(READER_SRC
  ../../../app/src/config.cpp
  ../../../app/src/reader.cpp
  ../../../app/src/reader_base.cpp
  ../../../app/src/reader_interiornet.cpp
  ../../../app/src/reader_newercollege.cpp
  ../../../app/src/reader_openni.cpp
  ../../../app/src/reader_raw.cpp
  ../../../app/src/reader_tum.cpp
  ../../../app/src/reader_leica.cpp
  #../../../app/src/reader_realsense.cpp
)

get_filename_component(CONFIG_FILE "config.yaml" ABSOLUTE)

if(PCL_FOUND)
    # These have to be added globaly for compatibility with PCL 1.8.1 on Ubuntu 18.04.
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()


add_executable(multires-ofusion-systemtest multires_ofusion_systemtest.cpp ${READER_SRC})
target_include_directories(multires-ofusion-systemtest PRIVATE ../../../app/include)
target_link_libraries(multires-ofusion-systemtest PRIVATE SRL::Supereight2 gtest ${glog_LIBRARIES})

if(PCL_FOUND)
    # These have to be added globaly for compatibility with PCL 1.8.1 on Ubuntu 18.04.
    target_link_libraries(multires-ofusion-systemtest PRIVATE ${PCL_LIBRARIES})
    target_compile_definitions(multires-ofusion-systemtest PRIVATE "-DSE_PCL")
endif()

add_test(NAME multires-ofusion-systemtest COMMAND multires-ofusion-systemtest ${CONFIG_FILE})

