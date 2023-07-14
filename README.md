README                        {#mainpage}
======

This is a fork of OKVIS2: Open Keyframe-based Visual-Inertial SLAM.

It adds the following features:

* Tightly-coupled GPS Integration

* EUCM Camera Model Support

* SE2-based Mapping Backend

* Application of submapping strategies based on visual keyframes

* WIP: submap alignment strategies

This is the Author's implementation of [1]. It is further based on work
presented in [2-4].

[1] Stefan Leutenegger. [OKVIS2: Realtime Scalable Visual-Inertial SLAM with 
    Loop Closure](https://arxiv.org/pdf/2202.09199). arXiv, 2022.

[2] Stefan Leutenegger, Simon Lynen, Michael Bosse, Roland Siegwart and Paul
    Timothy Furgale. Keyframe-based visual–inertial odometry using nonlinear
    optimization. The International Journal of Robotics Research, 2015.

[3] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for
    Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.

[4] Stefan Leutenegger, Paul Timothy Furgale, Vincent Rabaud, Margarita Chli,
    Kurt Konolige, Roland Siegwart. Keyframe-Based Visual-Inertial SLAM using
    Nonlinear Optimization. In Proceedings of Robotics: Science and Systems,
    2013.

Note that the codebase that you are provided here is free of charge and without
any warranty. This is bleeding edge research software.

Also note that the quaternion standard has been adapted to match Eigen/ROS,
thus some related mathematical description in [2-3] will not match the
implementation here.

If you publish work that relates to this software, please cite at least [1].

### License ###

The 3-clause BSD license (see file LICENSE) applies.

### How do I get set up? ###
#### Ubuntu ####

This is a pure cmake project.

You will need to install the following dependencies,

* CMake,

        sudo apt-get install cmake

* google-glog + gflags,

        sudo apt-get install libgoogle-glog-dev

* BLAS & LAPACK,

        sudo apt-get install libatlas-base-dev

* Eigen3,

        sudo apt-get install libeigen3-dev

* SuiteSparse and CXSparse,

        sudo apt-get install libsuitesparse-dev

* Boost,

        sudo apt-get install libboost-dev libboost-filesystem-dev

* OpenCV 2.4-4: follow the instructions on http://opencv.org/ or install
  via

        sudo apt-get install libopencv-dev

* LibTorch: if you want to run the segmentation CNN to remove Sky points etc,
  install with instructions from the link below. Get the C++ version with C++11
  ABI with or without CUDA (depending on availability on your machine):

    https://pytorch.org/get-started/locally/

    Also, depending on where you downloaded it to, you may want to tell cmake in
    your ~.bashrc:

        export Torch_DIR=/path/to/libtorch

    In case you absolutely do not want to use LibTorch, you may disable with
    USE_NN=OFF.

* Optional: you can use this package with a Realsense D435i or D455.
Follow the instructions on:
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

In addition to the previously mentioned packages you also need to install

* GeographicLIB
* PCL (+VTK)



### Building the project ###

Clone the OKVIS2-Leica repository:

    git clone git@bitbucket.org:bochsim/okvis2-leica.git


To change the cmake build type for the whole project use:

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j8

### Running the demo application ###

The demo example uses a dataset in the Leica-TUM format (based on ASL/ETH format) which can be obtained extracting recorded data from a BLK2Fly

To run the full pipeline using OKVIS2 + Submapping + Ray-based SE2

You will find a demo application in okvis_apps. It can process datasets in the
ASL/ETH format.

In order to run a minimal working example, follow the steps below:

        env OMP_NUM_THREADS=3 ./okvis_app_leica_submaps ../../config/config_leica_eucm.yaml /srv/Users/Boche/Documents/Datasets/Leica/20220209_kemptthal_flight03/ ../../config/config_submapping.yaml
        
1. env OMP_NUM_THREADS=3: limits the maximum number of OMP threads (required to have mapping at a reasonable time).
2. ./okvis_app_leica_submaps: Program call
3. ../../config/config_leica_eucm_gps.yaml: OKVIS2 (Visual-Inertial SLAM) config file
4. /srv/Users/Boche/Documents/Datasets/Leica/20220209_kemptthal_flight03/: path to dataset to be processed.
5. ../../config/config_submapping.yaml: config file for mapping backend (mostly following logic of original SE2).

### Outputs and frames

In terms of coordinate frames and notation,

* W denotes the OKVIS World frame (z up),
* C\_i denotes the i-th camera frame
* S denotes the IMU sensor frame
* B denotes a (user-specified) body frame.

The output of the okvis library is the pose T\_WS as a position r\_WS and quaternion
q\_WS, followed by the velocity in World frame v\_W and gyro biases (b_g) as well as
accelerometer biases (b_a). See the example application to get an idea on how to
use the estimator and its outputs (callbacks returning states).

Furthermore, for every created submap a mesh file is exported (*.vtk format; can be viewed e.g. in ParaView). The result directory can be specified in the config file for the mapping backend.

### Configuration files ###

The config folder contains example configuration files. Please read the
documentation of the individual parameters in the yaml file carefully.
You have various options to trade-off accuracy and computational expense as well
as to enable online calibration.

