#include "ceres/ceres.h"
#include <gtest/gtest.h>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/assert_macros.hpp>

#include <okvis/ceres/SubmapIcpError.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>

#include <okvis/config_mapping.hpp>
//#include "../../supereight-2-srl-fork/app/include/config.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"

#include <random>

// Map Config
const std::string config_filename = "/home/boche/projects/okvis2-leica/config/icp_unit_test.yaml";

TEST(okvisTestSuite, SubmapAlignmentJacobian){
  /**
   * Create plane wall example
   */
  const double elevation_min = -10.0;
  const double elevation_max = 10.0;
  const double azimuth_min = -15.0;
  const double azimuth_max = 15.0;
  // angular resolution [degree]
  const double elevation_res = 0.25;
  const double azimuth_res = 0.25;
  // conversion degree <-> rad
  const double deg_to_rad = M_PI / 180.;
  // distance of plane wall [m]
  const double d = 15.0;
  // measurement uncertainty
  double sigma_measurement = 0.1;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pointCloud_a;
  size_t num_points_elevation = std::floor((elevation_max - elevation_min) / elevation_res);
  size_t num_points_azimuth = std::floor((azimuth_max - azimuth_min) / azimuth_res);

  double elevation_angle = elevation_min;
  double azimuth_angle = azimuth_min;
  double x, y, z;
  x = d;
  for(int i = 0; i < num_points_elevation; i++){
    z = d*tan(elevation_angle * deg_to_rad);
    for(int j = 0; j < num_points_azimuth; j++){
      y = d*tan(azimuth_angle * deg_to_rad);
      // save point
      pointCloud_a.push_back(Eigen::Vector3d(x,y,z));
      // increase azimuth angle
      azimuth_angle+=azimuth_res;
    }
    azimuth_angle = azimuth_min;
    //increase elevation angle
    elevation_angle+=elevation_res;
  }
  std::cout << "Created " << pointCloud_a.size() << " points" << std::endl;

  /**
   * Create reference submap in frame {A}
   * Scenario: Plane wall
   */

  // ========= Config & I/O INITIALIZATION  =========
  se::Config<se::OccupancyDataConfig, se::LeicaLidarConfig> se_config(config_filename);
  std::cout << "Created LiDAR Mapping backend with following parameters:\n" << se_config << std::endl;

  // ========= Map INITIALIZATION  =========
  se::OccupancyMap<se::Res::Multi> map(se_config.map, se_config.data);

  // ========= Sensor INITIALIZATION  =========
  se::LeicaLidarConfig sensorConfig(se_config.sensor);
  const se::LeicaLidar sensor(sensorConfig);

  // Setup input, processed and output imgs
  Eigen::Matrix4f T_WS = Eigen::Matrix4f::Identity();

  // ========= Integrator INITIALIZATION  =========
  se::MapIntegrator integrator(map);

  auto measurementIter = pointCloud_a.begin();
  int frame = 0;
  for(; measurementIter!=pointCloud_a.end(); measurementIter++) {
    integrator.integrateRay(sensor, (*measurementIter).cast<float>(), T_WS, frame);
  }

  std::cout << "Finished Integration. Saving submap (mesh & slice)" << std::endl;
  std::cout << "------------------- " << std::endl;
  std::cout << " Occupancies along center ray: " << std::endl;

  double d_i = 0.0;
  while(d_i < (d + 1.)){
    Eigen::Vector3f p_i(d_i, 0., 0.);
    double o_i = map.getFieldInterp(p_i).value_or(0.);
    std::cout << o_i << std::endl;
    d_i += 0.1;
  }
  std::cout << "------------------- " << std::endl;
  std::cout << "------------------- " << std::endl;
  std::cout << " Gradients along center ray: " << std::endl;
  d_i = 0.0;
  while(d_i < (d + 1.)){
    Eigen::Vector3f p_i(d_i, 0., 0.);
    int scale_returned;
    Eigen::Vector3f grad = map.getFieldGrad(p_i).value_or(Eigen::Vector3f::Zero());
    std::cout << "d_i = " << d_i << " : " << grad.transpose() << "(" << grad.norm() << ")" << std::endl;
    d_i += 0.1;
  }
  d_i = 0.0;
  while(d_i < (d + 1.)){
    Eigen::Vector3f p_i(d_i, 0., 0.);
    Eigen::Vector3f grad = map.getFieldGrad(p_i).value_or(Eigen::Vector3f::Zero());
    std::cout << grad.norm() << std::endl;
    d_i += 0.01;
  }
  std::cout << "------------------- " << std::endl;


  // Save mesh and slice for testing purposes
  map.saveMesh("testMesh.ply");
  map.saveMeshVoxel("testMeshVoxel.ply");
  map.saveStructure("testStructure.ply");
  map.saveFieldSlices("testSliceX.vtk", "testSliceY.vtk", "testSliceZ.vtk", Eigen::Vector3f(0.,0.,0.));

  /**
   * Check Jacobians in a linear region
   */

  Eigen::Vector3d p_b(14.9, 0., 0.);
  okvis::kinematics::Transformation T_WS_a, T_WS_b;
  T_WS_a.setIdentity();
  T_WS_b.setIdentity();
  okvis::ceres::PoseParameterBlock poseParameterBlock_jac0(T_WS_a,0,okvis::Time(0));
  okvis::ceres::PoseParameterBlock poseParameterBlock_jac1(T_WS_b,0,okvis::Time(0));

  // Set up parameter blocks and compute analytic jacobians
  double *parameters[2];
  parameters[0] = poseParameterBlock_jac0.parameters();
  parameters[1] = poseParameterBlock_jac1.parameters();
  double *jacobians[2];
  Eigen::Matrix<double, 1, 7, Eigen::RowMajor> J0;
  Eigen::Matrix<double, 1, 7, Eigen::RowMajor> J1;
  jacobians[0] = J0.data();
  jacobians[1] = J1.data();
  Eigen::Matrix<double, 1, 1> residual;

  ::ceres::CostFunction *cost_function = new okvis::ceres::SubmapIcpError(map, p_b, sigma_measurement);
  static_cast<okvis::ceres::SubmapIcpError *>(cost_function)->EvaluateWithMinimalJacobians(parameters,
                                                                                           residual.data(),
                                                                                           jacobians, NULL);

  // Numerical Jacobian w.r.t. T_WS_a
  double dx = 0.0003; // Do not choose too fine for given map resolution

  // w.r.t. pose {A}
  Eigen::Matrix<double, 1, 6> J0_numDiff;
  for (size_t i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 6, 1> dp_0;
    Eigen::Matrix<double, 1, 1> residual_p;
    Eigen::Matrix<double, 1, 1> residual_m;
    dp_0.setZero();
    dp_0[i] = dx;
    okvis::ceres::PoseManifold::plus(parameters[0], dp_0.data(), parameters[0]);
    static_cast<okvis::ceres::SubmapIcpError *>(cost_function)->Evaluate(parameters, residual_p.data(), NULL);
    poseParameterBlock_jac0.setEstimate(T_WS_a);
    parameters[0] = poseParameterBlock_jac0.parameters();// reset
    dp_0[i] = -dx;
    okvis::ceres::PoseManifold::plus(parameters[0], dp_0.data(), parameters[0]);
    static_cast<okvis::ceres::SubmapIcpError *>(cost_function)->Evaluate(parameters, residual_m.data(), NULL);
    poseParameterBlock_jac0.setEstimate(T_WS_a);
    parameters[0] = poseParameterBlock_jac0.parameters(); // reset


    J0_numDiff.col(i) = (residual_p - residual_m) / (2. * dx);

  }

  // Use lift Jacobian for non-minimal Jacobian
  Eigen::Matrix<double, 1, 7, Eigen::RowMajor> J0_numDiff_lift;
  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> liftJac0;
  okvis::ceres::PoseManifold::minusJacobian(parameters[0], liftJac0.data());
  J0_numDiff_lift = J0_numDiff * liftJac0;

  EXPECT_LT((J0_numDiff_lift - J0).norm(), 1e-03)
                << "J0: Jacobian Evaluation leads error  " << (J0_numDiff_lift - J0).norm() << " > 1e-05"
                << std::endl;


  std::cout << "J0: Analytical Jacobian evaluates to: \n" << J0 << std::endl;
  std::cout << "J0: Numerical Jacobian evaluates to: \n" << J0_numDiff_lift << std::endl;
  std::cout << "J0: J0_numDiff: \n" << J0_numDiff << std::endl;

  // Check Jacobian w.r.t. T_WS_b
  Eigen::Matrix<double, 1, 6> J1_numDiff;
  for (size_t i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 6, 1> dp_1;
    Eigen::Matrix<double, 1, 1> residual_p;
    Eigen::Matrix<double, 1, 1> residual_m;
    dp_1.setZero();
    dp_1[i] = dx;
    okvis::ceres::PoseManifold::plus(parameters[1], dp_1.data(), parameters[1]);
    static_cast<okvis::ceres::SubmapIcpError *>(cost_function)->Evaluate(parameters, residual_p.data(), NULL);
    poseParameterBlock_jac1.setEstimate(T_WS_b);
    parameters[1] = poseParameterBlock_jac1.parameters();// reset
    dp_1[i] = -dx;
    okvis::ceres::PoseManifold::plus(parameters[1], dp_1.data(), parameters[1]);
    static_cast<okvis::ceres::SubmapIcpError *>(cost_function)->Evaluate(parameters, residual_m.data(), NULL);
    poseParameterBlock_jac1.setEstimate(T_WS_b);
    parameters[1] = poseParameterBlock_jac1.parameters(); // reset


    J1_numDiff.col(i) = (residual_p - residual_m) / (2. * dx);

  }

  // Use lift Jacobian for non-minimal Jacobian
  Eigen::Matrix<double, 1, 7, Eigen::RowMajor> J1_numDiff_lift;
  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> liftJac1;
  okvis::ceres::PoseManifold::minusJacobian(parameters[1], liftJac1.data());
  J1_numDiff_lift = J1_numDiff * liftJac1;

  EXPECT_LT((J1_numDiff_lift - J1).norm(), 1e-03)
                << "J1: Jacobian Evaluation leads error  " << (J1_numDiff_lift - J1).norm() << " > 1e-03"
                << std::endl;

  std::cout << "J1: Jacobian evaluates to: \n" << J1 << std::endl;
  std::cout << "J1: Numerical Jacobian evaluates to: \n" << J1_numDiff_lift << std::endl;
  std::cout << "J1: J1_numDiff: \n" << J1_numDiff << std::endl;
}

TEST(okvisTestSuite, SubmapAlignmentOptimisation){
  /**
    * Create room corner example
    */
  const double angular_resolution = 0.5;
  const double deg_to_rad = M_PI / 180.;
  const double d = 5.0;
  double sigma_measurement = 0.05;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pointCloud_a;
  pcl::PointCloud<pcl::PointXYZ> pclCloud_a;
  pclCloud_a.height = 1; // will be unorganized cloud
  size_t num_points = std::floor(90.0 / angular_resolution);

  double elevation_angle = -45.0;
  double azimuth_angle = -45.0;
  double x, y, z;

  // Wall looking frontward (fw: x)
  x = d;
  for(int i = 0; i < num_points; i++){
    z = d*tan(elevation_angle * deg_to_rad);
    for(int j = 0; j < num_points; j++){
      y = d*tan(azimuth_angle * deg_to_rad);
      // save point
      pointCloud_a.push_back(Eigen::Vector3d(x,y,z));
      pclCloud_a.push_back(pcl::PointXYZ(x,y,z));
      // increase azimuth angle
      azimuth_angle+=angular_resolution;
    }
    azimuth_angle = -45.;
    //increase elevation angle
    elevation_angle+=angular_resolution;
  }

  // Wall looking left (left: y)
  elevation_angle = -45.0;
  azimuth_angle = -45.0;
  y = d;
  for(int i = 0; i < num_points; i++){
    z = d*tan(elevation_angle * deg_to_rad);
    for(int j = 0; j < num_points; j++){
      x = d*tan(azimuth_angle * deg_to_rad);
      // save point
      pointCloud_a.push_back(Eigen::Vector3d(x,y,z));
      pclCloud_a.push_back(pcl::PointXYZ(x,y,z));
      // increase azimuth angle
      azimuth_angle+=angular_resolution;
    }
    azimuth_angle = -45.;
    //increase elevation angle
    elevation_angle+=angular_resolution;
  }

  // Wall looking down (down: -z)
  elevation_angle = -45.0;
  azimuth_angle = -45.0;
  z = -d;
  for(int i = 0; i < num_points; i++){
    y = d*tan(elevation_angle * deg_to_rad);
    for(int j = 0; j < num_points; j++){
      x = d*tan(azimuth_angle * deg_to_rad);
      // save point
      pointCloud_a.push_back(Eigen::Vector3d(x,y,z));
      pclCloud_a.push_back(pcl::PointXYZ(x,y,z));
      // increase azimuth angle
      azimuth_angle+=angular_resolution;
    }
    azimuth_angle = -45.;
    //increase elevation angle
    elevation_angle+=angular_resolution;
  }


  // Save Point Cloud
  pclCloud_a.width = pointCloud_a.size();
  pcl::io::savePCDFileASCII ("cloud_a.pcd", pclCloud_a);
  std::cout << "Created " << pointCloud_a.size() << " points" << std::endl;

  // Transform point cloud with random transformation
  okvis::kinematics::Transformation T_Sa_Sb;
  T_Sa_Sb.setRandom(0.05, deg_to_rad * 10.0);
  // Transform Point Cloud
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pointCloud_b(pointCloud_a.size());
  pcl::PointCloud<pcl::PointXYZ> pclCloud_b;
  pclCloud_b.height = 1; // will be unorganized cloud
  for(int i = 0; i < pointCloud_a.size(); i++){
    pointCloud_b.at(i) = (T_Sa_Sb.inverse().T() * pointCloud_a.at(i).homogeneous()).head<3>();
    pclCloud_b.push_back(pcl::PointXYZ(pointCloud_b.at(i).x(), pointCloud_b.at(i).y(), pointCloud_b.at(i).z()));
  }
  pclCloud_b.width = pointCloud_b.size();
  std::cout << "Transformed point cloud has size:  " << pointCloud_b.size() << " points" << std::endl;

  pcl::io::savePCDFileASCII ("cloud_b.pcd", pclCloud_b);

  /**
   * Create reference submap in frame {A}
   * Scenario: Corner of a room
   */


  // ========= Config & I/O INITIALIZATION  =========
  se::Config<se::OccupancyDataConfig, se::LeicaLidarConfig> se_config(config_filename);
  std::cout << "Created LiDAR Mapping backend with following parameters:\n" << se_config << std::endl;

  // ========= Map INITIALIZATION  =========
  //se_config.map.dim = Eigen::Vector3f(12., 12., 12.);
  //se_config.map.res = 0.05;
  //se_config.map.fs_res = 0.1;
  se::OccupancyMap<se::Res::Multi> map(se_config.map, se_config.data);

  // ========= Sensor INITIALIZATION  =========
  se::LeicaLidarConfig sensorConfig(se_config.sensor);
  const se::LeicaLidar sensor(sensorConfig);

  // Setup input, processed and output imgs
  Eigen::Matrix4f T_WS = Eigen::Matrix4f::Identity(); //< assume identity, no body-to-sensor transformation for testing

  // ========= Integrator INITIALIZATION  =========
  se::MapIntegrator integrator(map);

  auto measurementIter = pointCloud_a.begin();
  int frame = 0;
  for(; measurementIter!=pointCloud_a.end(); measurementIter++) {
    integrator.integrateRay(sensor, (*measurementIter).cast<float>(), T_WS, frame);
  }

  std::cout << "Finished Integration. Saving submap (mesh & slice)" << std::endl;
  // Save mesh and slice for testing purposes
  map.saveMesh("testMesh.ply");
  map.saveMeshVoxel("testMeshVoxel.ply");
  map.saveStructure("testStructure.ply");
  map.saveFieldSlices("testSliceX.vtk", "testSliceY.vtk", "testSliceZ.vtk", Eigen::Vector3f(0.,0.,0.));


  /**
   * Set up Ceres Problem
   */
  ::ceres::Problem::Options problemOptions;
  problemOptions.manifold_ownership =
          ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.loss_function_ownership =
          ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.cost_function_ownership =
          ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  ::ceres::Problem problem(problemOptions);
  ::ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ::FLAGS_stderrthreshold=google::WARNING; // enable console warnings (Jacobian verification)
  ::ceres::Solver::Summary summary;
  /**
   * Set up reference frame {A} of the submap
   */

  // General stuff and ground truth trafos
  okvis::ceres::PoseManifold pose6dParameterisation;
  // T_WS_a
  okvis::kinematics::Transformation T_WS_a;
  T_WS_a.setIdentity(); // Ground Truth
  okvis::ceres::PoseParameterBlock poseParameterBlock_a(T_WS_a,0,okvis::Time(0));
  problem.AddParameterBlock(poseParameterBlock_a.parameters(), okvis::ceres::PoseParameterBlock::Dimension, &pose6dParameterisation);
  // T_WS_b & T_Sa_Sb
  okvis::kinematics::Transformation T_WS_b;
  T_WS_b = T_WS_a * T_Sa_Sb;
  okvis::ceres::PoseParameterBlock poseParameterBlock_b(T_WS_b,0,okvis::Time(0));
  problem.AddParameterBlock(poseParameterBlock_b.parameters(), okvis::ceres::PoseParameterBlock::Dimension, &pose6dParameterisation);
  // Creat disturbance
  okvis::kinematics::Transformation T_dist;
  T_dist.setRandom(0.1, deg_to_rad * 10.0);

  // Create
  pcl::PointCloud<pcl::PointXYZ> pclCloud_noises;
  pclCloud_noises.height = 1; // will be unorganized cloud
  std::default_random_engine genx;
  genx.seed(1);
  std::normal_distribution<double> distributionx(0.0, sigma_measurement);
  std::default_random_engine geny;
  geny.seed(2);
  std::normal_distribution<double> distributiony(0.0, sigma_measurement);
  std::default_random_engine genz;
  genz.seed(3);
  std::normal_distribution<double> distributionz(0.0, sigma_measurement);

  // Add residual blocks
  int count = 0;
  for(auto p_b : pointCloud_b){
    double sigx = distributionx(genx);
    double sigy = distributiony(geny);
    double sigz = distributionz(genz);
    Eigen::Vector3d disturbed_point;
    disturbed_point = p_b;
    disturbed_point.x()+=sigx;
    disturbed_point.y()+=sigy;
    disturbed_point.z()+=sigz;

    if(count % 20 ==0){
      disturbed_point.x() += 2.;
      disturbed_point.y() -= 3.;
      disturbed_point.z() += 1.;
    }

    count++;
    pclCloud_noises.push_back(pcl::PointXYZ(disturbed_point.x(), disturbed_point.y(), disturbed_point.z()));

    ::ceres::CostFunction* cost_function = new okvis::ceres::SubmapIcpError(map, disturbed_point, sigma_measurement);
    problem.AddResidualBlock(cost_function, NULL, poseParameterBlock_a.parameters(), poseParameterBlock_b.parameters());
  }
  pclCloud_noises.width = pointCloud_b.size();
  pcl::io::savePCDFileASCII ("cloud_disturbed.pcd", pclCloud_noises);

  // Problem 1: Set {A} const and disturb & optimize for {B}
  std::cout << "### Problem 1: Set {A} const and disturb & optimize for {B} ###" << std::endl;
  problem.SetParameterBlockConstant(poseParameterBlock_a.parameters());
  okvis::kinematics::Transformation T_WS_b_dist = T_WS_b * T_dist;
  poseParameterBlock_b.setEstimate(T_WS_b_dist);

  // Run the solver!
  std::cout << "run the solver... " << std::endl;
  Solve(options, &problem, &summary);

  std::cout << " ------ T_WSa (optimised) ------ \n" << poseParameterBlock_a.estimate().T() << std::endl;
  std::cout << " ------------------------------- " << std::endl;
  std::cout << " ------ T_WSb ------ \n" << T_WS_b.T() << std::endl;
  std::cout << " ------------------------------- " << std::endl;
  std::cout << " ------ T_WS_b_dist ------ \n" << T_WS_b_dist.T() << std::endl;
  std::cout << " ------------------------------- " << std::endl;
  std::cout << " ------ T_WSb (optimised) ------ \n" << poseParameterBlock_b.estimate().T() << std::endl;
  std::cout << " ------------------------------- " << std::endl;

  // Translation Error
  EXPECT_LT((T_WS_b.r()-poseParameterBlock_b.estimate().r()).norm(), 1e-02) << "Translation Error too large!";
  // Orientation Error
  EXPECT_LT(2*(T_WS_b.q()*poseParameterBlock_b.estimate().q().inverse()).vec().norm(), 1e-02) << "Orientation Error too large!";

  // Problem 2: Set {B} const and disturb & optimize for {A}
  std::cout << "### Problem 2: Set {B} const and disturb & optimize for {A} ###" << std::endl;

  // Do the same fixing frame {B} and optimising frame {A}
  poseParameterBlock_b.setEstimate(T_WS_b);
  problem.SetParameterBlockConstant(poseParameterBlock_b.parameters());
  okvis::kinematics::Transformation T_WS_a_dist = T_WS_a * T_dist;
  poseParameterBlock_a.setEstimate(T_WS_a_dist);
  problem.SetParameterBlockVariable(poseParameterBlock_a.parameters());
  // Run the solver!
  std::cout << "run the solver... " << std::endl;
  Solve(options, &problem, &summary);

  std::cout << " ------ T_WSa ------ \n" << T_WS_a.T() << std::endl;
  std::cout << " ------------------------------- " << std::endl;
  std::cout << " ------ T_WS_a_dist ------ \n" << T_WS_a_dist.T() << std::endl;
  std::cout << " ------------------------------- " << std::endl;
  std::cout << " ------ T_WSa (optimised) ------ \n" << poseParameterBlock_a.estimate().T() << std::endl;
  std::cout << " ------------------------------- " << std::endl;

  EXPECT_LT((T_WS_a.r()-poseParameterBlock_a.estimate().r()).norm(), 1e-02) << "Translation Error too large!";
  // Orientation Error
  EXPECT_LT(2*(T_WS_a.q()*poseParameterBlock_a.estimate().q().inverse()).vec().norm(), 1e-02) << "Orientation Error too large!";

}
