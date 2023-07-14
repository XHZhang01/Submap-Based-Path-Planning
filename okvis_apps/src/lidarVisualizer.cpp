//
// Created by boche on 4/26/22.
//
#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_lib_io.h>
#include <boost/thread.hpp>

#include <okvis/kinematics/Transformation.hpp>
#include <okvis/TrajectoryOutput.hpp>

// Flight02
/*const okvis::kinematics::Transformation T_CL_old(Eigen::Vector3d(0.07235501047797023, 0.0017529662792704348, -0.03625700624694586),
                                             Eigen::Quaterniond(-0.3107885830313003, 0.637112774371234, -0.31040356950079717,  0.6333619766228074));
const okvis::kinematics::Transformation T_SC_old(Eigen::Vector3d(0., 0., 0.),
                                             Eigen::Quaterniond(0.008032572456710416, -0.0020134062736605835,0.9999530239380764,0.005037250440647953));
const okvis::kinematics::Transformation T_BC_old(Eigen::Vector3d(0.08165, -0.00239, 0.007),
                                             Eigen::Quaterniond(0.9999961447586534, -0.00022746219806146848,-0.002434893045869958,0.00131530400816469)); */
// outdoor flight03
//const okvis::kinematics::Transformation T_CL(Eigen::Vector3d(0.070466281969720, 0.000469872176891, -0.032769723644106),
//                                             Eigen::Quaterniond(0.152975452229739, 0.696338822860729, 0.136025580418902, 0.687901007604831));
//const okvis::kinematics::Transformation T_SC(Eigen::Vector3d(0., 0., 0.),
//                                             Eigen::Quaterniond(0.008923019006909, -0.003094270475085, 0.999948713228815, 0.003657339752711));
//const okvis::kinematics::Transformation T_BC(Eigen::Vector3d(0.08165, -0.00239, 0.007),
//                                             Eigen::Quaterniond(0.999867991833150, 0.002473281120503, -0.005667012253960, 0.015025536934025));

// Newer College (use C: Cam1, B: also Cam1 as GT Poses are given in C1 frame, L: Lidar, S: IMU)
const okvis::kinematics::Transformation T_CL(Eigen::Vector3d(0.0250, -0.0503, -0.0842), Eigen::Quaterniond(0.2706, 0.2706, 0.6533, -0.6533));
const okvis::kinematics::Transformation T_SC(Eigen::Vector3d(-0.0058, -0.0174, 0.0159), Eigen::Quaterniond(1.0000, 0.0007, -0.0009, 0.0015));
const okvis::kinematics::Transformation T_BC(Eigen::Vector3d(0., 0., 0.), Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5));

bool loadLidarCsv(std::string lidarFileName, std::vector<okvis::Time>& timestamps,
                  std::vector<Eigen::Vector3d>& lidarMeasurements){

  // Open the Lidar File
  std::ifstream lidarFile_;
  lidarFile_.open(lidarFileName);
  if(!lidarFile_.good()){
    std::cout << "No LiDAR file found at " << lidarFileName << std::endl;
    return false;
  }
  // Iterate through whole file
  std::string line;
  int number_of_lines = 0;
  while (std::getline(lidarFile_, line))
    ++number_of_lines;

  std::cout << "No. LiDAR measurements: " << number_of_lines-1 << std::endl;
  if (number_of_lines - 1 <= 0) {
    std::cout << "no LiDAR messages present in " << lidarFileName << std::endl;
    return false;
  }

  // set reading position to second line
  lidarFile_.clear();
  lidarFile_.seekg(0, std::ios::beg);
  std::getline(lidarFile_, line);

  // Now read all LiDAR measurements
  uint64_t ts;
  int intensity;
  Eigen::Vector3d ptXYZ;

  while(std::getline(lidarFile_, line)){

    // Parse single line
    std::stringstream stream(line);
    std::string s;

    // 1st entry: timestamp
    std::getline(stream, s, ',');
    ts = std::stol(s.c_str());

    // 2nd - 4th entry: xyz coordinates in LiDAR frame
    for(int j = 0; j < 3; ++j){
      std::getline(stream, s, ',');
      ptXYZ[j] = std::stod(s);
    }
    // save measurement
    okvis::Time time;
    time.fromNSec(ts);
    timestamps.push_back(time);
    lidarMeasurements.push_back(ptXYZ);

    // 5th entry: intensity
    std::getline(stream, s);
    intensity = std::stoi(s);
  }
  return true;
}


bool loadLeicaTrajectoryCsv(std::string trajectoryFileName, std::vector<okvis::Time>& timestamps,
                            std::vector<okvis::kinematics::Transformation>& trajMeasurements){

  // Open the Trajectory File
  std::ifstream trajectoryFile_;
  trajectoryFile_.open(trajectoryFileName);
  if(!trajectoryFile_.good()){
    std::cout << "No Trajectory file found at " << trajectoryFileName << std::endl;
    return false;
  }
  // Iterate through whole file
  std::string line;
  int number_of_lines = 0;
  while (std::getline(trajectoryFile_, line))
    ++number_of_lines;

  std::cout << "No. Poses: " << number_of_lines-1 << std::endl;
  if (number_of_lines - 1 <= 0) {
    std::cout << "no poses present in " << trajectoryFileName << std::endl;
    return false;
  }

  // set reading position to second line
  trajectoryFile_.clear();
  trajectoryFile_.seekg(0, std::ios::beg);
  std::getline(trajectoryFile_, line);

  // Now read all trajectory poses
  uint64_t ts;
  int id;
  Eigen::Quaterniond q_WB;
  Eigen::Vector3d t_WB;

  while(std::getline(trajectoryFile_, line)){

    // Parse single line
    std::stringstream stream(line);
    std::string s;

    // 1st entry: ID
    std::getline(stream, s, ',');
    id = std::stoi(s);

    // 2nd entry: timestamp
    std::getline(stream, s, ',');
    ts = std::stol(s.c_str());

    // 3rd - 5th entry: t_WB
    for(int j = 0; j < 3; ++j){
      std::getline(stream,s,',');
      t_WB[j] = std::stod(s);
    }

    // 6th - 9th entry: q_WB_w q_WB_x q_WB_y q_WB_z
    std::getline(stream, s, ',');
    q_WB.w() = std::stof(s);
    std::getline(stream, s, ',');
    q_WB.x() = std::stof(s);
    std::getline(stream, s, ',');
    q_WB.y() = std::stof(s);
    std::getline(stream, s, ',');
    q_WB.z() = std::stof(s);

    // save measurements
    okvis::Time time;
    time.fromNSec(ts);
    timestamps.push_back(time);
    trajMeasurements.push_back(okvis::kinematics::Transformation(t_WB, q_WB));
  }
  return true;
}

bool loadOkvisTrajectoryCsv(std::string trajectoryFileName, std::vector<okvis::Time>& timestamps,
                            std::vector<okvis::kinematics::Transformation>& trajMeasurements){

  // Open the Trajectory File
  std::ifstream trajectoryFile_;
  trajectoryFile_.open(trajectoryFileName);
  if(!trajectoryFile_.good()){
    std::cout << "No Trajectory file found at " << trajectoryFileName << std::endl;
    return false;
  }
  // Iterate through whole file
  std::string line;
  int number_of_lines = 0;
  while (std::getline(trajectoryFile_, line))
    ++number_of_lines;

  std::cout << "No. Poses: " << number_of_lines-1 << std::endl;
  if (number_of_lines - 1 <= 0) {
    std::cout << "no poses present in " << trajectoryFileName << std::endl;
    return false;
  }

  // set reading position to second line
  trajectoryFile_.clear();
  trajectoryFile_.seekg(0, std::ios::beg);
  std::getline(trajectoryFile_, line);

  // Now read all trajectory poses
  uint64_t ts;
  Eigen::Quaterniond q_WS;
  Eigen::Vector3d t_WS;

  while(std::getline(trajectoryFile_, line)){

    // Parse single line
    std::stringstream stream(line);
    std::string s;

    // 1st entry: timestamp
    std::getline(stream, s, ',');
    ts = std::stol(s.c_str());

    // 2rd - 4th entry: t_WS
    for(int j = 0; j < 3; ++j){
      std::getline(stream,s,',');
      t_WS[j] = std::stof(s);
    }

    // 5th - 8th entry: q_WB_x q_WB_y q_WB_z q_WB_w
    std::getline(stream, s, ',');
    q_WS.x() = std::stof(s);
    std::getline(stream, s, ',');
    q_WS.y() = std::stof(s);
    std::getline(stream, s, ',');
    q_WS.z() = std::stof(s);
    std::getline(stream, s, ',');
    q_WS.w() = std::stof(s);

    // save measurements
    okvis::Time time;
    time.fromNSec(ts);
    timestamps.push_back(time);
    trajMeasurements.push_back(okvis::kinematics::Transformation(t_WS, q_WS));
  }
  return true;
}

void lidarToPcl(okvis::kinematics::Transformation T_ref_L,
                std::vector<okvis::Time>& lidarTimestamps, std::vector<Eigen::Vector3d>& lidarMeasurements,
                std::vector<okvis::Time>& trajTimestamps, std::vector<okvis::kinematics::Transformation>& trajMeasurements,
                pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud){

  size_t nTraj = trajTimestamps.size();
  size_t nLidar = lidarTimestamps.size();

  size_t l = 0;
  size_t t = 0;

  // skip LiDAR measurements that are older than the first pose
  int count = 0;
  while(lidarTimestamps.at(l).toNSec() < trajTimestamps.at(0).toNSec()){
    count++;
    l++;
  }

  std::cout << "dropped first " << count << " measurements " << std::endl;
  std::cout << trajTimestamps.at(0).toNSec() << std::endl;
  std::cout << lidarTimestamps.at(l).toNSec() << std::endl;

  // Iter over trajectory measurements
  for(; t < nTraj; t++){
    //
    if(t == (nTraj-1)){
      std::cout << "last pose" << std::endl;
      while(l < nLidar){
        Eigen::Vector4d pt_L;
        pt_L.head<3>() = lidarMeasurements.at(l);
        pt_L[3] = 1.;

        Eigen::Vector4d pt_W = trajMeasurements.at(t) * T_ref_L * pt_L;
        ptCloud->points.push_back(pcl::PointXYZ(pt_W[0], pt_W[1], pt_W[2]));

        ++l;
      }
    }
    else{
      while((lidarTimestamps.at(l) < trajTimestamps.at(t+1)) && (t < nTraj-1)){
        // take current pose and convert
        Eigen::Vector4d pt_L;
        pt_L.head<3>() = lidarMeasurements.at(l);
        pt_L[3] = 1.;
        Eigen::Vector4d pt_W = trajMeasurements.at(t) * T_ref_L * pt_L;
        ptCloud->points.push_back(pcl::PointXYZ(pt_W[0], pt_W[1], pt_W[2]));

        l++;
        if(l == (nLidar-1)){
          break;
        }
      }
      std::cout << "t = " << t << std::endl;
      std::cout << "ptCloud->size()" << ptCloud->size() << std::endl;
      if(l == (nLidar-1)){
        break;
      }
    }
  }
  return;
}

void visualisePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud, double ptThickness = 0.1,
                         double camPosX = 0.0, double camPosY = 0.0, double camPosZ = 0.0,
                         double lookAtX = 0.0, double lookAtY = 0.0, double lookAtZ = 0.0,
                         double upX = 0.0, double upY = 0.0, double upZ = 1.0){

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (ptCloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ptThickness, "sample cloud");
  viewer->setCameraPosition(camPosX, camPosY, camPosZ, lookAtX, lookAtY, lookAtZ, upX, upY, upZ);
  viewer->addCoordinateSystem (1.0);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    pcl::visualization::Camera cam;
    std::vector<pcl::visualization::Camera> cams;
    cams.push_back(cam);
    viewer->getCameras(cams);
    std::cout << "posx: " << cams.at(0).pos[0] << std::endl;
    std::cout << "posy: " << cams.at(0).pos[1] << std::endl;
    std::cout << "posz: " << cams.at(0).pos[2] << std::endl;
    std::cout << "focx: " << cams.at(0).focal[0] << std::endl;
    std::cout << "focy: " << cams.at(0).focal[1] << std::endl;
    std::cout << "focz: " << cams.at(0).focal[2] << std::endl;


    // Leica: z-downward
    //viewer->setCameraPosition(3.,-8.,-4.5,2.,-.5,-1.2,0.,0.,-1.);
    // OKVIS: z-upward => change sign x and z
    //viewer->setCameraPosition(camPosX, camPosY, camPosZ, lookAtX, lookAtY, lookAtZ, upX, upY, upZ);

    //std::cout << "current viewing pose:\n" << viewer->getViewerPose().matrix() << std::endl;
  }

  return;
}

void downSamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud, double res, pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPtCloud){
  return;
}


int main(int argc, char** argv){

  // obtain transformation Lidar to body
  okvis::kinematics::Transformation T_BL = T_BC * T_CL;
  std::cout << "###" << std::endl;
  std::cout << T_CL.T() << std::endl;
  std::cout << T_BC.T() << std::endl;
  std::cout << T_BL.T() << std::endl;
  std::cout << "###" << std::endl;
  okvis::kinematics::Transformation T_SL = T_SC * T_CL; // S: Sensor (IMU bottom)

  // command line options: pcd file | leica-csv | okvis-csv   (csv modes always uses previous frame pose for motion compensation)
  if (argc != 3) {
    std::cout << "[ERROR] Usage: ./" << argv[0] << " [ -pcd | -csv-leica | -csv-okvis ] [filename]" << std::endl;
    return -1;
  }

  //
  std::string format(argv[1]);
  std::string filename(argv[2]);

  if(strcmp(argv[1], "-pcd") == 0){
    // Reading full LiDAR point cloud already in correct frame => no lidar file needed
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *lidarPointCloud) == -1){
      PCL_ERROR ("Couldn't read PCL file. \n");
      return (-1);
    }
    std::cout << "Loaded " << lidarPointCloud->width * lidarPointCloud->height << " data points." << std::endl;

    // Visualize
    double camPosX = -12.; //3.;
    double camPosY = 23.; // -8.;
    double camPosZ = 27.; // -4.5;
    double lookAtX = 2.3; //2.;
    double lookAtY = 1.3; //-.5;
    double lookAtZ = 1.7; //-1.2;
    double upX = 0.;
    double upY = 0.;
    double upZ = 1.;


    visualisePointCloud(lidarPointCloud, 0.2, camPosX, camPosY, camPosZ, lookAtX, lookAtY, lookAtZ, upX, upY,upZ);

    // downsampling
//    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarPointCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::VoxelGrid<pcl::PointXYZ> sor;
//    sor.setInputCloud(lidarPointCloud);
//    sor.setLeafSize(0.05f, 0.05f, 0.05f);
//    sor.filter(*lidarPointCloudFiltered);
//    std::cout << "downsampled to " << lidarPointCloudFiltered->points.size() << " data points" << std::endl;
//
//    // Visualize
//    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    viewer->addPointCloud<pcl::PointXYZ> (lidarPointCloudFiltered, "sample cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//    viewer->addCoordinateSystem (1.0);
//    viewer->initCameraParameters ();
//
//    while (!viewer->wasStopped ())
//    {
//      viewer->spinOnce (100);
//    }


    // mesh visualisation

    // Normal estimation*
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud (lidarPointCloudFiltered);
//    n.setInputCloud (lidarPointCloudFiltered);
//    n.setSearchMethod (tree);
//    n.setKSearch (20);
//    std::cout << "Computing normals" << std::endl;
//    n.compute (*normals);
//    std::cout << "Normals computed" << std::endl;
//    //* normals should not contain the point normals + surface curvatures
//
//    // Concatenate the XYZ and normal fields*
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields (*lidarPointCloudFiltered, *normals, *cloud_with_normals);
//    //* cloud_with_normals = cloud + normals
//
//    // Create search tree*
//    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//    tree2->setInputCloud (cloud_with_normals);
//
//    std::cout << "created search tree" << std::endl;
//
//    // Initialize objects
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    pcl::PolygonMesh triangles;
//
//    // Set the maximum distance between connected points (maximum edge length)
//    gp3.setSearchRadius (0.2);
//
//    // Set typical values for the parameters
//    gp3.setMu (2.);
//    gp3.setMaximumNearestNeighbors (200);
//    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//    gp3.setMinimumAngle(M_PI/18); // 10 degrees
//    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//    gp3.setNormalConsistency(false);
//
//    // Get result
//    gp3.setInputCloud (cloud_with_normals);
//    gp3.setSearchMethod (tree2);
//    std::cout << "Start meshing" << std::endl;
//    gp3.reconstruct (triangles);
//    std::cout << "Mesh ready" << std::endl;
//
//    std::cout << triangles.header << std::endl;
//    std::cout << triangles.polygons.size() << std::endl;
//
//    // Additional vertex information
//    std::vector<int> parts = gp3.getPartIDs();
//    std::vector<int> states = gp3.getPointStates();
//
//    pcl::io::savePolygonFilePLY("test.ply", triangles);
//    std::cout << "successfully saved ply file" << std::endl;

//    pcl::visualization::PCLVisualizer::Ptr viewerMesh (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    std::cout << "created viewer instance" << std::endl;
//    viewerMesh->setBackgroundColor (0, 0, 0);
//    viewerMesh->addPolygonMesh(triangles);
//    viewerMesh->addCoordinateSystem (1.0);
//    viewerMesh->initCameraParameters ();
//    std::cout << "trying to visualise sth" << std::endl;
//    while (!viewerMesh->wasStopped ()){
//      viewerMesh->spinOnce (100);
//    }

    return 0;
  }
  else if(strcmp(argv[1], "-csv-leica") == 0){
    std::string path(argv[2]);
    std::string lidarFileName = path+"/lidar.csv";
    std::string trajectoryFileName = path+"/trajectory.csv";
    std::vector<okvis::Time> lidarTimestamps;
    std::vector<Eigen::Vector3d> lidarMeasurements;
    loadLidarCsv(lidarFileName,lidarTimestamps,lidarMeasurements);
    std::vector<okvis::Time> trajTimestamps;
    std::vector<okvis::kinematics::Transformation> trajMeasurements;
    loadLeicaTrajectoryCsv(trajectoryFileName,trajTimestamps, trajMeasurements);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    lidarToPcl(T_BL, lidarTimestamps, lidarMeasurements, trajTimestamps, trajMeasurements, ptCloudPtr);

    ptCloudPtr->width = ptCloudPtr->size();
    ptCloudPtr->height = 1;

    // Visualize
    //double camPosX = 12.; //3.; for OKVIS * -1
    //double camPosY = 23.; // -8.; for OKVIS * 1
    //double camPosZ = -27.; // -4.5; for OKVIS * -1
    //double lookAtX = -2.3; //2.;  for OKVIS * -1
    //double lookAtY = 1.3; //-.5;  for OKVIS * 1
    //double lookAtZ = -1.7; //-1.2;  for OKVIS * -1
    //double upX = 0.;
    //double upY = 0.;
    //double upZ = -1.;
    double camPosX = 9.97675;
    double camPosY = -21.6016;
    double camPosZ = -111.372;
    double lookAtX = 9.52828;
    double lookAtY = -16.4113;
    double lookAtZ = 7.55467;
    double upX = 0.;
    double upY = 0.;
    double upZ = -1.;


    visualisePointCloud(ptCloudPtr, 0.2, camPosX, camPosY, camPosZ, lookAtX, lookAtY, lookAtZ, upX, upY,upZ);

    return 0;
  }
  else if(strcmp(argv[1], "-csv-okvis") == 0){
    std::string path(argv[2]);
    std::string lidarFileName = path+"/lidar.csv";
    std::string trajectoryFileName = path+"/okvis2-vio-final_trajectory.csv";
    std::vector<okvis::Time> lidarTimestamps;
    std::vector<Eigen::Vector3d> lidarMeasurements;
    loadLidarCsv(lidarFileName,lidarTimestamps,lidarMeasurements);
    std::vector<okvis::Time> trajTimestamps;
    std::vector<okvis::kinematics::Transformation> trajMeasurements;
    loadOkvisTrajectoryCsv(trajectoryFileName,trajTimestamps, trajMeasurements);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    lidarToPcl(T_SL, lidarTimestamps, lidarMeasurements, trajTimestamps, trajMeasurements, ptCloudPtr);

    ptCloudPtr->width = ptCloudPtr->size();
    ptCloudPtr->height = 1;

    // Visualize
    double camPosX = -9.97675;
    double camPosY = -21.6016;
    double camPosZ = 111.372;
    double lookAtX = -9.52828;
    double lookAtY = -16.4113;
    double lookAtZ = -7.55467;
    double upX = 0.;
    double upY = 0.;
    double upZ = 1.;
    visualisePointCloud(ptCloudPtr, 0.1, camPosX,camPosY,camPosZ,lookAtX,lookAtY,lookAtZ,upX, upY,upZ);

    return 0;
  }
  else{
    std::cout << "[ERROR] Unknown file format flag." << std::endl;
    return -1;
  }

}