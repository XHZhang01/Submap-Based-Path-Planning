#ifndef INCLUDE_OKVIS_LIDARMAPPING_CONFIG_HPP
#define INCLUDE_OKVIS_LIDARMAPPING_CONFIG_HPP



#include <se/map/map.hpp>
#include <se/sensor/sensor.hpp>
#include <se/sensor/leica_lidar.hpp>
#include <se/tracker/tracker.hpp>



namespace se {

struct SubMapConfig{

    /** Type of sensor: depth (realsense; not yet supported) | leica | ouster (not yet supported)
     */
    std::string sensorType;
    /** directory to save mesh files to
     */
    std::string resultsDirectory;
    /** Type of integration: ray | rangeImage (not yet supported)
     */
    std::string integrationType;
    /** Type of integration: ray | rangeImage (not yet supported)
     */
    float submapTimeThreshold;
    
    /**
     * How many keyframes conform a submap
    */
    int submapKfThreshold;

    /**
     * How many states within a submap are needed to send a chunk of the submap
    */
    int submapStateThreshold;

    /**
     * Downsampling rate of the depth sensor measurements
    */
    int sensorMeasurementDownsampling;
    
    /**
     * Downsampling factor of the depth image resolution 
    */
    int depthImageResDownsampling;
    
    /** Default Constructor
     */
    SubMapConfig() : sensorType("leica"), resultsDirectory("/home/"), integrationType("ray"), submapTimeThreshold(7.0f), submapKfThreshold(5) {};

    /** Initializes the config from a YAML file. Data not present in the YAML file will be initialized
     * as in MapConfig::MapConfig().
     */
    SubMapConfig(const std::string& yaml_file)
    {
      // Open the file for reading.
      cv::FileStorage fs;
      try {
        if (!fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
          std::cerr << "Error: couldn't read configuration file " << yaml_file << "\n";
          return;
        }
      }
      catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << yaml_file << "\n";
        return;
      }

      // Get the node containing the general configuration.
      const cv::FileNode node = fs["general"];
      if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default map configuration, no \"map\" section found in "
                  << yaml_file << "\n";
        return;
      }

      // Read the config parameters.
      se::yaml::subnode_as_string(node, "sensor_type", sensorType);
      se::yaml::subnode_as_string(node, "results_directory", resultsDirectory);
      se::yaml::subnode_as_string(node, "integration_type", integrationType);
      se::yaml::subnode_as_float(node, "submap_time_threshold", submapTimeThreshold);
      se::yaml::subnode_as_int(node, "submap_kf_threshold", submapKfThreshold);
      se::yaml::subnode_as_int(node, "submap_state_buffer", submapStateThreshold);
      se::yaml::subnode_as_int(node, "sensor_measurement_downsampling", sensorMeasurementDownsampling);
      se::yaml::subnode_as_int(node, "depth_image_resolution_downsampling", depthImageResDownsampling);

    }

    friend std::ostream& operator<<(std::ostream& os, const SubMapConfig& c)
    {
      os << str_utils::value_to_pretty_str(c.sensorType, "sensor_type") << " \n";
      os << str_utils::value_to_pretty_str(c.resultsDirectory, "results_directory") << " \n";
      os << str_utils::value_to_pretty_str(c.integrationType, "integration_type") << " \n";
      os << str_utils::value_to_pretty_str(c.submapTimeThreshold, "submap_time_threshold") << " \n";
      os << str_utils::value_to_pretty_str(c.submapKfThreshold, "submap_kf_threshold") << " \n";
      os << str_utils::value_to_pretty_str(c.submapStateThreshold, "submap_state_threshold") << " \n";
      os << str_utils::value_to_pretty_str(c.sensorMeasurementDownsampling, "sensor_measurement_downsampling") << " \n";
      os << str_utils::value_to_pretty_str(c.depthImageResDownsampling, "depth_image_res_downsampling") << " \n";
      return os;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
// ToDo: bad practise

template<typename DataConfigT, typename SensorConfigT>
struct Config {
    MapConfig map;
    DataConfigT data;
    SensorConfigT sensor;

    /** Default initializes all configs.
     */
    Config();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in Config::Config().
     */
    Config(const std::string& yaml_file);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



template<typename DataConfigT, typename SensorConfigT>
std::ostream& operator<<(std::ostream& os, const Config<DataConfigT, SensorConfigT>& c);



template<typename DataConfigT, typename SensorConfigT>
struct OkvisSubmapsConfig {
    SubMapConfig general;
    Config<DataConfigT,SensorConfigT> se_config;

    /** Default initializes all configs.
     */
    OkvisSubmapsConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in Config::Config().
     */
    OkvisSubmapsConfig(const std::string& yaml_file);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename DataConfigT, typename SensorConfigT>
std::ostream& operator<<(std::ostream& os, const OkvisSubmapsConfig<DataConfigT, SensorConfigT>& c);

} // namespace se

#include "okvis/impl/config_mapping_impl.hpp"

#endif // INCLUDE_OKVIS_LIDARMAPPING_CONFIG_HPP
