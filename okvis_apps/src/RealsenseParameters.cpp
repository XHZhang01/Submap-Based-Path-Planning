#include <okvis/RealsenseParameters.hpp>
#include <opencv2/core.hpp>

namespace okvis {

void RealsenseParameters::readConfigFile(const std::string &filename) {
  {
    // Open file
    cv::FileStorage file(filename, cv::FileStorage::READ);

    OKVIS_ASSERT_TRUE(Exception, file.isOpened(), "Could not open realsense config file: " << filename);
    LOG(INFO) << "Opened realsense configuration file : " << filename;

    // Read timestamp source
    if (file["global_time"].isString()) {
      const std::string str = (std::string)(file["global_time"]);
      if (str.compare("true") == 0 || str.compare("yes") == 0 || str.compare("y") == 0) {
        global_time = true;
      } else if (str.compare("false") == 0 || str.compare("no") == 0 || str.compare("n") == 0) {
        global_time = false;
      } else {
        LOG(INFO) << "Realsense config : global_time not a valid option, setting to false";
        global_time = false;
      }
    } else {
      LOG(INFO) << "Realsense config : global_time not found setting to false";
    }

    // Read enable_rgb
    if (file["enable_rgb"].isString()) {
      const std::string str = (std::string)(file["enable_rgb"]);
      if (str.compare("true") == 0 || str.compare("yes") == 0 || str.compare("y") == 0) {
        enable_rgb = true;
      } else if (str.compare("false") == 0 || str.compare("no") == 0 || str.compare("n") == 0) {
        enable_rgb = false;
      } else {
        LOG(INFO) << "Realsense config : enable_rgb not a valid option, setting to false";
        enable_rgb = false;
      }
    } else {
      LOG(INFO) << "Realsense config : enable_rgb not found, setting to false";
    }

    // Read ir_autoexposure
    if (file["ir_autoexposure"].isString()) {
      const std::string str = (std::string)(file["ir_autoexposure"]);
      if (str.compare("true") == 0 || str.compare("yes") == 0 || str.compare("y") == 0) {
        ir_autoexposure = true;
      } else if (str.compare("false") == 0 || str.compare("no") == 0 || str.compare("n") == 0) {
        ir_autoexposure = false;
      } else {
        LOG(INFO) << "Realsense config : ir_autoexposure not a valid option, setting to false";
        ir_autoexposure = false;
      }
    } else {
      LOG(INFO) << "Realsense config : rgb_autoexposure not found, setting to false";
    }

    // Read ir_exposure
    if (file["ir_exposure"].isReal()) {
      ir_exposure = (float)(file["ir_exposure"]);
    } else {
      LOG(INFO) << "Realsense config : ir_exposure not found, setting to " << 6500.f;
      ;
      ir_exposure = 6500.f;
    }

    // Read rgb_autoexposure
    if (file["rgb_autoexposure"].isString()) {
      const std::string str = (std::string)(file["rgb_autoexposure"]);
      if (str.compare("true") == 0 || str.compare("yes") == 0 || str.compare("y") == 0) {
        rgb_autoexposure = true;
      } else if (str.compare("false") == 0 || str.compare("no") == 0 || str.compare("n") == 0) {
        rgb_autoexposure = false;
      } else {
        LOG(INFO) << "Realsense config : rgb_autoexposure not a valid option, setting to false";
        rgb_autoexposure = false;
      }
    } else {
      LOG(INFO) << "Realsense config : rgb_autoexposure not found, setting to false";
    }

    // Read rgb_auto_wb
    if (file["rgb_auto_wb"].isString()) {
      const std::string str = (std::string)(file["rgb_auto_wb"]);
      if (str.compare("true") == 0 || str.compare("yes") == 0 || str.compare("y") == 0) {
        rgb_auto_wb = true;
      } else if (str.compare("false") == 0 || str.compare("no") == 0 || str.compare("n") == 0) {
        rgb_auto_wb = false;
      } else {
        LOG(INFO) << "Realsense config : rgb_auto_wb not a valid option, setting to false";
        rgb_auto_wb = false;
      }
    } else {
      LOG(INFO) << "Realsense config : rgb_auto_wb not found, setting to true";
      rgb_auto_wb = true;
    }

    // Read rgb_exposure
    if (file["rgb_exposure"].isReal()) {
      rgb_exposure = (float)(file["rgb_exposure"]);
    } else {
      LOG(INFO) << "Realsense config : rgb_exposure not found, setting to " << 150.f;
      rgb_exposure = 150.f;
    }

    // Read rgb_wb
    if (file["rgb_wb"].isReal()) {
      rgb_wb = (float)(file["rgb_wb"]);
    } else {
      LOG(INFO) << "Realsense config : rgb_wb not found, setting to " << 4000.f;
      rgb_wb = 4000.f;
    }

    // Read laser_power
    if (file["laser_power"].isReal()) {
      laser_power = (float)(file["laser_power"]);
    } else {
      LOG(INFO) << "Realsense config : laser_power not found, setting to " << 360.f;
      laser_power = 360.f;
    }
  }
}
} // namespace okvis
