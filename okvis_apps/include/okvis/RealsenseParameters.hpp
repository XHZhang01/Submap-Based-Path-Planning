#ifndef INCLUDE_OKVIS_REALSENSEPARAMETERS_HPP_
#define INCLUDE_OKVIS_REALSENSEPARAMETERS_HPP_

#include <glog/logging.h>
#include <okvis/assert_macros.hpp>

namespace okvis {

/**
 * @brief      Struct with realsense parameters
 */
struct RealsenseParameters {
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief      Constructs a new instance given a config file.
   *
   * @param[in]  filename  The path to the config file
   */
  RealsenseParameters(const std::string &filename) { readConfigFile(filename); };

  /**
   * @brief      Constructor
   *
   * @param[in]  global_time       When true the host computer clock is used for
   *                               the timestamps.
   * @param[in]  enable_rgb        Enable/Disable RGB stream
   * @param[in]  ir_autoexposure   Enable/Disable Infrared camera autoexposure
   * @param[in]  ir_exposure       The IR cameras exposure time in usec
   * @param[in]  rgb_autoexposure  Enable/Disabe RGB autoexposure
   * @param[in]  rgb_exposure      The RGB exposure time in microseconds
   * @param[in]  rgb_auto_wb       Enable/Disable RGB auto white balance
   * @param[in]  rgb_wb            RGB white balance in Kelvins (?). Range: [2800, 6500]
   */
  RealsenseParameters(const bool global_time = false, const bool enable_rgb = false, const bool ir_autoexposure = false,
                      const float ir_exposure = 4000.f, const bool rgb_autoexposure = false,
                      const float rgb_exposure = 150.f, const bool rgb_auto_wb = true, const float rgb_wb = 3000.f,
                      const float laser_power = 360.f)
      : global_time(global_time), enable_rgb(enable_rgb), ir_autoexposure(ir_autoexposure), ir_exposure(ir_exposure),
        rgb_autoexposure(rgb_autoexposure), rgb_exposure(rgb_exposure), rgb_auto_wb(rgb_auto_wb), rgb_wb(rgb_wb),
        laser_power(laser_power){};

  bool global_time;      ///< True for using host computer timestamps
  bool enable_rgb;       ///< Enable/Disable the RGB stream
  bool ir_autoexposure;  ///< True for autoexposure, False for manual exposure
  float ir_exposure;     ///< Exposure time of the IR frames in microseconds
  bool rgb_autoexposure; ///< True for autoexposure, False for manual exposure
  float rgb_exposure;    ///< Exposure time of the RGB frames in microseconds
  bool rgb_auto_wb;      ///< True for auto whitebalance, False for manual white balance setting
  float rgb_wb;          ///< RGB white balance in Kelvins (?). Range: [2800, 6500]
  float laser_power;     ///< Laser power [0, 360.0]
protected:
  /**
   * @brief      Reads the realsense parameters from the given config file
   *
   * @param[in]  filename  The filename
   */
  void readConfigFile(const std::string &filename);
};

} // namespace okvis

#endif // INCLUDE_OKVIS_REALSENSEPARAMETERS_HPP_