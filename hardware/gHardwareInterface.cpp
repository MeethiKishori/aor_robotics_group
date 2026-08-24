//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Adaptive Autonomy & Off-road Robotics (AOR), RPTU University Kaiserslautern-Landau
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/scout/hardware/gHardwareInterface.cpp
 *
 * \author  Patrick Wolf
 *
 * \date    2026-02-19
 *
 */
//----------------------------------------------------------------------
#include "projects/scout/hardware/gHardwareInterface.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "libraries/hid/mCH341UsbSignalLight.h"

#ifdef _LIB_FINROC_LIBRARIES_LASER_SCANNER_ROBOSENSE_PRESENT_
#include "libraries/laser_scanner/mRobosense.h"
#endif

#ifdef _LIB_FINROC_LIBRARIES_CAMERA_INTEL_REALSENSE_CAMERAS_PRESENT_
#include "libraries/camera/intel_realsense/mD400.h"
#include <librealsense2/rs.hpp>
#endif

#include <filesystem>
#include <exception>
#include <memory>
#include <vector>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace scout
{
namespace hardware
{

namespace
{

const char *cFRONT_INDICATOR_SERIAL_DEVICE = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0";
const char *cREAR_INDICATOR_SERIAL_DEVICE = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0";

std::string ResolveFrontIndicatorDevice()
{
  // Fixed mapping from terminal verification: front lamp is /dev/ttyUSB0.
  if (std::filesystem::exists("/dev/ttyUSB0"))
  {
    return "/dev/ttyUSB0";
  }
  // Fall back to stable physical path if ttyUSB index changed.
  if (std::filesystem::exists(cFRONT_INDICATOR_SERIAL_DEVICE))
  {
    return cFRONT_INDICATOR_SERIAL_DEVICE;
  }
  if (std::filesystem::exists(cREAR_INDICATOR_SERIAL_DEVICE))
  {
    return cREAR_INDICATOR_SERIAL_DEVICE;
  }
  if (std::filesystem::exists("/dev/ttyUSB0"))
  {
    return "/dev/ttyUSB0";
  }
  return cFRONT_INDICATOR_SERIAL_DEVICE;
}

std::string ResolveRearIndicatorDevice()
{
  // Fixed mapping from terminal verification: rear lamp is /dev/ttyUSB1.
  if (std::filesystem::exists("/dev/ttyUSB1"))
  {
    return "/dev/ttyUSB1";
  }
  // Fall back to stable physical path if ttyUSB index changed.
  if (std::filesystem::exists(cREAR_INDICATOR_SERIAL_DEVICE))
  {
    return cREAR_INDICATOR_SERIAL_DEVICE;
  }
  return "";
}

std::string ResolveAlternateRearIndicatorDevice(const std::string& front_device)
{
  // If a dedicated rear path is not distinct, pick another available ttyUSB device.
  if (front_device != "/dev/ttyUSB0" && std::filesystem::exists("/dev/ttyUSB0"))
  {
    return "/dev/ttyUSB0";
  }
  if (front_device != "/dev/ttyUSB1" && std::filesystem::exists("/dev/ttyUSB1"))
  {
    return "/dev/ttyUSB1";
  }
  return "";
}

void ForceDisableIndicator(const std::string& device)
{
  if (device.empty())
  {
    return;
  }
  try
  {
    rrlib::hid::tCH341UsbSignalLightDriver driver(device);
    driver.DisableLight();
  }
  catch (const std::exception& e)
  {
    RRLIB_LOG_PRINT(WARNING, "Failed to force-disable CH341 indicator on ", device, ": ", e.what());
  }
  catch (...)
  {
    RRLIB_LOG_PRINT(WARNING, "Failed to force-disable CH341 indicator on ", device, ": unknown exception");
  }
}

void ForceSetIndicator(const std::string& device,
                       rrlib::hid::ch341::tLightMode light_mode,
                       rrlib::hid::ch341::tBuzzerMode buzzer_mode,
                       rrlib::hid::ch341::tFlashFrequency flash_frequency)
{
  if (device.empty())
  {
    return;
  }
  try
  {
    rrlib::hid::tCH341UsbSignalLightDriver driver(device);
    driver.SetMode(light_mode, buzzer_mode, flash_frequency);
  }
  catch (const std::exception& e)
  {
    RRLIB_LOG_PRINT(WARNING, "Failed to force-set CH341 indicator on ", device, ": ", e.what());
  }
  catch (...)
  {
    RRLIB_LOG_PRINT(WARNING, "Failed to force-set CH341 indicator on ", device, ": unknown exception");
  }
}

#ifdef _LIB_FINROC_LIBRARIES_CAMERA_INTEL_REALSENSE_CAMERAS_PRESENT_
std::vector<std::string> QueryRealsenseSerials(const std::shared_ptr<rs2::context>& context)
{
  std::vector<std::string> serials;
  auto devices = context->query_devices();
  for (size_t i = 0; i < devices.size(); ++i)
  {
    rs2::device dev;
    try
    {
      dev = devices[i];
    }
    catch (...)
    {
      continue;
    }
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
    {
      serials.emplace_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    }
  }
  return serials;
}

std::string ResolveRealsenseSerialByPhysicalPortHint(const std::shared_ptr<rs2::context>& context, const std::string& hint)
{
  auto devices = context->query_devices();
  for (size_t i = 0; i < devices.size(); ++i)
  {
    rs2::device dev;
    try
    {
      dev = devices[i];
    }
    catch (...)
    {
      continue;
    }

    if (!(dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) && dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)))
    {
      continue;
    }

    const std::string physical_port = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
    if (physical_port.find(hint) != std::string::npos)
    {
      return dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    }
  }
  return "";
}

void ConfigureRealsenseDefaults(camera::intel_realsense::mD400* camera)
{
  camera->par_color_width.Set(640);
  camera->par_color_height.Set(480);
  camera->par_depth_width.Set(640);
  camera->par_depth_height.Set(480);
  camera->par_color_framerate.Set(15);
  camera->par_depth_framerate.Set(15);
  camera->par_alignment.Set(camera::intel_realsense::tAlignmentMethod::eALIGN_DEPTH_TO_COLOR);
  camera->in_enable.SetDefault(true);
}
#endif

}

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<gHardwareInterface> cCREATE_ACTION_FOR_G_HARDWAREINTERFACE("HardwareInterface");
#endif

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gHardwareInterface constructor
//----------------------------------------------------------------------
gHardwareInterface::gHardwareInterface(core::tFrameworkElement *parent, const std::string &name,
                                       const std::string &structure_config_file) :
  tSenseControlGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its output and/or input ports)
{


  this->ci_led_indiactor_front_light_mode.SetDefault(rrlib::hid::ch341::tLightMode::eRED);
  this->ci_led_indiactor_front_flash_frequency.SetDefault(rrlib::hid::ch341::tFlashFrequency::eNO_FLASH);
  this->ci_led_indiactor_front_buzzer_mode.SetDefault(rrlib::hid::ch341::tBuzzerMode::eOFF);
  this->ci_led_indiactor_rear_light_mode.SetDefault(rrlib::hid::ch341::tLightMode::eRED);
  this->ci_led_indiactor_rear_flash_frequency.SetDefault(rrlib::hid::ch341::tFlashFrequency::eNO_FLASH);
  this->ci_led_indiactor_rear_buzzer_mode.SetDefault(rrlib::hid::ch341::tBuzzerMode::eOFF);

  const std::string front_device = ResolveFrontIndicatorDevice();
  std::string rear_device = ResolveRearIndicatorDevice();
  if (rear_device == front_device)
  {
    const std::string alternate_rear = ResolveAlternateRearIndicatorDevice(front_device);
    if (!alternate_rear.empty())
    {
      rear_device = alternate_rear;
    }
  }
  const std::string rear_effective_device = rear_device.empty() ? front_device : rear_device;
  RRLIB_LOG_PRINT(USER, "Indicator mapping: front=", front_device, ", rear=", rear_effective_device);
  front_indicator_device_ = front_device;
  rear_indicator_device_ = rear_effective_device;
  ForceSetIndicator(front_indicator_device_,
                    rrlib::hid::ch341::tLightMode::eRED,
                    rrlib::hid::ch341::tBuzzerMode::eOFF,
                    rrlib::hid::ch341::tFlashFrequency::eNO_FLASH);
  if (rear_indicator_device_ != front_indicator_device_)
  {
    ForceSetIndicator(rear_indicator_device_,
                      rrlib::hid::ch341::tLightMode::eRED,
                      rrlib::hid::ch341::tBuzzerMode::eOFF,
                      rrlib::hid::ch341::tFlashFrequency::eNO_FLASH);
  }

  auto led_light_front = new hid::mCH341UsbSignalLight(this, "Front Indicator Light");
  led_light_front->par_serial_device_name.Set(front_device);
  led_light_front->in_buzzer_mode.ConnectTo(this->ci_led_indiactor_front_buzzer_mode);
  led_light_front->in_flash_frequency.ConnectTo(this->ci_led_indiactor_front_flash_frequency);
  led_light_front->in_light_mode.ConnectTo(this->ci_led_indiactor_front_light_mode);


  auto led_light_rear = new hid::mCH341UsbSignalLight(this, "Rear Indicator Light");
  led_light_rear->par_serial_device_name.Set(rear_effective_device);
  led_light_rear->in_buzzer_mode.ConnectTo(this->ci_led_indiactor_rear_buzzer_mode);
  led_light_rear->in_flash_frequency.ConnectTo(this->ci_led_indiactor_rear_flash_frequency);
  led_light_rear->in_light_mode.ConnectTo(this->ci_led_indiactor_rear_light_mode);

#ifdef _LIB_FINROC_LIBRARIES_LASER_SCANNER_ROBOSENSE_PRESENT_
  auto rs_lidar_16 = new laser_scanner::mRobosense(this, "RS-Lidar-16");
  rs_lidar_16->par_ip_address.Set("192.168.1.100");
  rs_lidar_16->par_rs_lidar_model.Set(rrlib::laser_scanner::robosense::tRSLidarModel::eRS_BPEARL);
  rs_lidar_16->out_point_cloud.ConnectTo(this->so_3d_laser);
#endif

#ifdef _LIB_FINROC_LIBRARIES_CAMERA_INTEL_REALSENSE_CAMERAS_PRESENT_
  try
  {
    auto realsense_context = std::make_shared<rs2::context>();
    std::string front_sn = ResolveRealsenseSerialByPhysicalPortHint(realsense_context, "-1/");
    std::string rear_sn = ResolveRealsenseSerialByPhysicalPortHint(realsense_context, "-4/");
    const auto serials = QueryRealsenseSerials(realsense_context);
    if (front_sn.empty() && !serials.empty())
    {
      front_sn = serials.front();
    }
    if (rear_sn.empty())
    {
      for (const auto& sn : serials)
      {
        if (sn != front_sn)
        {
          rear_sn = sn;
          break;
        }
      }
    }

    RRLIB_LOG_PRINT(USER, "RealSense mapping: front_sn=", front_sn, ", rear_sn=", rear_sn);

    auto realsense_front = new camera::intel_realsense::mD400(this, "RealSense D435 Front", realsense_context);
    ConfigureRealsenseDefaults(realsense_front);
    if (!front_sn.empty())
    {
      realsense_front->par_sn.Set(front_sn);
    }
    realsense_front->out_color.ConnectTo(this->so_front_stereo_camera_images);
    realsense_front->out_point_cloud.ConnectTo(this->so_front_stereo_point_cloud);

    if (!rear_sn.empty() && rear_sn != front_sn)
    {
      auto realsense_rear = new camera::intel_realsense::mD400(this, "RealSense D435 Rear", realsense_context);
      ConfigureRealsenseDefaults(realsense_rear);
      realsense_rear->par_sn.Set(rear_sn);
      realsense_rear->out_color.ConnectTo(this->so_rear_stereo_camera_images);
      realsense_rear->out_point_cloud.ConnectTo(this->so_rear_stereo_point_cloud);
    }
    else
    {
      RRLIB_LOG_PRINT(WARNING, "Rear RealSense not uniquely available. Falling back to front stream for rear ports.");
      realsense_front->out_color.ConnectTo(this->so_rear_stereo_camera_images);
      realsense_front->out_point_cloud.ConnectTo(this->so_rear_stereo_point_cloud);
    }
  }
  catch (const std::exception& e)
  {
    RRLIB_LOG_PRINT(ERROR, "Failed to initialize RealSense D435 Front module: ", e.what());
  }
#endif

}

//----------------------------------------------------------------------
// gHardwareInterface destructor
//----------------------------------------------------------------------
gHardwareInterface::~gHardwareInterface()
{
  // Force both indicators into safe OFF state on shutdown (also if last runtime command was blinking).
  ForceDisableIndicator(front_indicator_device_);
  if (rear_indicator_device_ != front_indicator_device_)
  {
    ForceDisableIndicator(rear_indicator_device_);
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
