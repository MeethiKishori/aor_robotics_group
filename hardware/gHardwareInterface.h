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
/*!\file    projects/scout/hardware/gHardwareInterface.h
 *
 * \author  Patrick Wolf
 *
 * \date    2026-02-19
 *
 * \brief Contains gHardwareInterface
 *
 * \b gHardwareInterface
 *
 * Corresponding hardware interface of robot scout 
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__scout__hardware__gHardwareInterface_h__
#define __projects__scout__hardware__gHardwareInterface_h__

#include "plugins/structure/tSenseControlGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/distance_data/tDistanceData.h"
#include "rrlib/coviroa/tImage.h"
#include "rrlib/hid/tCH341UsbSignalLightDriver.h" // 

//----------------------------------------------------------------------
// Internal includes with ""
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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * Corresponding hardware interface of robot scout 
 */
class gHardwareInterface : public structure::tSenseControlGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tControllerInput<rrlib::hid::ch341::tLightMode> ci_led_indiactor_front_light_mode;
	tControllerInput<rrlib::hid::ch341::tFlashFrequency> ci_led_indiactor_front_flash_frequency;
	tControllerInput<rrlib::hid::ch341::tBuzzerMode> ci_led_indiactor_front_buzzer_mode;

	tControllerInput<rrlib::hid::ch341::tLightMode> ci_led_indiactor_rear_light_mode;
	tControllerInput<rrlib::hid::ch341::tFlashFrequency> ci_led_indiactor_rear_flash_frequency;
	tControllerInput<rrlib::hid::ch341::tBuzzerMode> ci_led_indiactor_rear_buzzer_mode;

#ifdef _LIB_FINROC_LIBRARIES_LASER_SCANNER_ROBOSENSE_PRESENT_
	tSensorOutput<rrlib::distance_data::tDistanceData> so_3d_laser;
#endif
#ifdef _LIB_FINROC_LIBRARIES_CAMERA_INTEL_REALSENSE_CAMERAS_PRESENT_
	tSensorOutput<rrlib::distance_data::tDistanceData> so_front_stereo_point_cloud;
	tSensorOutput<rrlib::coviroa::tImages> so_front_stereo_camera_images;
	tSensorOutput<rrlib::distance_data::tDistanceData> so_rear_stereo_point_cloud;
	tSensorOutput<rrlib::coviroa::tImages> so_rear_stereo_camera_images;
#endif

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gHardwareInterface(core::tFrameworkElement *parent, const std::string &name = "HardwareInterface",
                     const std::string &structure_config_file = __FILE__".xml");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of groups is declared protected to avoid accidental deletion. Deleting
   * groups is already handled by the framework.
   */
  virtual ~gHardwareInterface();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:
  std::string front_indicator_device_;
  std::string rear_indicator_device_;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
