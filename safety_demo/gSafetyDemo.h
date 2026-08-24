//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    projects/scout/safety_demo/gSafetyDemo.h
 *
 * \author  Patrick Wolf
 *
 * \date    2026-02-20
 *
 * \brief Contains gSafetyDemo
 *
 * \b gSafetyDemo
 *
 * Sense control group for safety demo at GRC (Scout should detect nearby persons and indicate the situational risk).
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__scout__safety_demo__gSafetyDemo_h__
#define __projects__scout__safety_demo__gSafetyDemo_h__

#include "plugins/structure/tSenseControlGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/distance_data/tDistanceData.h"
#include "rrlib/coviroa/tImage.h"
#include "rrlib/hid/tCH341UsbSignalLightDriver.h" // TODO: should be replaced by generic HID light mode type to make demo independent of specific driver

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
namespace safety_demo
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * Sense control group for safety demo at GRC (Scout should detect nearby persons and indicate the situational risk).
 */
class gSafetyDemo : public structure::tSenseControlGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tControllerOutput<rrlib::hid::ch341::tLightMode> co_led_indiactor_front_light_mode;
	tControllerOutput<rrlib::hid::ch341::tFlashFrequency> co_led_indiactor_front_flash_frequency;
	tControllerOutput<rrlib::hid::ch341::tBuzzerMode> co_led_indiactor_front_buzzer_mode;

	tControllerOutput<rrlib::hid::ch341::tLightMode> co_led_indiactor_rear_light_mode;
	tControllerOutput<rrlib::hid::ch341::tFlashFrequency> co_led_indiactor_rear_flash_frequency;
	tControllerOutput<rrlib::hid::ch341::tBuzzerMode> co_led_indiactor_rear_buzzer_mode;
	tVisualizationOutput<rrlib::coviroa::tImage, tLevelOfDetail::ALL> vo_front_perception_risk;
	tVisualizationOutput<rrlib::coviroa::tImage, tLevelOfDetail::ALL> vo_rear_perception_risk;
	tSensorOutput<float> so_front_perception_latency_ms;
	tSensorOutput<float> so_front_risk_latency_ms;
	tSensorOutput<float> so_front_actuation_latency_ms;
	tSensorOutput<float> so_front_total_latency_ms;
	tSensorOutput<float> so_rear_perception_latency_ms;
	tSensorOutput<float> so_rear_risk_latency_ms;
	tSensorOutput<float> so_rear_actuation_latency_ms;
	tSensorOutput<float> so_rear_total_latency_ms;

	tSensorInput<rrlib::distance_data::tDistanceData> si_front_stereo_point_cloud;
	// Original type kept for reference:
	// tSensorInput<rrlib::distance_data::tDistanceData> si_front_stereo_camera_images;
	tSensorInput<rrlib::coviroa::tImages> si_front_stereo_camera_images;
	tSensorInput<rrlib::distance_data::tDistanceData> si_rear_stereo_point_cloud;
	// Original type kept for reference:
	// tSensorInput<rrlib::distance_data::tDistanceData> si_rear_stereo_camera_images;
	tSensorInput<rrlib::coviroa::tImages> si_rear_stereo_camera_images;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gSafetyDemo(core::tFrameworkElement *parent, const std::string &name = "SafetyDemo",
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
  virtual ~gSafetyDemo();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
