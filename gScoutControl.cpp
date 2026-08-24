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
/*!\file    projects/scout/gScoutControl.cpp
 *
 * \author  Patrick Wolf
 *
 * \date    2026-02-19
 *
 */
//----------------------------------------------------------------------
#include "projects/scout/gScoutControl.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/scout/hardware/gHardwareInterface.h"
#include "projects/scout/safety_demo/gSafetyDemo.h"

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<gScoutControl> cCREATE_ACTION_FOR_G_SCOUTCONTROL("ScoutControl");
#endif

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gScoutControl constructor
//----------------------------------------------------------------------
gScoutControl::gScoutControl(core::tFrameworkElement *parent, const std::string &name,
                             const std::string &structure_config_file) :
  tGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
	auto hardware_interface = new hardware::gHardwareInterface(this, "Hardware Interface");

	auto safety_demo = new safety_demo::gSafetyDemo(this, "Safety Demo");
	safety_demo->co_led_indiactor_front_buzzer_mode.ConnectTo(hardware_interface->ci_led_indiactor_front_buzzer_mode);
	safety_demo->co_led_indiactor_front_flash_frequency.ConnectTo(hardware_interface->ci_led_indiactor_front_flash_frequency);
	safety_demo->co_led_indiactor_front_light_mode.ConnectTo(hardware_interface->ci_led_indiactor_front_light_mode);
	safety_demo->co_led_indiactor_rear_buzzer_mode.ConnectTo(hardware_interface->ci_led_indiactor_rear_buzzer_mode);
	safety_demo->co_led_indiactor_rear_flash_frequency.ConnectTo(hardware_interface->ci_led_indiactor_rear_flash_frequency);
	safety_demo->co_led_indiactor_rear_light_mode.ConnectTo(hardware_interface->ci_led_indiactor_rear_light_mode);

#ifdef _LIB_FINROC_LIBRARIES_CAMERA_INTEL_REALSENSE_CAMERAS_PRESENT_
	safety_demo->si_front_stereo_camera_images.ConnectTo(hardware_interface->so_front_stereo_camera_images);
	safety_demo->si_rear_stereo_camera_images.ConnectTo(hardware_interface->so_rear_stereo_camera_images);
	safety_demo->si_front_stereo_point_cloud.ConnectTo(hardware_interface->so_front_stereo_point_cloud);
	safety_demo->si_rear_stereo_point_cloud.ConnectTo(hardware_interface->so_rear_stereo_point_cloud);
#endif

}

//----------------------------------------------------------------------
// gScoutControl destructor
//----------------------------------------------------------------------
gScoutControl::~gScoutControl()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
