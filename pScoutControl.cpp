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
/*!\file    projects/scout/pScoutControl.cpp
 *
 * \author  Patrick Wolf
 *
 * \date    2026-02-19
 *
 * \brief Contains mScoutControl
 *
 * \b pScoutControl
 *
 * Part to start the control of robot scout.
 *
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <chrono>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/scout/gScoutControl.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This program executes the ScoutControl module/group.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{}

//----------------------------------------------------------------------
// CreateMainGroup
//----------------------------------------------------------------------
void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
{
  auto main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);
  // 30 Hz main loop for smoother live updates in Finstruct.
  main_thread->SetCycleTime(std::chrono::milliseconds(33));

  new finroc::scout::gScoutControl(main_thread, "Scout Control");

}
