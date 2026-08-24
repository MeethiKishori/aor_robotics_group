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
/*!\file    projects/scout/gScoutControl.h
 *
 * \author  Patrick Wolf
 *
 * \date    2026-02-19
 *
 * \brief Contains gScoutControl
 *
 * \b gScoutControl
 *
 * Corresponding control group to run robot scout.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__scout__gScoutControl_h__
#define __projects__scout__gScoutControl_h__

#include "plugins/structure/tGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * Corresponding control group to run robot scout.
 */
class gScoutControl : public structure::tGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gScoutControl(core::tFrameworkElement *parent, const std::string &name = "ScoutControl",
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
  virtual ~gScoutControl();

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



#endif
