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
//----------------------------------------------------------------------
/*!\file    projects/scout/safety_demo/mRiskToLampIndicator.h
 *
 * \brief   Maps integer risk levels to CH341 signal lamp commands.
 */
//----------------------------------------------------------------------
#ifndef __projects__scout__safety_demo__mRiskToLampIndicator_h__
#define __projects__scout__safety_demo__mRiskToLampIndicator_h__

#include "plugins/structure/tModule.h"
#include "rrlib/hid/tCH341UsbSignalLightDriver.h"

#include <chrono>

namespace finroc
{
namespace scout
{
namespace safety_demo
{

class mRiskToLampIndicator : public structure::tModule
{
public:
  tInput<int> in_risk_level;
  tInput<std::string> in_nearest_class;
  tInput<double> in_pipeline_start_ms;
  tInput<float> in_perception_latency_ms;
  tInput<float> in_risk_latency_ms;

  tOutput<rrlib::hid::ch341::tLightMode> out_light_mode;
  tOutput<rrlib::hid::ch341::tFlashFrequency> out_flash_frequency;
  tOutput<rrlib::hid::ch341::tBuzzerMode> out_buzzer_mode;
  tOutput<float> out_actuation_latency_ms;
  tOutput<float> out_total_latency_ms;

  tParameter<rrlib::hid::ch341::tLightMode> par_light_risk_0;
  tParameter<rrlib::hid::ch341::tLightMode> par_light_risk_1;
  tParameter<rrlib::hid::ch341::tLightMode> par_light_risk_2;
  tParameter<rrlib::hid::ch341::tLightMode> par_light_risk_3;
  tParameter<rrlib::hid::ch341::tLightMode> par_light_risk_4;

  tParameter<rrlib::hid::ch341::tFlashFrequency> par_flash_risk_0;
  tParameter<rrlib::hid::ch341::tFlashFrequency> par_flash_risk_1;
  tParameter<rrlib::hid::ch341::tFlashFrequency> par_flash_risk_2;
  tParameter<rrlib::hid::ch341::tFlashFrequency> par_flash_risk_3;
  tParameter<rrlib::hid::ch341::tFlashFrequency> par_flash_risk_4;

  tParameter<int> par_buzzer_on_from_risk;
  tParameter<std::string> par_fast_blink_class_name;
  tParameter<float> par_camera_timeout_ms;
  tParameter<rrlib::hid::ch341::tLightMode> par_light_camera_unavailable;

  // Minimum time the light must hold a level before dropping to a less
  // urgent one, so brief upstream jitter (flickering detections) doesn't
  // flicker the physical light. Escalating to a MORE urgent level is never
  // delayed by this -- only de-escalation waits.
  tParameter<float> par_min_hold_ms;

  mRiskToLampIndicator(core::tFrameworkElement *parent, const std::string &name = "Risk To Lamp Indicator");

protected:
  ~mRiskToLampIndicator();

private:
  virtual void Update() override;

  int last_published_level_ = -1;   //!< -1 = nothing shown yet (first Update() always applies)
  std::chrono::steady_clock::time_point last_change_time_;
  bool has_last_change_ = false;
};

}
}
}

#endif
