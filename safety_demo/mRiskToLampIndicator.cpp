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
/*!\file    projects/scout/safety_demo/mRiskToLampIndicator.cpp
 *
 * \brief   Maps integer risk levels to CH341 signal lamp commands.
 */
//----------------------------------------------------------------------
#include "projects/scout/safety_demo/mRiskToLampIndicator.h"

#include <algorithm>
#include <chrono>

namespace finroc
{
namespace scout
{
namespace safety_demo
{

namespace
{

inline double NowSteadyMs()
{
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

}

runtime_construction::tStandardCreateModuleAction<mRiskToLampIndicator> cCREATE_ACTION_FOR_M_RISK_TO_LAMP_INDICATOR("RiskToLampIndicator");

mRiskToLampIndicator::mRiskToLampIndicator(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false),
  in_risk_level("Risk Level", this),
  in_nearest_class("Nearest Class", this),
  in_pipeline_start_ms("Pipeline Start [ms]", this),
  in_perception_latency_ms("Perception Latency [ms]", this),
  in_risk_latency_ms("Risk Latency [ms]", this),
  out_light_mode("Light Mode", this),
  out_flash_frequency("Flash Frequency", this),
  out_buzzer_mode("Buzzer Mode", this),
  out_actuation_latency_ms("Actuation Latency [ms]", this),
  out_total_latency_ms("Total Latency [ms]", this),
  // Each level gets its own distinct color (0-4: off/green/blue/magenta/red)
  // so HIGH and EXTREME no longer look identical on the physical light.
  par_light_risk_0("Light Mode Risk 0", this, rrlib::hid::ch341::tLightMode::eLIGHT_OFF),
  par_light_risk_1("Light Mode Risk 1", this, rrlib::hid::ch341::tLightMode::eGREEN),
  par_light_risk_2("Light Mode Risk 2", this, rrlib::hid::ch341::tLightMode::eBLUE),
  par_light_risk_3("Light Mode Risk 3", this, rrlib::hid::ch341::tLightMode::eMAGENTA),
  par_light_risk_4("Light Mode Risk 4", this, rrlib::hid::ch341::tLightMode::eRED),
  // Flashing off at every level by default -- color alone distinguishes levels.
  par_flash_risk_0("Flash Frequency Risk 0", this, rrlib::hid::ch341::tFlashFrequency::eNO_FLASH),
  par_flash_risk_1("Flash Frequency Risk 1", this, rrlib::hid::ch341::tFlashFrequency::eNO_FLASH),
  par_flash_risk_2("Flash Frequency Risk 2", this, rrlib::hid::ch341::tFlashFrequency::eNO_FLASH),
  par_flash_risk_3("Flash Frequency Risk 3", this, rrlib::hid::ch341::tFlashFrequency::eNO_FLASH),
  par_flash_risk_4("Flash Frequency Risk 4", this, rrlib::hid::ch341::tFlashFrequency::eNO_FLASH),
  // Sound the buzzer from high risk onward.
  par_buzzer_on_from_risk("Buzzer On From Risk Level", this, 99),
  par_fast_blink_class_name("Fast Blink Class Name", this, "person"),
  par_camera_timeout_ms("Camera Timeout [ms]", this, 1500.0f),
  // Some CH341 tower-light variants have red/blue channels swapped.
  // Keep the default on red so the highest-risk state is visually urgent.
  par_light_camera_unavailable("Light Mode Camera Unavailable", this, rrlib::hid::ch341::tLightMode::eRED),
  par_min_hold_ms("Min Hold [ms]", this, 500.0f)
{
}

mRiskToLampIndicator::~mRiskToLampIndicator()
{
}

void mRiskToLampIndicator::Update()
{
  const double actuation_stage_start_ms = NowSteadyMs();
  const double pipeline_start_ms = in_pipeline_start_ms.Get();
  const auto ts = in_pipeline_start_ms.GetTimestamp();

  const float timeout_ms = std::max(0.0f, par_camera_timeout_ms.Get());
  const bool has_pipeline_timestamp = (pipeline_start_ms > 0.0);
  const bool camera_data_fresh = has_pipeline_timestamp && ((actuation_stage_start_ms - pipeline_start_ms) <= timeout_ms);

  if (!camera_data_fresh)
  {
    // Hard safety override, shown immediately -- and reset the hold state so
    // that once the camera comes back, the first fresh reading always applies
    // right away instead of possibly being held down by a stale timer.
    has_last_change_ = false;
    out_light_mode.Publish(par_light_camera_unavailable.Get(), ts);
    out_flash_frequency.Publish(rrlib::hid::ch341::tFlashFrequency::eNO_FLASH, ts);
    out_buzzer_mode.Publish(rrlib::hid::ch341::tBuzzerMode::eOFF, ts);
    out_actuation_latency_ms.Publish(static_cast<float>(NowSteadyMs() - actuation_stage_start_ms), ts);
    out_total_latency_ms.Publish(-1.0f, ts);
    return;
  }

  const int raw_level = std::clamp(in_risk_level.Get(), 0, 4);

  // Rise instantly (more danger must never be delayed); hold the current
  // level for par_min_hold_ms before dropping to a less urgent one, so brief
  // upstream jitter (e.g. a flickering detection) doesn't flicker the light.
  const auto now = std::chrono::steady_clock::now();
  int displayed_level = raw_level;
  if (!has_last_change_)
  {
    last_published_level_ = raw_level;
    last_change_time_ = now;
    has_last_change_ = true;
  }
  else
  {
    if (raw_level > last_published_level_)
    {
      last_published_level_ = raw_level;
      last_change_time_ = now;
    }
    else if (raw_level < last_published_level_)
    {
      const float held_ms = std::chrono::duration<float, std::milli>(now - last_change_time_).count();
      if (held_ms >= par_min_hold_ms.Get())
      {
        last_published_level_ = raw_level;
        last_change_time_ = now;
      }
    }
    displayed_level = last_published_level_;
  }

  rrlib::hid::ch341::tLightMode light_mode = par_light_risk_0.Get();
  rrlib::hid::ch341::tFlashFrequency flash = par_flash_risk_0.Get();
  if (displayed_level == 1)
  {
    light_mode = par_light_risk_1.Get();
    flash = par_flash_risk_1.Get();
  }
  else if (displayed_level == 2)
  {
    light_mode = par_light_risk_2.Get();
    flash = par_flash_risk_2.Get();
  }
  else if (displayed_level == 3)
  {
    light_mode = par_light_risk_3.Get();
    flash = par_flash_risk_3.Get();
  }
  else if (displayed_level == 4)
  {
    light_mode = par_light_risk_4.Get();
    flash = par_flash_risk_4.Get();
  }

  const bool buzzer_on = (displayed_level >= std::max(0, par_buzzer_on_from_risk.Get()));
  const auto buzzer_mode = buzzer_on ? rrlib::hid::ch341::tBuzzerMode::eON : rrlib::hid::ch341::tBuzzerMode::eOFF;
  (void)in_perception_latency_ms.Get();
  (void)in_risk_latency_ms.Get();

  out_light_mode.Publish(light_mode, ts);
  out_flash_frequency.Publish(flash, ts);
  out_buzzer_mode.Publish(buzzer_mode, ts);
  out_actuation_latency_ms.Publish(static_cast<float>(NowSteadyMs() - actuation_stage_start_ms), ts);
  out_total_latency_ms.Publish(static_cast<float>(NowSteadyMs() - pipeline_start_ms), ts);
}

}
}
}
