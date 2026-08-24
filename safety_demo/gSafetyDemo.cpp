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
/*!\file    projects/scout/safety_demo/gSafetyDemo.cpp
 *
 * \author  Patrick Wolf
 *
 * \date    2026-02-20
 *
 */
//----------------------------------------------------------------------
#include "projects/scout/safety_demo/gSafetyDemo.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "projects/scout/safety_demo/mRiskAssessment.h"
#include "projects/scout/safety_demo/mRiskToLampIndicator.h"
#include "projects/scout/safety_demo/mPercept.h"
#include "projects/scout/safety_demo/mRiskVisualization.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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
// Const values
//----------------------------------------------------------------------
#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<gSafetyDemo> cCREATE_ACTION_FOR_G_SAFETYDEMO("SafetyDemo");
#endif

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gSafetyDemo constructor
//----------------------------------------------------------------------
gSafetyDemo::gSafetyDemo(core::tFrameworkElement *parent, const std::string &name,
                         const std::string &structure_config_file) :
  tSenseControlGroup(parent, name, structure_config_file, false)
{
  auto front_percept = new mPercept(this, "Front Percept");
  front_percept->par_target_class_name.Set("person");
  front_percept->par_darknet_config.Set("../finroc_projects_scout/third_party/darknet/cfg/yolov4-tiny.cfg");
  front_percept->par_darknet_weights.Set("../finroc_projects_scout/third_party/darknet/yolov4-tiny.weights");
  front_percept->par_robot_config.Set("../finroc_projects_scout/third_party/darknet/unitree-go1-tiny.cfg");
  front_percept->par_robot_weights.Set("../finroc_projects_scout/third_party/darknet/unitree-go1-tiny_best.weights");
  front_percept->par_robot_class_name.Set("robot");
  front_percept->par_darknet_threshold.Set(0.15f);
  front_percept->par_print_detection_log.Set(false);
  front_percept->par_max_inference_fps.Set(8.0f);
  front_percept->par_risk_roi_length_m.Set(2.0f);
  front_percept->par_risk_roi_width_m.Set(1.0f);
  front_percept->in_images.ConnectTo(this->si_front_stereo_camera_images);
  front_percept->in_point_cloud.ConnectTo(this->si_front_stereo_point_cloud);

  auto front_risk_assessment = new mRiskAssessment(this, "Front Risk Assessment");
  front_risk_assessment->in_nearest_distance_m.ConnectTo(front_percept->out_nearest_distance_m);
  front_risk_assessment->in_nearest_class_id.ConnectTo(front_percept->out_nearest_class_id);
  front_risk_assessment->in_nearest_class.ConnectTo(front_percept->out_nearest_class);
  front_risk_assessment->in_target_class_distance_m.ConnectTo(front_percept->out_target_class_distance_m);
  front_risk_assessment->in_target_class_count.ConnectTo(front_percept->out_target_class_count);
  front_risk_assessment->in_pipeline_start_ms.ConnectTo(front_percept->out_pipeline_start_ms);
  front_risk_assessment->in_perception_latency_ms.ConnectTo(front_percept->out_perception_latency_ms);
  front_risk_assessment->in_ranged_detections.ConnectTo(front_percept->out_ranged_detections);
  front_risk_assessment->in_ranged_distances_m.ConnectTo(front_percept->out_ranged_distances_m);
  this->so_front_perception_latency_ms.ConnectTo(front_percept->out_perception_latency_ms);
  this->so_front_risk_latency_ms.ConnectTo(front_risk_assessment->out_risk_latency_ms);

  auto front_indicator_mapping = new mRiskToLampIndicator(this, "Front Indicator Mapping");
  front_indicator_mapping->in_risk_level.ConnectTo(front_risk_assessment->out_risk_level);
  front_indicator_mapping->in_nearest_class.ConnectTo(front_risk_assessment->out_nearest_class);
  front_indicator_mapping->in_pipeline_start_ms.ConnectTo(front_risk_assessment->out_pipeline_start_ms);
  front_indicator_mapping->in_perception_latency_ms.ConnectTo(front_risk_assessment->out_perception_latency_ms);
  front_indicator_mapping->in_risk_latency_ms.ConnectTo(front_risk_assessment->out_risk_latency_ms);
  this->co_led_indiactor_front_light_mode.ConnectTo(front_indicator_mapping->out_light_mode);
  this->co_led_indiactor_front_flash_frequency.ConnectTo(front_indicator_mapping->out_flash_frequency);
  this->co_led_indiactor_front_buzzer_mode.ConnectTo(front_indicator_mapping->out_buzzer_mode);
  this->so_front_actuation_latency_ms.ConnectTo(front_indicator_mapping->out_actuation_latency_ms);
  this->so_front_total_latency_ms.ConnectTo(front_indicator_mapping->out_total_latency_ms);

  auto front_risk_visualization = new mRiskVisualization(this, "Front Risk Visualization");
  front_risk_visualization->par_enable_stream.Set(true);
  front_risk_visualization->in_image.ConnectTo(front_percept->out_visualization_classes);
  front_risk_visualization->in_risk_level.ConnectTo(front_risk_assessment->out_risk_level);
  front_risk_visualization->in_risk_label.ConnectTo(front_risk_assessment->out_risk_label);
  this->vo_front_perception_risk.ConnectTo(front_risk_visualization->out_image);

  auto rear_percept = new mPercept(this, "Rear Percept");
  rear_percept->par_target_class_name.Set("person");
  rear_percept->par_darknet_config.Set("../finroc_projects_scout/third_party/darknet/cfg/yolov3-tiny.cfg");
  rear_percept->par_darknet_weights.Set("../finroc_projects_scout/third_party/darknet/yolov3-tiny.weights");
  rear_percept->par_robot_config.Set("../finroc_projects_scout/third_party/darknet/unitree-go1-tiny.cfg");
  rear_percept->par_robot_weights.Set("../finroc_projects_scout/third_party/darknet/unitree-go1-tiny_best.weights");
  rear_percept->par_robot_class_name.Set("robot");
  rear_percept->par_darknet_threshold.Set(0.15f);
  rear_percept->par_print_detection_log.Set(false);
  rear_percept->par_max_inference_fps.Set(8.0f);
  rear_percept->par_risk_roi_length_m.Set(2.0f);
  rear_percept->par_risk_roi_width_m.Set(1.0f);
  rear_percept->in_images.ConnectTo(this->si_rear_stereo_camera_images);
  rear_percept->in_point_cloud.ConnectTo(this->si_rear_stereo_point_cloud);

  auto rear_risk_assessment = new mRiskAssessment(this, "Rear Risk Assessment");
  rear_risk_assessment->in_nearest_distance_m.ConnectTo(rear_percept->out_nearest_distance_m);
  rear_risk_assessment->in_nearest_class_id.ConnectTo(rear_percept->out_nearest_class_id);
  rear_risk_assessment->in_nearest_class.ConnectTo(rear_percept->out_nearest_class);
  rear_risk_assessment->in_target_class_distance_m.ConnectTo(rear_percept->out_target_class_distance_m);
  rear_risk_assessment->in_target_class_count.ConnectTo(rear_percept->out_target_class_count);
  rear_risk_assessment->in_pipeline_start_ms.ConnectTo(rear_percept->out_pipeline_start_ms);
  rear_risk_assessment->in_perception_latency_ms.ConnectTo(rear_percept->out_perception_latency_ms);
  rear_risk_assessment->in_ranged_detections.ConnectTo(rear_percept->out_ranged_detections);
  rear_risk_assessment->in_ranged_distances_m.ConnectTo(rear_percept->out_ranged_distances_m);
  this->so_rear_perception_latency_ms.ConnectTo(rear_percept->out_perception_latency_ms);
  this->so_rear_risk_latency_ms.ConnectTo(rear_risk_assessment->out_risk_latency_ms);

  auto rear_indicator_mapping = new mRiskToLampIndicator(this, "Rear Indicator Mapping");
  rear_indicator_mapping->in_risk_level.ConnectTo(rear_risk_assessment->out_risk_level);
  rear_indicator_mapping->in_nearest_class.ConnectTo(rear_risk_assessment->out_nearest_class);
  rear_indicator_mapping->in_pipeline_start_ms.ConnectTo(rear_risk_assessment->out_pipeline_start_ms);
  rear_indicator_mapping->in_perception_latency_ms.ConnectTo(rear_risk_assessment->out_perception_latency_ms);
  rear_indicator_mapping->in_risk_latency_ms.ConnectTo(rear_risk_assessment->out_risk_latency_ms);
  this->co_led_indiactor_rear_light_mode.ConnectTo(rear_indicator_mapping->out_light_mode);
  this->co_led_indiactor_rear_flash_frequency.ConnectTo(rear_indicator_mapping->out_flash_frequency);
  this->co_led_indiactor_rear_buzzer_mode.ConnectTo(rear_indicator_mapping->out_buzzer_mode);
  this->so_rear_actuation_latency_ms.ConnectTo(rear_indicator_mapping->out_actuation_latency_ms);
  this->so_rear_total_latency_ms.ConnectTo(rear_indicator_mapping->out_total_latency_ms);

  auto rear_risk_visualization = new mRiskVisualization(this, "Rear Risk Visualization");
  rear_risk_visualization->par_enable_stream.Set(true);
  rear_risk_visualization->in_image.ConnectTo(rear_percept->out_visualization_classes);
  rear_risk_visualization->in_risk_level.ConnectTo(rear_risk_assessment->out_risk_level);
  rear_risk_visualization->in_risk_label.ConnectTo(rear_risk_assessment->out_risk_label);
  this->vo_rear_perception_risk.ConnectTo(rear_risk_visualization->out_image);
}

//----------------------------------------------------------------------
// gSafetyDemo destructor
//----------------------------------------------------------------------
gSafetyDemo::~gSafetyDemo()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
