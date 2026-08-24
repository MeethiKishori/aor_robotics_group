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
/*!\file    projects/scout/safety_demo/mPercept.h
 *
 *
 * \author  
 *
 * \date    2026-02-19
 * \brief Perception pre-processing for risk assessment.
 */

//----------------------------------------------------------------------
#ifndef __projects__scout__safety_demo__mPercept_h__
#define __projects__scout__safety_demo__mPercept_h__

#include "plugins/structure/tModule.h"

#include "rrlib/coviroa/tImage.h"
#include "rrlib/distance_data/tDistanceData.h"
#include "rrlib/machine_learning_appliance/ml_definitions.h"
#include <chrono>
#include <memory>
#include <vector>

namespace finroc { namespace object_detection { class tDarknetYOLO; } }

namespace finroc
{
namespace scout
{
namespace safety_demo
{

//! One detection with a free-form string class label. Unlike tDarknetDetection
//! (fixed COCO enum) this can carry any label, e.g. "robot" from a custom model.
struct tLabeledDetection
{
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;           //!< bounding box in image pixels
  std::string label;        //!< e.g. "person", "robot"
  int class_id = -1;        //!< raw network class id
  float probability = 0.0f;
};

class mPercept : public structure::tModule
{
public:
  tInput<std::vector<rrlib::machine_learning_appliance::tDarknetDetection>> in_detections;
  tInput<rrlib::coviroa::tImages> in_images;
  tInput<rrlib::distance_data::tDistanceData> in_point_cloud;

  tOutput<float> out_nearest_distance_m;
  tOutput<int> out_nearest_class_id;
  tOutput<std::string> out_nearest_class;
  tOutput<float> out_target_class_distance_m;
  tOutput<int> out_target_class_count;
  tOutput<int> out_total_detections;
  tOutput<std::string> out_visible_classes;
  tOutput<double> out_pipeline_start_ms;
  tOutput<float> out_perception_latency_ms;
  tOutput<std::vector<rrlib::machine_learning_appliance::tDarknetDetection>> out_detections;
  // Per-object list (label + bbox) with an index-aligned distance vector, for
  // downstream per-object tracking / time-to-collision risk (mRiskAssessment).
  // Reuses rrlib's string-labelled detection type instead of a new custom
  // struct, since it already supports arbitrary labels (e.g. "robot") and is
  // already wire-serializable.
  tOutput<std::vector<rrlib::machine_learning_appliance::tMLStringDetection2D<float>>> out_ranged_detections;
  tOutput<std::vector<float>> out_ranged_distances_m;
  tVisualizationOutput<rrlib::coviroa::tImage, tLevelOfDetail::ALL> out_visualization;
  tVisualizationOutput<rrlib::coviroa::tImage, tLevelOfDetail::ALL> out_visualization_classes;

  tParameter<std::string> par_target_class_name;
  tParameter<std::string> par_darknet_config;      //!< person/COCO model
  tParameter<std::string> par_darknet_weights;
  tParameter<float> par_darknet_threshold;
  tParameter<bool> par_print_detection_log;
  tParameter<float> par_max_inference_fps;
  tParameter<float> par_risk_roi_length_m;
  tParameter<float> par_risk_roi_width_m;

  // Second detector: custom-trained Darknet model (e.g. Unitree robot).
  // Leave config/weights empty to disable it and run person detection only.
  tParameter<std::string> par_robot_config;
  tParameter<std::string> par_robot_weights;
  tParameter<std::string> par_robot_class_name;    //!< label for all robot-model detections

  // Optional external detector: when enabled, send frames to a Python YOLO
  // server over TCP and use the returned boxes instead of the local Darknet
  // models (Finroc runs no YOLO itself in this mode).
  tParameter<bool> par_use_python_detector;
  tParameter<std::string> par_python_host;
  tParameter<int> par_python_port;

  //! false pauses this percept entirely (no detection, no camera processing).
  tParameter<bool> par_enabled;

  mPercept(core::tFrameworkElement* parent, const std::string& name = "Percept");

protected:
  virtual ~mPercept();

private:
  std::string loaded_darknet_config_;
  std::string loaded_darknet_weights_;
  std::string loaded_robot_config_;
  std::string loaded_robot_weights_;
  std::chrono::steady_clock::time_point last_inference_time_;
  bool has_last_inference_time_ = false;
  std::vector<tLabeledDetection> cached_detections_;   //!< merged person + robot detections
  bool has_cached_detections_ = false;

  // YOLO wrappers — created lazily when their config/weights are valid.
  std::unique_ptr<finroc::object_detection::tDarknetYOLO> darknet_yolo_;    //!< person/COCO
  std::unique_ptr<finroc::object_detection::tDarknetYOLO> darknet_robot_;   //!< custom robot

  int python_socket_fd_ = -1;   //!< TCP connection to the Python YOLO server (-1 = not connected)

  virtual void Update() override;
};

}
}
}

#endif
