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
/*!\file    projects/scout/safety_demo/mRiskAssessment.h
*\author  Catharina Helten
 *
 *\date    2026-02-24
 *
 *\brief   Risk estimation from percept outputs.
 */
//----------------------------------------------------------------------
#ifndef __projects__scout__safety_demo__mRiskAssessment_h__
#define __projects__scout__safety_demo__mRiskAssessment_h__

#include "plugins/structure/tModule.h"

#include "rrlib/machine_learning_appliance/ml_definitions.h"

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

namespace finroc
{
namespace scout
{
namespace safety_demo
{

class mRiskAssessment : public structure::tModule
{
public:
  // tInput<std::vector<rrlib::machine_learning_appliance::tDarknetDetection>> in_detections;
  tInput<float> in_nearest_distance_m; // tinput is a type of port to get data from other modules
  tInput<int> in_nearest_class_id;
  tInput<std::string> in_nearest_class;
  tInput<float> in_target_class_distance_m;
  tInput<int> in_target_class_count;
  tInput<double> in_pipeline_start_ms;
  tInput<float> in_perception_latency_ms;

  // Per-object detections (label + bbox) and an index-aligned distance list.
  // Enables tracking each object across frames instead of collapsing the
  // frame to a single "nearest" scalar, so a far-but-fast-approaching object
  // can be judged by its own time-to-collision rather than being ignored
  // just because something else is currently closer.
  tInput<std::vector<rrlib::machine_learning_appliance::tMLStringDetection2D<float>>> in_ranged_detections;
  tInput<std::vector<float>> in_ranged_distances_m;

  tOutput<float> out_distance_estimate_m;
  tOutput<int> out_risk_level;
  tOutput<std::string> out_risk_label;
  tOutput<std::string> out_nearest_class;
  tOutput<int> out_target_class_count;
  tOutput<double> out_pipeline_start_ms;
  tOutput<float> out_perception_latency_ms;
  tOutput<float> out_risk_latency_ms;
  // Task-2 groundwork: closing speed and time-to-collision (published for
  // telemetry; not yet folded into the risk level).
  tOutput<float> out_approach_velocity_mps;
  tOutput<float> out_time_to_collision_s;

  // Raw risk score (risk_env, i.e. R = w(c)*g(d)*(1+alpha*v_tilde), possibly
  // TTC-floored) of the winning track -- the actual number the 0-4 level and
  // the hysteresis ladder are computed from, visible live instead of only the
  // already-discretized risk_level/risk_label. Compare against the
  // par_distance_*_m parameters (converted via 3.0/distance) to see why.
  tOutput<float> out_risk_score;

  tParameter<float> par_distance_extreme_m;
  tParameter<float> par_distance_high_m;
  tParameter<float> par_distance_medium_m;
  tParameter<float> par_distance_low_m;

  // Hysteresis band around each of the four distance thresholds above, as a
  // fraction (0.15 = 15%). Crossing a boundary requires risk_env to clear it
  // by this margin in the direction of travel, so measurement noise sitting
  // right on a boundary can't flip the level back and forth by itself.
  tParameter<float> par_level_hysteresis_pct;

  // Class-aware scaling (editable in Finstruct): < 1.0 => more conservative risk.
  tParameter<float> par_person_threshold_scale;
  tParameter<float> par_vehicle_threshold_scale;
  tParameter<float> par_animal_threshold_scale;
  tParameter<float> par_object_threshold_scale;

  // Signal loss (nothing detected, no depth) at or above this level, AND the
  // last tracked object was genuinely within par_contact_distance_m, is
  // treated as the object entering the sensor's blind zone (touching it) and
  // latches CONTACT at maximum risk. Both conditions are required so a
  // far-but-fast object (risk boosted by TTC, never actually close) or a
  // calmly withdrawn object can't falsely trigger CONTACT.
  tParameter<int> par_contact_from_level;
  tParameter<float> par_contact_distance_m;

  // Multi-object tracking: how far (pixels) a detection may move between
  // frames and still count as the same tracked object, and how many
  // consecutive frames it may go unseen before its track is dropped.
  tParameter<float> par_track_match_radius_px;
  tParameter<int> par_track_max_misses;

  // Time-to-collision risk boost (mirrors the Python prototype's
  // compute_ttc_risk): a track whose TTC drops below these thresholds is
  // forced to at least the EXTREME / HIGH risk level, even if its raw
  // distance-based risk alone is still low because the object is far away.
  tParameter<float> par_ttc_extreme_s;
  tParameter<float> par_ttc_high_s;

  // Reference speed for the (1 + alpha * v_tilde) velocity term: velocity_mps
  // is divided by this to get v_tilde (so this speed maps to v_tilde = 1.0).
  tParameter<float> par_velocity_ref_mps;

  mRiskAssessment(core::tFrameworkElement* parent, const std::string& name = "Risk Assessment");

protected:
  virtual ~mRiskAssessment();

private:
  virtual void Update() override;

  // ── One tracked object, matched across frames by label + bbox proximity ───
  struct tTrack
  {
    std::string label;
    float distance_m = -1.0f;    //!< last known distance (m), <= 0 = unknown
    float velocity_mps = 0.0f;   //!< smoothed approach velocity; + = closing in
    float cx = 0.0f;
    float cy = 0.0f;             //!< last bbox centre, image pixels
    std::chrono::steady_clock::time_point last_update;
    int misses = 0;              //!< consecutive frames without a match
  };
  std::unordered_map<int, tTrack> tracks_;
  int next_track_id_ = 0;

  // ── Temporal risk state (persists across Update() calls) ──────────────────
  int   risk_state_ = 0;               //!< smoothed risk level 0..4
  bool  contact_latched_ = false;      //!< current high state came from signal loss
  float last_known_distance_m_ = -1.0f; //!< distance of the last risk-driving track, for the CONTACT check
};

}
}
}

#endif
