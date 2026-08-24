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
/*!\file    projects/scout/safety_demo/mRiskAssessment.cpp
 *
 * \brief   Risk estimation from percept outputs.
 */
//----------------------------------------------------------------------

#include "projects/scout/safety_demo/mRiskAssessment.h"

#include <algorithm> // provides max, clamp
#include <chrono> // provides clock, duration, steady_clock, time_point
#include <cmath>  // provides isfinite
#include <set> // provides set
#include <string> // provides string and standard library format < .. >

// name of class is finroc::scout::safety_demo::mRiskAssessment
// lines 42–120   anonymous namespace  →  private helpers (math, lookup tables)
// lines 122–309  the module itself    →  constructor, destructor, Update()


namespace finroc
{
namespace scout
{
namespace safety_demo
{

namespace // this is private to this .cpp file only, when no name is added
{

inline double NowSteadyMs() // Difference of how long this module took to run 
// Hint to compiler: "instead of a real function call, paste the body straight into the caller." doing this is fast
{
  using namespace std::chrono; // provides library tool in scope
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}
// steady_clock gives time , time_since_epoch gives since laptop booted, then in milliseconds, then difference via duration, then count gives number


inline std::string RiskLabelFromLevel(int level)
{
  switch (level)
  {
  case 4:
    return "EXTREME";
  case 3:
    return "HIGH";
  case 2:
    return "MEDIUM";
  case 1:
    return "LOW";
  default:
    return "NONE";
  }
}

// ── Risk model R = w(c) * g(d), ported from the Python yolo-hazard pipeline ──
// (risk/category_weights.py + risk/distance_risk.py + risk/runtime_risk_model.py).
// The velocity term (1 + alpha * v_tilde) is deferred until robot velocity is
// tracked; the Python prototype also runs with v_tilde = 0.

// w(c): LIVING (humans, animals) 3.0, DYNAMIC (vehicles, robots) 2.0, STATIC 1.0. 
// constexpr goes into compile time and every update needs new build 
constexpr float cWEIGHT_LIVING  = 3.0f;
constexpr float cWEIGHT_DYNAMIC = 2.0f;
constexpr float cWEIGHT_STATIC  = 1.0f;

// g(d): inverse-distance risk, bounded for numerical stability once an object
// is effectively touching the robot. d <= 0 means "no object / no valid
// distance" and carries no risk (the percept stage publishes 0.0 for both).
constexpr float cRISK_G_MAX = 30.0f; // gain max is 30.0f, so risk is 30.0f when distance is 0.4m or less
constexpr float cRISK_D_MIN = 0.4f; // distance min is 0.4m
constexpr float cRISK_D_MAX = 5.0f; // distance max is 5.0m

// (1 + alpha * v_tilde) velocity term, same shape as the Python prototype's
// object_risk(). v_tilde = velocity_mps / par_velocity_ref_mps (a tunable
// parameter now, not fixed), clamped so a fast approach can at most triple
// the base risk and a fast retreat can at least halve it, but never flips
// risk negative.
constexpr float cRISK_VELOCITY_ALPHA = 0.5f;

// Below this speed, treat velocity as sensor jitter, not real motion: ignored
// for EMA smoothing (line ~285) and for whether a track counts as "closing" (TTC).
constexpr float cVELOCITY_DEADBAND_MPS = 0.05f;

inline float DistanceRiskG(float distance_m)
{
  // guard rails below , actual is 1/d
  if (!std::isfinite(distance_m) || distance_m >= cRISK_D_MAX || distance_m <= 0.0f) // std::isfinite checks the number is not infinity or NaN.mpercept give 0.0 when nothing is detected
  {// rejects if NaN(sqrt(-1) or infinity, 
    return 0.0f;
  }
  if (distance_m <= cRISK_D_MIN)
  {
    return cRISK_G_MAX;
  }
  return 1.0f / distance_m;
}

inline bool IsAnimalClass(const std::string& cls)
{ // pass by ref and constant cls , returns if the class is in the set of animals
  static const std::set<std::string> animals =
  { // this creates the list once and alive for whole program, otherwise its created and destroyed every time the function is called
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe"
  };
  return animals.count(cls) > 0; // turns 0/1 into false/true
}

inline bool IsVehicleClass(const std::string& cls)
{
  static const std::set<std::string> vehicles =
  {
    "robot", "unitree-go1-robot" //, "bicycle", "car", "motorbike", "airplane", "bus", "train", "truck", "boat"
    // Custom-trained classes: the Unitree robot is a moving machine -> DYNAMIC.
    
  };
  return vehicles.count(cls) > 0;
}

}
// global object to create the module in the framework, with the name "RiskAssessment" and the class mRiskAssessment 
runtime_construction::tStandardCreateModuleAction<mRiskAssessment> cCREATE_ACTION_FOR_M_RISK_ASSESSMENT("RiskAssessment");

mRiskAssessment::mRiskAssessment(core::tFrameworkElement* parent, const std::string& name) :
 // class name, then same name(mRiskAssessment) () is constructor. then parent  and name of module
  tModule(parent, name, false), // base class
  in_nearest_distance_m("Nearest Distance [m]", this), // afterwards its all members of the class, with name and this points to mriskAssessment
  in_nearest_class_id("Nearest Class Id", this), // in for input, out for output, par for parameter
  in_nearest_class("Nearest Class", this),// eg. build a port with name "Nearest Class" in gui
  in_target_class_distance_m("Target Class Distance [m]", this),
  in_target_class_count("Target Class Count", this),
  in_pipeline_start_ms("Pipeline Start [ms]", this),
  in_perception_latency_ms("Perception Latency [ms]", this),
  in_ranged_detections("Ranged Detections", this),
  in_ranged_distances_m("Ranged Distances [m]", this),
  out_distance_estimate_m("Distance Estimate [m]", this),
  out_risk_level("Risk Level", this),
  out_risk_label("Risk Label", this),
  out_nearest_class("Nearest Class", this),
  out_target_class_count("Target Class Count", this),
  out_pipeline_start_ms("Pipeline Start [ms]", this),
  out_perception_latency_ms("Perception Latency [ms]", this),
  out_risk_latency_ms("Risk Latency [ms]", this),
  out_approach_velocity_mps("Approach Velocity [m/s]", this),
  out_time_to_collision_s("Time To Collision [s]", this),
  out_risk_score("Risk Score", this),
  par_distance_extreme_m("Distance Extreme [m]", this, 0.5f), // default value 0.5f
  par_distance_high_m("Distance High [m]", this, 1.0f),
  par_distance_medium_m("Distance Medium [m]", this, 1.5f),
  par_distance_low_m("Distance Low [m]", this, 2.0f),
  par_level_hysteresis_pct("Level Hysteresis [pct]", this, 0.15f),
  par_person_threshold_scale("Person Threshold Scale", this, 1.0f),
  par_vehicle_threshold_scale("Vehicle Threshold Scale", this, 1.0f),
  par_animal_threshold_scale("Animal Threshold Scale", this, 1.0f),
  par_object_threshold_scale("Object Threshold Scale", this, 1.0f),
  par_contact_from_level("Contact From Risk Level", this, 3),
  par_contact_distance_m("Contact Distance [m]", this, 0.15f),
  par_track_match_radius_px("Track Match Radius [px]", this, 120.0f),
  par_track_max_misses("Track Max Misses", this, 5),
  par_ttc_extreme_s("TTC Extreme [s]", this, 2.0f),
  par_ttc_high_s("TTC High [s]", this, 4.0f),
  par_velocity_ref_mps("Velocity Ref [m/s]", this, 0.2f)
{
}

mRiskAssessment::~mRiskAssessment() // destructor
{
}

void mRiskAssessment::Update()
{
  const double risk_stage_start_ms = NowSteadyMs();
  if (!(in_ranged_detections.HasChanged() || in_ranged_distances_m.HasChanged() ||
        in_target_class_count.HasChanged() ||
        in_pipeline_start_ms.HasChanged() || in_perception_latency_ms.HasChanged()))
  {
    return;
  }

  auto detections = in_ranged_detections.GetPointer();
  auto distances = in_ranged_distances_m.GetPointer();
  auto target_class_count = in_target_class_count.GetPointer();
  const double pipeline_start_ms = in_pipeline_start_ms.Get();
  const float perception_latency_ms = in_perception_latency_ms.Get();

  const auto ts = detections.GetTimestamp();

  // Class weight w(c), scaled by the per-class tuning parameters (all default
  // 1.0, so the default weights stay 3 / 2 / 1). Same lookup as before, now
  // usable per-track instead of only for "the nearest object".
  const auto ClassWeight = [this](const std::string& label) -> float
  {
    if (label == "person")
    {
      return cWEIGHT_LIVING * par_person_threshold_scale.Get();
    }
    if (IsAnimalClass(label))
    {
      return cWEIGHT_LIVING * par_animal_threshold_scale.Get();
    }
    if (IsVehicleClass(label))
    {
      return cWEIGHT_DYNAMIC * par_vehicle_threshold_scale.Get();
    }
    return cWEIGHT_STATIC * par_object_threshold_scale.Get();
  };

  // g(d) is the same as before, but now per-track instead of only for "the nearest object".
  const auto risk_threshold = [](float distance_par)
  {
    return cWEIGHT_LIVING / std::max(distance_par, cRISK_D_MIN);
  };

  // ── Match this frame's detections onto tracked objects ────────────────────
  // Each track keeps its own distance history across frames (matched by label
  // + how far its bbox centre moved), so its own velocity/TTC is real instead
  // of one shared "nearest" history that jumps whenever a different object
  // happens to be closest.
  const float match_radius_px = std::max(1.0f, par_track_match_radius_px.Get()); // reads pixels between box centers, like box moved between frames
  const float match_radius2 = match_radius_px * match_radius_px;
  const auto now = std::chrono::steady_clock::now();
  // Tracks touched (created or updated) this frame -- used below only to stop
  // two detections claiming the same track, and to know which tracks aged by
  // one miss. Risk scoring and "is anything present" use tracks_ itself
  // (further down), since a track can still count for a few frames after
  // dropping out of matched_ids.
  std::set<int> matched_ids;

  const size_t n = std::min(detections->size(), distances->size()); //list sent from mpercept of detections and distances
  for (size_t i = 0; i < n; ++i)
  {
    const auto& det = detections->at(i);
    const float dist = distances->at(i);
    const auto center = det.GetBBox().GetCenterPoint();
    const float cx = center.X();
    const float cy = center.Y();
    const std::string label = det.GetMLClass(); //  the detected class as a string, e.g. "person"

    // Closest not-yet-matched track with the same label, within range.
    int best_id = -1;
    float best_d2 = match_radius2;
    for (auto& kv : tracks_) // lookup table of all tracks, kv.first is the track id, kv.second is the track data
    {
      if (matched_ids.count(kv.first) || kv.second.label != label)
      {
        continue;
      }
      const float dx = kv.second.cx - cx;
      const float dy = kv.second.cy - cy;
      const float d2 = dx * dx + dy * dy;
      if (d2 <= best_d2)
      {
        best_d2 = d2;
        best_id = kv.first;
      }
    }
    if (best_id < 0)
    {
      best_id = next_track_id_++;
    }
    tTrack& track = tracks_[best_id];   // creates a fresh (default-initialized) entry if new
    track.label = label;

    // Velocity: only trust it when both this and the previous reading had
    // real depth and the track wasn't just re-acquired -- a depth dropout or
    // a fresh track must not look like a sudden stop or a velocity spike.
    if (dist > 0.0f && track.distance_m > 0.0f && track.misses == 0)
    {
      const float dt = std::chrono::duration<float>(now - track.last_update).count();
      if (dt > 1e-3f)
      {
        float raw_v = (track.distance_m - dist) / dt;   // + = closing in
        if (std::fabs(raw_v) < cVELOCITY_DEADBAND_MPS)   // dead-band, same as the Python prototype
        {
          raw_v = 0.0f;
        }
        track.velocity_mps = 0.2f * raw_v + 0.8f * track.velocity_mps;   // EMA, alpha=0.2
      }
    }
    else
    {
      track.velocity_mps = 0.0f;
    }

    if (dist > 0.0f)
    {
      track.distance_m = dist;
    }
    track.cx = cx;
    track.cy = cy;
    track.last_update = now;
    track.misses = 0;
    matched_ids.insert(best_id);
  }

  // Age out tracks not seen this frame; drop after too many consecutive misses
  // (a brief occlusion survives, a genuinely gone object eventually gets pruned).
  const int max_misses = std::max(0, par_track_max_misses.Get());
  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    if (!matched_ids.count(it->first))
    {
      if (++it->second.misses > max_misses)
      {
        it = tracks_.erase(it);
        continue;
      }
    }
    ++it;
  }

  // ── Per-object risk, then take the worst: R_env = max_i R_i ───────────────
  const float ttc_extreme_s = std::max(0.01f, par_ttc_extreme_s.Get());
  const float ttc_high_s    = std::max(ttc_extreme_s, par_ttc_high_s.Get());
  const float velocity_ref_mps = std::max(0.01f, par_velocity_ref_mps.Get());
  // par_distance_*_m can't change mid-Update(), so each threshold is worth
  // computing once and reusing below instead of re-deriving it per track.
  const float thr_extreme = risk_threshold(par_distance_extreme_m.Get());
  const float thr_high    = risk_threshold(par_distance_high_m.Get());
  const float thr_medium  = risk_threshold(par_distance_medium_m.Get());
  const float thr_low     = risk_threshold(par_distance_low_m.Get());

  // Score every track that survived the aging step above -- not just ones
  // matched THIS exact frame. A track within its miss budget (misses > 0 but
  // not yet pruned) still counts, scored off its last known distance/velocity,
  // so one dropped detection frame can't make a genuinely-still-there object
  // look like it vanished.
  float risk_env = 0.0f;
  int winning_id = -1;
  float winning_ttc_s = -1.0f;   // TTC of whichever track is currently winning
  for (const auto& kv : tracks_)
  {
    const int id = kv.first;
    const tTrack& t = kv.second;

    // Base term: R = w(c) * g(d) * (1 + alpha * v_tilde), same shape as the
    // Python prototype. A closing object counts extra; a receding one counts less.
    const float v_norm = std::clamp(t.velocity_mps / velocity_ref_mps, -1.0f, 2.0f);
    float r = ClassWeight(t.label) * DistanceRiskG(t.distance_m) * (1.0f + cRISK_VELOCITY_ALPHA * v_norm);

    // TTC (only meaningful while genuinely closing); also floors the risk:
    // g(d) alone is 0 once an object is farther than cRISK_D_MAX, but a
    // fast-closing far object is still dangerous -- force at least the
    // EXTREME/HIGH threshold so it cannot be ignored just because it is far away right now.
    float ttc_s = -1.0f;
    if (t.velocity_mps > cVELOCITY_DEADBAND_MPS && t.distance_m > 0.0f)
    {
      ttc_s = t.distance_m / t.velocity_mps;
      if (ttc_s < ttc_extreme_s)
      {
        r = std::max(r, thr_extreme);
      }
      else if (ttc_s < ttc_high_s)
      {
        r = std::max(r, thr_high);
      }
    }

    if (r > risk_env)
    {
      risk_env = r;
      winning_id = id;
      winning_ttc_s = ttc_s;
    }
  }

  // Raw level: a hysteresis "ladder" walk starting from the current level,
  // not a stateless lookup. Rising past a boundary needs risk_env to clear it
  // by par_level_hysteresis_pct; falling back needs to drop below it by that
  // same margin -- so noise sitting right on one boundary can't flip the
  // level back and forth by itself. A big, decisive change in risk_env can
  // still cross several levels in a single frame, in either direction.
  const float hysteresis = std::clamp(par_level_hysteresis_pct.Get(), 0.0f, 0.9f);
  const float level_threshold[5] = {0.0f, thr_low, thr_medium, thr_high, thr_extreme};
  int raw_level = risk_state_;
  while (raw_level < 4 && risk_env >= level_threshold[raw_level + 1] * (1.0f + hysteresis))
  {
    ++raw_level;
  }
  while (raw_level > 0 && risk_env < level_threshold[raw_level] * (1.0f - hysteresis))
  {
    --raw_level;
  }

  // Is anything still being tracked (matched this frame, or recently missed
  // but within its miss budget)? Only a track that has been pruned entirely
  // -- genuinely gone, not just one dropped frame -- counts as "not present".
  const bool object_present = !tracks_.empty();

  // Remember the distance of whichever track is actually driving risk, so the
  // CONTACT check below can ask "was it genuinely close", not just "was the
  // (possibly TTC-boosted) risk score high".
  if (winning_id >= 0)
  {
    last_known_distance_m_ = tracks_[winning_id].distance_m;
  }

  // ── Temporal state machine ──────────────────────────────────────────────
  // While an object is present, raw_level already IS the new state: it
  // started from risk_state_ and only moved when risk_env decisively cleared
  // a hysteresis band, so no separate smoothing is needed here. If the
  // signal is lost while a genuinely close object was being tracked, it most
  // likely moved into the sensor's blind zone (touching it) -> latch CONTACT
  // at max risk. Requiring real proximity (not just a high risk_state_) stops
  // a far-but-fast object (risk boosted by TTC) or a calmly withdrawn object
  // from falsely triggering CONTACT. Signal loss with no prior contact just
  // decays one step at a time (a briefly-lost track never snaps to "safe").
  const int contact_from = std::clamp(par_contact_from_level.Get(), 1, 4);
  if (object_present)
  {
    contact_latched_ = false;
    risk_state_ = raw_level;
  }
  else
  {
    const bool was_genuinely_close = last_known_distance_m_ > 0.0f &&
                                     last_known_distance_m_ <= par_contact_distance_m.Get();
    if (risk_state_ >= contact_from && was_genuinely_close)
    {
      risk_state_ = 4;                               // object entered blind zone
      contact_latched_ = true;
    }
    else if (!contact_latched_)
    {
      risk_state_ = std::max(0, risk_state_ - 1);    // decay toward none (empty scene)
    }
  }

  const int risk_level = risk_state_;
  const std::string risk_label = contact_latched_ ? std::string("CONTACT")
                                                  : RiskLabelFromLevel(risk_level);

  // ── Telemetry from whichever object actually drove R_env this frame ───────
  // (distance/class/velocity/ttc of the winning track, already found above)
  float out_distance = 0.0f;
  std::string out_class;
  float out_velocity = 0.0f;
  float out_ttc = winning_ttc_s;
  if (winning_id >= 0)
  {
    const tTrack& winner = tracks_[winning_id];
    out_distance = std::max(0.0f, winner.distance_m);
    out_class    = winner.label;
    out_velocity = winner.velocity_mps;
  }

  out_distance_estimate_m.Publish(out_distance, ts);
  out_risk_level.Publish(risk_level, ts);
  out_risk_label.Publish(risk_label, ts);
  out_nearest_class.Publish(out_class, ts);
  out_target_class_count.Publish(*target_class_count, ts);
  out_pipeline_start_ms.Publish(pipeline_start_ms, ts);
  out_perception_latency_ms.Publish(perception_latency_ms, ts);
  out_approach_velocity_mps.Publish(out_velocity, ts);
  out_time_to_collision_s.Publish(out_ttc, ts);
  out_risk_score.Publish(risk_env, ts);
  out_risk_latency_ms.Publish(static_cast<float>(NowSteadyMs() - risk_stage_start_ms), ts);
}

}
}
}
