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
/*!\file    projects/scout/safety_demo/mPercept.cpp
 *
 * \brief   Perception pre-processing for risk assessment.
 */
//----------------------------------------------------------------------
#include "projects/scout/safety_demo/mPercept.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "libraries/object_detection/tDarknetYOLO.h"

#include <opencv2/imgcodecs.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>

namespace finroc
{
namespace scout
{
namespace safety_demo
{

namespace
{

void PublishUnavailableVisualizations(mPercept& module,
                                      const rrlib::coviroa::tImages& images,
                                      const rrlib::time::tTimestamp& ts,
                                      const cv::Mat& cam_image,
                                      const char* message)
{
  if (module.out_visualization.IsConnected())
  {
    auto visualization = module.out_visualization.GetUnusedBuffer();
    visualization->Resize(images.at(0).GetWidth(), images.at(0).GetHeight(), rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_BGR24, 0);
    auto vis_image = rrlib::coviroa::AccessImageAsMat(*visualization);
    cam_image.copyTo(vis_image);
    cv::putText(vis_image, message, cv::Point(16, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    visualization.SetTimestamp(ts);
    module.out_visualization.Publish(visualization);
  }
  if (module.out_visualization_classes.IsConnected())
  {
    auto classes_vis = module.out_visualization_classes.GetUnusedBuffer();
    classes_vis->Resize(images.at(0).GetWidth(), images.at(0).GetHeight(), rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_BGR24, 0);
    auto classes_vis_mat = rrlib::coviroa::AccessImageAsMat(*classes_vis);
    cam_image.copyTo(classes_vis_mat);
    cv::putText(classes_vis_mat, message, cv::Point(16, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    classes_vis.SetTimestamp(ts);
    module.out_visualization_classes.Publish(classes_vis);
  }
}

inline float ClampFloat(float v, float lo, float hi)
{
  return std::max(lo, std::min(v, hi));
}

inline bool IsFinite(float x)
{
  return std::isfinite(x);
}

inline float DistanceFromPoint(const rrlib::math::tVec3f& p)
{
  const float x = p.X();
  const float y = p.Y();
  const float z = p.Z();
  return std::sqrt(x * x + y * y + z * z);
}

inline std::string JoinVisibleClasses(const std::set<std::string>& classes)
{
  std::ostringstream out;
  bool first = true;
  for (const auto& c : classes)
  {
    if (!first)
    {
      out << ",";
    }
    out << c;
    first = false;
  }
  return out.str();
}

inline std::string ToLowerAscii(std::string s)
{
  for (char& c : s)
  {
    if (c >= 'A' && c <= 'Z')
    {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  return s;
}

inline int RiskLevelFromDistance(float d)
{
  if (!std::isfinite(d) || d < 0.0f)
  {
    return 0;
  }
  if (d < 0.5f)
  {
    return 4;
  }
  if (d < 1.0f)
  {
    return 3;
  }
  if (d < 1.5f)
  {
    return 2;
  }
  if (d < 2.0f)
  {
    return 1;
  }
  return 0;
}

inline const char* RiskLabelFromLevel(int level)
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

inline cv::Scalar RiskColorFromLevel(int level)
{
  switch (level)
  {
  case 4:
  case 3:
    return cv::Scalar(0, 0, 255);      // red
  case 2:
    return cv::Scalar(0, 255, 255);    // yellow
  case 1:
    return cv::Scalar(0, 255, 0);      // green
  default:
    return cv::Scalar(255, 0, 255);    // purple
  }
}

inline cv::Rect LabeledRect(const tLabeledDetection& d)
{
  return cv::Rect(d.x, d.y, d.width, d.height);
}

// Run one Darknet model and append its detections as labeled boxes.
// - forced_label: if non-empty, every box gets this label (used for the
//   single-class robot model, whose class id 0 would otherwise decode to "person").
// - keep_only: if non-empty, keep only detections whose natural class matches it
//   (used to restrict the COCO model to "person", like the Python classes=[0]).
void RunDetector(object_detection::tDarknetYOLO& yolo, const cv::Mat& cam_image,
                 const rrlib::time::tTimestamp& ts, float threshold, bool print_log,
                 const std::string& forced_label, const std::string& keep_only,
                 std::vector<tLabeledDetection>& out)
{
  cv::Mat vis_mat = cam_image.clone();
  const auto dets = yolo.ClassifyImage(cam_image, ts, vis_mat, threshold,
                                       /*draw_visualization=*/false, print_log, /*open_window=*/false);
  for (const auto& det : dets)
  {
    const std::string natural = ToLowerAscii(det.GetMLClassString());
    if (!keep_only.empty() && natural != keep_only)
    {
      continue;   // drop non-target classes (person-only filter)
    }
    const auto& bb = det.GetBBox();
    tLabeledDetection ld;
    ld.x = static_cast<int>(bb.Min().X());
    ld.y = static_cast<int>(bb.Min().Y());
    ld.width  = static_cast<int>(bb.Max().X()) - ld.x;
    ld.height = static_cast<int>(bb.Max().Y()) - ld.y;
    ld.class_id = static_cast<int>(det.GetMLClass());
    ld.probability = det.GetProbability();
    ld.label = forced_label.empty() ? natural : forced_label;
    out.push_back(ld);
  }
}

// The RealSense driver publishes an organized cloud (row-major depth grid with
// invalid points as zeros). Its grid size is not transported, so infer it from
// the point count: either it matches the camera image (depth aligned to color)
// or one of the common RealSense depth resolutions.
bool GuessCloudGrid(unsigned int dimension, int cam_w, int cam_h, int& cloud_w, int& cloud_h)
{
  if (cam_w > 0 && cam_h > 0 && dimension == static_cast<unsigned int>(cam_w) * static_cast<unsigned int>(cam_h))
  {
    cloud_w = cam_w;
    cloud_h = cam_h;
    return true;
  }
  static const int known_resolutions[][2] =
  {
    {1920, 1080}, {1280, 800}, {1280, 720}, {848, 480}, {640, 480}, {640, 360}, {424, 240}, {320, 240}
  };
  for (const auto& wh : known_resolutions)
  {
    if (dimension == static_cast<unsigned int>(wh[0]) * static_cast<unsigned int>(wh[1]))
    {
      cloud_w = wh[0];
      cloud_h = wh[1];
      return true;
    }
  }
  return false;
}

// Median distance (metres) over a small patch at the bbox centre, for noise
// robustness. Box is in camera image coordinates; the patch is sampled in
// cloud grid coordinates. Returns 0 when the patch holds no valid points.
float MedianDepthInBox(const rrlib::distance_data::tDistanceData& cloud, const cv::Rect& box,
                       int cam_w, int cam_h, int cloud_w, int cloud_h)
{
  const float sx = static_cast<float>(cloud_w) / static_cast<float>(cam_w);
  const float sy = static_cast<float>(cloud_h) / static_cast<float>(cam_h);
  const int cx = static_cast<int>((box.x + box.width / 2) * sx);
  const int cy = static_cast<int>((box.y + box.height / 2) * sy);
  const int pw = std::max(3, static_cast<int>(box.width * sx) / 5);
  const int ph = std::max(3, static_cast<int>(box.height * sy) / 5);
  const int x1 = std::max(0, cx - pw / 2);
  const int x2 = std::min(cloud_w - 1, cx + pw / 2);
  const int y1 = std::max(0, cy - ph / 2);
  const int y2 = std::min(cloud_h - 1, cy + ph / 2);
  if (x1 > x2 || y1 > y2)
  {
    return 0.0f;
  }

  float to_m = 1.0f;
  switch (cloud.Unit())
  {
  case rrlib::distance_data::eDISTANCE_UNIT_MM:
    to_m = 0.001f;
    break;
  case rrlib::distance_data::eDISTANCE_UNIT_CM:
    to_m = 0.01f;
    break;
  case rrlib::distance_data::eDISTANCE_UNIT_DM:
    to_m = 0.1f;
    break;
  default:
    break;
  }

  const auto* points = cloud.GetConstDataPtr<rrlib::math::tVec3f>();
  std::vector<float> values;
  values.reserve(static_cast<size_t>(x2 - x1 + 1) * static_cast<size_t>(y2 - y1 + 1));
  for (int yy = y1; yy <= y2; ++yy)
  {
    for (int xx = x1; xx <= x2; ++xx)
    {
      const float d = DistanceFromPoint(points[yy * cloud_w + xx]);
      if (d > 0.0f && IsFinite(d))
      {
        values.push_back(d);
      }
    }
  }
  if (values.empty())
  {
    return 0.0f;
  }
  std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
  return values[values.size() / 2] * to_m;
}

// ── Python TCP detector bridge ────────────────────────────────────────────
// Wire protocol (little-endian; localhost, same arch):
//   request : [uint32 jpeg_len][jpeg bytes]
//   response: [uint32 count] then count x
//             [int32 x][int32 y][int32 w][int32 h][float conf][int32 len][label]

int ConnectPythonServer(const std::string& host, int port)
{
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
  {
    return -1;
  }
  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0)
  {
    ::close(fd);
    return -1;
  }
  timeval tv;
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  int one = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    ::close(fd);
    return -1;
  }
  return fd;
}

bool SendAll(int fd, const void* data, size_t n)
{
  const char* p = static_cast<const char*>(data);
  for (size_t sent = 0; sent < n;)
  {
    const ssize_t r = ::send(fd, p + sent, n - sent, MSG_NOSIGNAL);
    if (r <= 0)
    {
      return false;
    }
    sent += static_cast<size_t>(r);
  }
  return true;
}

bool RecvAll(int fd, void* data, size_t n)
{
  char* p = static_cast<char*>(data);
  for (size_t got = 0; got < n;)
  {
    const ssize_t r = ::recv(fd, p + got, n - got, 0);
    if (r <= 0)
    {
      return false;
    }
    got += static_cast<size_t>(r);
  }
  return true;
}

// Send a frame, receive boxes. Returns false on any socket error (the caller
// then closes and reconnects on the next frame).
bool DetectViaPython(int fd, const cv::Mat& cam_image, std::vector<tLabeledDetection>& out)
{
  std::vector<uchar> jpeg;
  if (!cv::imencode(".jpg", cam_image, jpeg))
  {
    return false;
  }
  const uint32_t len = static_cast<uint32_t>(jpeg.size());
  if (!SendAll(fd, &len, sizeof(len)) || !SendAll(fd, jpeg.data(), jpeg.size()))
  {
    return false;
  }

  uint32_t count = 0;
  if (!RecvAll(fd, &count, sizeof(count)))
  {
    return false;
  }
  out.clear();
  out.reserve(count);
  for (uint32_t i = 0; i < count; ++i)
  {
    int32_t box[4];
    float conf = 0.0f;
    int32_t label_len = 0;
    if (!RecvAll(fd, box, sizeof(box)) || !RecvAll(fd, &conf, sizeof(conf)) ||
        !RecvAll(fd, &label_len, sizeof(label_len)))
    {
      return false;
    }
    std::string label;
    if (label_len > 0)
    {
      label.resize(static_cast<size_t>(label_len));
      if (!RecvAll(fd, &label[0], static_cast<size_t>(label_len)))
      {
        return false;
      }
    }
    tLabeledDetection d;
    d.x = box[0];
    d.y = box[1];
    d.width = box[2];
    d.height = box[3];
    d.probability = conf;
    d.class_id = -1;
    d.label = ToLowerAscii(label);
    out.push_back(d);
  }
  return true;
}

}

inline double NowSteadyMs()
{
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

runtime_construction::tStandardCreateModuleAction<mPercept> cCREATE_ACTION_FOR_M_PERCEPT("Percept");

mPercept::mPercept(core::tFrameworkElement* parent, const std::string& name) :
  tModule(parent, name, false),
  in_detections("Detections", this),
  in_images("Images", this),
  in_point_cloud("Point Cloud", this),
  out_nearest_distance_m("Nearest Distance [m]", this),
  out_nearest_class_id("Nearest Class Id", this),
  out_nearest_class("Nearest Class", this),
  out_target_class_distance_m("Target Class Distance [m]", this),
  out_target_class_count("Target Class Count", this),
  out_total_detections("Total Detections", this),
  out_visible_classes("Visible Classes", this),
  out_pipeline_start_ms("Pipeline Start [ms]", this),
  out_perception_latency_ms("Perception Latency [ms]", this),
  out_detections("Detections", this),
  out_ranged_detections("Ranged Detections", this),
  out_ranged_distances_m("Ranged Distances [m]", this),
  out_visualization("Visualization", this),
  out_visualization_classes("Visualization Classes", this),
  par_target_class_name("Target Class Name", this, "person"),
  par_darknet_config("Darknet Config", this, "sources/cpp/projects/scout/third_party/darknet/cfg/yolov3-tiny.cfg"),
  par_darknet_weights("Darknet Weights", this, "sources/cpp/projects/scout/third_party/darknet/yolov3-tiny.weights"),
  par_darknet_threshold("Darknet Threshold", this, 0.25f),
  par_print_detection_log("Print Detection Log", this, false),
  par_max_inference_fps("Max Inference FPS", this, 8.0f),
  par_risk_roi_length_m("Risk ROI Length [m]", this, 2.0f),
  par_risk_roi_width_m("Risk ROI Width [m]", this, 1.0f),
  par_robot_config("Robot Config", this, ""),
  par_robot_weights("Robot Weights", this, ""),
  par_robot_class_name("Robot Class Name", this, "robot"),
  par_use_python_detector("Use Python Detector", this, false),
  par_python_host("Python Host", this, "127.0.0.1"),
  par_python_port("Python Port", this, 5555),
  par_enabled("Enabled", this, true)
{
}

  mPercept::~mPercept()
{
  if (python_socket_fd_ >= 0)
  {
    ::close(python_socket_fd_);
    python_socket_fd_ = -1;
  }
}

void mPercept::Update()
{
  // Paused percept: do nothing at all (no detection, no camera processing).
  if (!par_enabled.Get())
  {
    return;
  }
  if (!this->InputChanged())
  {
    return;
  }
  const double pipeline_start_ms = NowSteadyMs();

  auto images = this->in_images.GetPointer();
  if (images->size() == 0)
  {
    return;
  }

  const auto ts = images.GetTimestamp();
  cv::Mat cam_image = rrlib::coviroa::ConvertFormat<cv::Vec3b>(images->at(0), rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_BGR24);

  const bool use_python = par_use_python_detector.Get();

  // ── Load / reload the local Darknet models (skipped in Python mode) ───────
  // Model 1 = person/COCO (par_darknet_*). Model 2 = custom robot (par_robot_*).
  if (!use_python)
  {
    std::string cfg;
    par_darknet_config.Get(cfg);
    std::string weights;
    par_darknet_weights.Get(weights);
    if (cfg != loaded_darknet_config_ || weights != loaded_darknet_weights_)
    {
      darknet_yolo_.reset();
      loaded_darknet_config_.clear();
      loaded_darknet_weights_.clear();
      if (!cfg.empty() && !weights.empty())
      {
        try
        {
          darknet_yolo_ = std::make_unique<object_detection::tDarknetYOLO>(cfg, weights);
          loaded_darknet_config_  = cfg;
          loaded_darknet_weights_ = weights;
          RRLIB_LOG_PRINT(USER, "Person YOLO model loaded: ", cfg);
        }
        catch (const std::exception& e)
        {
          RRLIB_LOG_PRINT(ERROR, "Failed to load person YOLO model: ", e.what());
        }
      }
    }

    std::string robot_cfg;
    par_robot_config.Get(robot_cfg);
    std::string robot_weights;
    par_robot_weights.Get(robot_weights);
    if (robot_cfg != loaded_robot_config_ || robot_weights != loaded_robot_weights_)
    {
      darknet_robot_.reset();
      loaded_robot_config_.clear();
      loaded_robot_weights_.clear();
      if (!robot_cfg.empty() && !robot_weights.empty())
      {
        try
        {
          darknet_robot_ = std::make_unique<object_detection::tDarknetYOLO>(robot_cfg, robot_weights);
          loaded_robot_config_  = robot_cfg;
          loaded_robot_weights_ = robot_weights;
          RRLIB_LOG_PRINT(USER, "Robot YOLO model loaded: ", robot_cfg);
        }
        catch (const std::exception& e)
        {
          RRLIB_LOG_PRINT(ERROR, "Failed to load robot YOLO model: ", e.what());
        }
      }
    }
  }

  // ── FPS throttle (applies to both the Python and Darknet paths) ──────────
  bool run_inference = true;
  const float max_fps = par_max_inference_fps.Get();
  if (max_fps > 0.0f)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto min_dt = std::chrono::duration<double>(1.0 / static_cast<double>(max_fps));
    if (has_last_inference_time_ && ((now - last_inference_time_) < min_dt))
    {
      run_inference = false;
    }
    else
    {
      last_inference_time_ = now;
      has_last_inference_time_ = true;
    }
  }

  // ── Acquire detections from the selected source ──────────────────────────
  std::vector<tLabeledDetection> result_detections;

  if (use_python)
  {
    // Send the frame to the Python YOLO server and use the boxes it returns.
    if (run_inference)
    {
      if (python_socket_fd_ < 0)
      {
        std::string host;
        par_python_host.Get(host);
        python_socket_fd_ = ConnectPythonServer(host, par_python_port.Get());
        if (python_socket_fd_ >= 0)
        {
          RRLIB_LOG_PRINT(USER, "Connected to Python YOLO server ", host, ":", par_python_port.Get());
        }
      }
      bool ok = false;
      if (python_socket_fd_ >= 0)
      {
        ok = DetectViaPython(python_socket_fd_, cam_image, result_detections);
        if (!ok)
        {
          ::close(python_socket_fd_);
          python_socket_fd_ = -1;
          RRLIB_LOG_PRINT(WARNING, "Python YOLO server unavailable; will retry");
        }
      }
      if (!ok)
      {
        PublishUnavailableVisualizations(*this, *images, ts, cam_image, "Python detector unavailable");
        {
          auto detections = out_detections.GetUnusedBuffer();
          detections->clear();
          detections.SetTimestamp(ts);
          out_detections.Publish(detections);
        }
        cached_detections_.clear();
        has_cached_detections_ = true;
        out_pipeline_start_ms.Publish(pipeline_start_ms, ts);
        out_perception_latency_ms.Publish(0.0f, ts);
        return;
      }
      cached_detections_     = result_detections;
      has_cached_detections_ = true;
    }
    else if (has_cached_detections_)
    {
      result_detections = cached_detections_;
    }
  }
  else
  {
    // Local Darknet path. Bail out with a message if no model is available.
    if (!darknet_yolo_ && !darknet_robot_)
    {
      PublishUnavailableVisualizations(*this, *images, ts, cam_image, "YOLO model not loaded");
      {
        auto detections = out_detections.GetUnusedBuffer();
        detections->clear();
        detections.SetTimestamp(ts);
        out_detections.Publish(detections);
      }
      cached_detections_.clear();
      has_cached_detections_ = true;
      out_pipeline_start_ms.Publish(pipeline_start_ms, ts);
      out_perception_latency_ms.Publish(0.0f, ts);
      return;
    }

    // Person model keeps its COCO class names; every robot-model box is forced
    // to the robot label (its class id 0 would otherwise decode as "person").
    if (run_inference)
    {
      const float threshold = par_darknet_threshold.Get();
      const bool print_log  = par_print_detection_log.Get();
      std::string robot_label;
      par_robot_class_name.Get(robot_label);
      robot_label = ToLowerAscii(robot_label);
      // Restrict the COCO model to the target class (person), like Python classes=[0].
      std::string person_filter;
      par_target_class_name.Get(person_filter);
      person_filter = ToLowerAscii(person_filter);

      if (darknet_yolo_)
      {
        RunDetector(*darknet_yolo_, cam_image, ts, threshold, print_log,
                    /*forced_label=*/"", /*keep_only=*/person_filter, result_detections);
      }
      if (darknet_robot_)
      {
        RunDetector(*darknet_robot_, cam_image, ts, threshold, print_log,
                    robot_label, /*keep_only=*/"", result_detections);
      }
      cached_detections_     = result_detections;
      has_cached_detections_ = true;
    }
    else if (has_cached_detections_)
    {
      result_detections = cached_detections_;
    }
  }

  const double perception_end_ms = NowSteadyMs();
  const float perception_latency = static_cast<float>(perception_end_ms - pipeline_start_ms);

  // ── Per-detection distances (depth only) ──────────────────────────────────
  // Median point-cloud depth at the bbox centre. No bbox-height fallback: when
  // the sensor returns no valid depth for a box, the distance stays 0 =
  // "unknown / not ranged". The risk stage's state machine decides its meaning.
  auto point_cloud = in_point_cloud.GetPointer();
  int cloud_w = 0;
  int cloud_h = 0;
  const bool have_depth = point_cloud->Dimension() > 0 &&
                          GuessCloudGrid(point_cloud->Dimension(), cam_image.cols, cam_image.rows, cloud_w, cloud_h);

  std::vector<float> detection_distances(result_detections.size(), 0.0f);
  for (size_t i = 0; i < result_detections.size(); ++i)
  {
    if (have_depth)
    {
      detection_distances[i] = MedianDepthInBox(*point_cloud, LabeledRect(result_detections[i]),
                                                cam_image.cols, cam_image.rows, cloud_w, cloud_h);
    }
    // Otherwise detection_distances[i] stays 0.0f = unknown depth.
  }

  // ── Compute outputs ───────────────────────────────────────────────────────
  std::string target_class;
  par_target_class_name.Get(target_class);
  target_class = ToLowerAscii(target_class);
  float nearest_dist   = std::numeric_limits<float>::infinity();
  int   nearest_cls_id = -1;
  std::string nearest_cls;
  float target_dist  = std::numeric_limits<float>::infinity();
  int   target_count = 0;
  std::set<std::string> visible;

  for (size_t i = 0; i < result_detections.size(); ++i)
  {
    const auto& det = result_detections[i];
    const std::string& cls_str = det.label;   // already lower-case
    visible.insert(cls_str);

    const float dist = detection_distances[i];

    // Only valid depths (> 0) set the nearest distance; unknown (0) is ignored
    // here so it can never masquerade as "nearest".
    if (dist > 0.0f && dist < nearest_dist)
    {
      nearest_dist   = dist;
      nearest_cls_id = det.class_id;
      nearest_cls    = cls_str;
    }
    if (cls_str == target_class)
    {
      ++target_count;
      if (dist > 0.0f && dist < target_dist)
      {
        target_dist = dist;
      }
    }
  }

  // If objects were detected but none could be ranged, still report presence
  // (class name) so the risk stage does not mistake "seen but no depth" for
  // "nothing there". Distance stays 0 = unknown.
  if (nearest_cls.empty() && !result_detections.empty())
  {
    nearest_cls    = result_detections.front().label;
    nearest_cls_id = result_detections.front().class_id;
  }

  // ── Publish scalar outputs ────────────────────────────────────────────────
  out_nearest_distance_m.Publish(std::isfinite(nearest_dist) ? nearest_dist : 0.0f, ts);
  out_nearest_class_id.Publish(nearest_cls_id, ts);
  out_nearest_class.Publish(nearest_cls, ts);
  out_target_class_distance_m.Publish(std::isfinite(target_dist) ? target_dist : 0.0f, ts);
  out_target_class_count.Publish(target_count, ts);
  out_total_detections.Publish(static_cast<int>(result_detections.size()), ts);
  out_visible_classes.Publish(JoinVisibleClasses(visible), ts);
  out_pipeline_start_ms.Publish(pipeline_start_ms, ts);
  out_perception_latency_ms.Publish(perception_latency, ts);

  // out_detections keeps its COCO-enum type, which cannot represent the robot
  // class, so it is published empty (no downstream consumer relies on it).
  {
    auto det_buf = out_detections.GetUnusedBuffer();
    det_buf->clear();
    det_buf.SetTimestamp(ts);
    out_detections.Publish(det_buf);
  }

  // ── Per-object list for downstream tracking (mRiskAssessment) ────────────
  // Index-aligned with detection_distances; free-form label so "robot" and
  // person-model classes both come through.
  {
    auto ranged_buf = out_ranged_detections.GetUnusedBuffer();
    ranged_buf->clear();
    ranged_buf->reserve(result_detections.size());
    auto dist_buf = out_ranged_distances_m.GetUnusedBuffer();
    dist_buf->clear();
    dist_buf->reserve(result_detections.size());
    for (size_t i = 0; i < result_detections.size(); ++i)
    {
      const auto& det = result_detections[i];
      ranged_buf->emplace_back(cv::Rect(det.x, det.y, det.width, det.height), det.label, det.probability, ts);
      dist_buf->push_back(detection_distances[i]);
    }
    ranged_buf.SetTimestamp(ts);
    dist_buf.SetTimestamp(ts);
    out_ranged_detections.Publish(ranged_buf);
    out_ranged_distances_m.Publish(dist_buf);
  }

  // ── Visualization (bounding boxes) ───────────────────────────────────────
  if (out_visualization.IsConnected())
  {
    auto vis_buf = out_visualization.GetUnusedBuffer();
    vis_buf->Resize(cam_image.cols, cam_image.rows, rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_BGR24, 0);
    cv::Mat vis = rrlib::coviroa::AccessImageAsMat(*vis_buf);
    cam_image.copyTo(vis);
    for (const auto& det : result_detections)
    {
      const int x0 = det.x;
      const int y0 = det.y;
      cv::rectangle(vis, cv::Point(x0, y0), cv::Point(det.x + det.width, det.y + det.height), cv::Scalar(0, 255, 0), 2);
      const std::string lbl = det.label + " " + std::to_string(static_cast<int>(det.probability * 100)) + "%";
      cv::putText(vis, lbl, cv::Point(x0, std::max(y0 - 4, 12)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    vis_buf.SetTimestamp(ts);
    out_visualization.Publish(vis_buf);
  }

  // ── Visualization (class + risk colour) ──────────────────────────────────
  if (out_visualization_classes.IsConnected())
  {
    auto cls_buf = out_visualization_classes.GetUnusedBuffer();
    cls_buf->Resize(cam_image.cols, cam_image.rows, rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_BGR24, 0);
    cv::Mat cls_vis = rrlib::coviroa::AccessImageAsMat(*cls_buf);
    cam_image.copyTo(cls_vis);
    for (size_t i = 0; i < result_detections.size(); ++i)
    {
      const auto& det = result_detections[i];
      const int x0 = det.x;
      const int y0 = det.y;
      const float d = detection_distances[i];
      const bool has_depth = (d > 0.0f && std::isfinite(d));
      // Grey = detected but not ranged (unknown depth); otherwise risk colour.
      const cv::Scalar color = has_depth ? RiskColorFromLevel(RiskLevelFromDistance(d))
                                         : cv::Scalar(200, 200, 200);
      cv::rectangle(cls_vis, cv::Point(x0, y0), cv::Point(det.x + det.width, det.y + det.height), color, 2);
      char label[80];
      if (has_depth)
      {
        std::snprintf(label, sizeof(label), "%s %.2fm", det.label.c_str(), d);
      }
      else
      {
        std::snprintf(label, sizeof(label), "%s ?", det.label.c_str());
      }
      cv::putText(cls_vis, label,
                  cv::Point(x0, std::max(y0 - 4, 12)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
    }
    cls_buf.SetTimestamp(ts);
    out_visualization_classes.Publish(cls_buf);
  }
}

}
}
}
