//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
//----------------------------------------------------------------------
/*!\file    projects/scout/safety_demo/mRiskVisualization.cpp
 *
 * \brief   Overlays current risk state on a perception visualization image.
 */
//----------------------------------------------------------------------
#include "projects/scout/safety_demo/mRiskVisualization.h"

#include "rrlib/coviroa/opencv_utils.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace finroc
{
namespace scout
{
namespace safety_demo
{

runtime_construction::tStandardCreateModuleAction<mRiskVisualization> cCREATE_ACTION_FOR_M_RISK_VISUALIZATION("RiskVisualization");

mRiskVisualization::mRiskVisualization(core::tFrameworkElement* parent, const std::string& name) :
  tModule(parent, name, false),
  in_image("Image", this),
  in_risk_level("Risk Level", this),
  in_risk_label("Risk Label", this),
  par_enable_stream("Enable Stream", this, true),
  out_image("Image", this)
{
}

mRiskVisualization::~mRiskVisualization()
{
}

void mRiskVisualization::Update()
{
  if (!par_enable_stream.Get())
  {
    return;
  }

  if (!(in_image.HasChanged() || in_risk_level.HasChanged() || in_risk_label.HasChanged()))
  {
    return;
  }

  auto image = in_image.GetPointer();
  if (image->GetWidth() == 0 || image->GetHeight() == 0)
  {
    return;
  }

  const auto ts = image.GetTimestamp();
  cv::Mat input = rrlib::coviroa::ConvertFormat<cv::Vec3b>(*image, rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_BGR24);

  auto output = out_image.GetUnusedBuffer();
  output->Resize(image->GetWidth(), image->GetHeight(), rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_BGR24, 0);
  cv::Mat vis = rrlib::coviroa::AccessImageAsMat(*output);
  input.copyTo(vis);

  int risk_level = 0;
  in_risk_level.Get(risk_level);
  std::string risk_label;
  in_risk_label.Get(risk_label);

  const std::string line = "Risk: " + std::to_string(risk_level) + " (" + risk_label + ")";

  cv::rectangle(vis, cv::Point(8, vis.rows - 52), cv::Point(std::min(vis.cols - 8, 420), vis.rows - 8), cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(vis, line, cv::Point(16, vis.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 255), 2);

  output.SetTimestamp(ts);
  out_image.Publish(output);
}

}
}
}
