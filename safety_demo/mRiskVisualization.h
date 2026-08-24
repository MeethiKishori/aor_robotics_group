//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
//----------------------------------------------------------------------
/*!\file    projects/scout/safety_demo/mRiskVisualization.h
 *
 * \brief   Overlays current risk state on a perception visualization image.
 */
//----------------------------------------------------------------------
#ifndef __projects__scout__safety_demo__mRiskVisualization_h__
#define __projects__scout__safety_demo__mRiskVisualization_h__

#include "plugins/structure/tModule.h"

#include "rrlib/coviroa/tImage.h"

namespace finroc
{
namespace scout
{
namespace safety_demo
{

class mRiskVisualization : public structure::tModule
{
public:
  tInput<rrlib::coviroa::tImage> in_image;
  tInput<int> in_risk_level;
  tInput<std::string> in_risk_label;

  tParameter<bool> par_enable_stream;

  tVisualizationOutput<rrlib::coviroa::tImage, tLevelOfDetail::ALL> out_image;

  mRiskVisualization(core::tFrameworkElement* parent, const std::string& name = "Risk Visualization");

protected:
  virtual ~mRiskVisualization();

private:
  virtual void Update() override;
};

}
}
}

#endif
