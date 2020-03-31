#include "LShape.h"

namespace whycon_plugin
{

void LShape::tick(double dt)
{
  lastUpdate_ += dt;
  visible = lastUpdate_ < 0.5;
}

void LShape::update(const sva::PTransformd & in, const sva::PTransformd & X_0_camera)
{
  visible = true;
  pos = in;
  posW = pos * X_0_camera;
  lastUpdate_ = 0;
}

} // namespace whycon_plugin
