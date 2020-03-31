#pragma once

#include <memory>

namespace whycon_plugin
{

/** Generic interface to subscribe to vision information that will be used in
 * the manipulation phase */
struct VisionSubscriber
{
  /** Update vision system */
  virtual void tick(double dt) = 0;
};

using VisionSubscriberPtr = std::shared_ptr<VisionSubscriber>;

} // namespace whycon_plugin
