#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace whycon_plugin
{

/** Represent an L-shape detected by the WhyCon detector */
struct LShape
{
  /** True if the l-shape is visible */
  bool visible = false;
  /** Position of the l-shape in the camera frame */
  sva::PTransformd pos = sva::PTransformd::Identity();
  /** Position of the l-shape in the world frame (estimated) */
  sva::PTransformd posW = sva::PTransformd::Identity();

  /** Robot to which the shape is attached */
  std::string robot{};
  /** Frame on the robot to which the shape is attached */
  std::string frame{};
  /** Offset relative to the frame on the robot */
  sva::PTransformd frameOffset = sva::PTransformd::Identity();

  /** Tick every iteration to update the visibility */
  void tick(double dt);
  /** Called to update the position of the marker from the vision system */
  void update(const sva::PTransformd & in, const sva::PTransformd & X_0_camera);
  /** time since last update */
  inline double lastUpdate() const noexcept
  {
    return lastUpdate_;
  }

private:
  double lastUpdate_ = 1;
};

} // namespace whycon_plugin
