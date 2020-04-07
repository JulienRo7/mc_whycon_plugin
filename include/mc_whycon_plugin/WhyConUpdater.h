#pragma once

#include "TaskUpdater.h"
#include "WhyConSubscriber.h"

namespace whycon_plugin
{

/** Update a PBVS task based on WhyCon data */
struct WhyConUpdater : public TaskUpdater
{
  /** Create a task updater
   *
   * \param subscriber WhyCon subscriber that will provide the data
   * \param surface Name of the surface marker
   * \param env Name of the environment marker
   * \param envOffset Offset from the environment marker to the target object
   * \param surfaceOffset Offset from the surface marker to the surface frame
   */
  WhyConUpdater(const WhyConSubscriber & subscriber,
                const std::string & surface,
                const std::string & env,
                const sva::PTransformd & envOffset = sva::PTransformd::Identity(),
                const sva::PTransformd & surfaceOffset = sva::PTransformd::Identity()
                );

  /** Update a PBVS task based on the information provided by the WhyCon subscriber */
  bool update(mc_tasks::MetaTask & task) override;

  /** Update look at task to look at the environment marker */
  bool updateLookAt(mc_tasks::LookAtTask & task) override;


  inline void envOffset(const sva::PTransformd & envOffset)
  {
    envOffset_ = envOffset;
  }

  inline const sva::PTransformd & envOffset() const
  {
    return envOffset_;
  }

  inline void surfaceOffset(const sva::PTransformd & surfaceOffset)
  {
    surfaceOffset_ = surfaceOffset;
  }

  inline const sva::PTransformd & surfaceOffset() const
  {
    return surfaceOffset_;
  }

private:
  const WhyConSubscriber & subscriber_;
  std::string surface_;
  std::string env_;
  sva::PTransformd envOffset_;
  sva::PTransformd surfaceOffset_;
};

} // namespace whycon_plugin
