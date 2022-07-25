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
   * \param frame Name of the frame marker
   * \param env Name of the environment marker
   * \param envOffset Offset from the environment marker to the target object
   * \param frameOffset Offset from the frame marker to the frame frame
   */
  WhyConUpdater(const WhyConSubscriber & subscriber,
                const std::string & frame,
                const std::string & env,
                const sva::PTransformd & envOffset = sva::PTransformd::Identity(),
                const sva::PTransformd & frameOffset = sva::PTransformd::Identity()
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

  inline void frameOffset(const sva::PTransformd & frameOffset)
  {
    frameOffset_ = frameOffset;
  }

  inline const sva::PTransformd & frameOffset() const
  {
    return frameOffset_;
  }

private:
  const WhyConSubscriber & subscriber_;
  std::string frame_;
  std::string env_;
  sva::PTransformd envOffset_;
  sva::PTransformd frameOffset_;
};

} // namespace whycon_plugin
