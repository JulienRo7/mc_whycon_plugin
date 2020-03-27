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
   * \param offset Offset from the environment marker to the target object
   */
  WhyConUpdater(const WhyConSubscriber & subscriber,
                const std::string & surface, const std::string & env,
                const sva::PTransformd & offset = sva::PTransformd::Identity());

  /** Update a PBVS task based on the information provided by the WhyCon subscriber */
  bool update(mc_tasks::MetaTask & task) override;

  /** Update look at task to look at the environment marker */
  bool updateLookAt(mc_tasks::LookAtTask & task) override;
private:
  const WhyConSubscriber & subscriber_;
  std::string surface_;
  std::string env_;
  sva::PTransformd offset_;
};

} /* whycon_plugin */
