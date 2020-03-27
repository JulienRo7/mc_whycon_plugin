#pragma once

#include <mc_tasks/LookAtTask.h>

namespace whycon_plugin
{

/** Abstract interface to update a task from the vision */
struct TaskUpdater
{
  /** Update the task, the exact type of the task is known to the updater
   *
   * \returns True if the task was updated, false otherwise (e.g. tracking lost)
   */
  virtual bool update(mc_tasks::MetaTask & task) = 0;

  /** Update a look at task */
  virtual bool updateLookAt(mc_tasks::LookAtTask & task) = 0;
};

} /* whycon_plugin */
