#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/LookAtTask.h>
#include <mc_tasks/PositionBasedVisServoTask.h>
#include <mc_tasks/TransformTask.h>

#include <mc_whycon_plugin/WhyConSubscriber.h>

namespace whycon_plugin
{

/** Implement a peg in hole strategy to insert something in something.
 * Use visual servoing to keep the orientation fixed w.r.t. the surface contact between the two somethings
 * Use admittance task to keep a constant contact force that will insert the somathing once both somethings are aligned
 * Use transform task to steer the something in the surface plane until the hole in found.
 * Bonus LookAt task to keep things in sight
 *
 * The visual servoing is also used for completion criteria: once a certain threshold distance in the direction
 * controlled by the admittance has been reached then the something can considered has inserted in the other something
 *
 * Implementation based on the ApproachVisualServoing state
 */
struct TouchWithAdmittance : mc_control::fsm::State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  sva::PTransformd robotMarkerToFrameOffset(const mc_control::fsm::Controller & ctl) const;
  sva::PTransformd targetMarkerToFrameOffset(const mc_control::fsm::Controller & ctl) const;

  bool updatePBVSTask(mc_control::fsm::Controller & ctl);
  void updateLookAt(const mc_control::fsm::Controller & ctl);

private:
  /** Estimation of the bracket position obtained from the vision system */
  std::string robotFrame_;
  std::string targetFrame_;
  sva::PTransformd targetFrameOffset_ = sva::PTransformd::Identity();
  std::string robotMarkerName_;
  std::string targetMarkerName_;

  /* Offset relative to the target frame where the
   * visual servoing task is to drive the robot */
  sva::PTransformd targetOffset_ = sva::PTransformd::Identity();

  std::shared_ptr<WhyConSubscriber> subscriber_ = nullptr;

  /** Visual servoing task used to adjust the gripper x/y position */
  std::shared_ptr<mc_tasks::PositionBasedVisServoTask> pbvsTask_;
  /* std::shared_ptr<mc_solver::BoundedSpeedConstr> constr_; */

  /* stiffness of the visual servoing task
   * Note that in case of non-convergence this stiffness will gradually increase
   * until convergence up to maxStiffness_
   **/
  double stiffness_ = 20;
  double maxStiffness_ = 100;
  /** actual max speed */
  /* double maxSpeed_ = 0.01; */
  /* desired max speed */
  double maxSpeedDesired_ = 0.01;
  /** Whether visual servoing needs to be manually triggered */
  /* bool manualConfirmation_ = true; */
  /** Evaluation threshold for the task */
  double evalTh_ = 0.02;
  /** Speed threshold for the task */
  double speedTh_ = 0.02;

  /** Task used to keep the vision system active */
  std::shared_ptr<mc_tasks::LookAtTask> lookAt_;

  /** True when the marker was visible at the previous update of PBVS */
  bool wasVisible_ = true;
  bool visible_ = false;
  std::vector<std::string> category_;

  /** Admittance Task*/
  std::shared_ptr<mc_tasks::force::AdmittanceTask> admittanceTask_;
  sva::PTransformd initialPose_;
  double p_, v_, R_, p2pi_, p24pi2_;
  double r_, rp_, theta_, thetap_;

  /** State logic */
  bool done_ = false;
  void done();

public: // for easy logging
  sva::PTransformd endEffectorFramesRelativePose_ = sva::PTransformd::Identity();
  double endEffectorFramesDistance_ = 1.0;
};

} // namespace whycon_plugin
