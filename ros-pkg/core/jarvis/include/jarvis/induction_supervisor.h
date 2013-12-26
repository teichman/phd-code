#ifndef INDUCTION_SUPERVISOR_H
#define INDUCTION_SUPERVISOR_H

#include <online_learning/tbssl.h>
#include <jarvis/descriptor_pipeline.h>

class InductionSupervisor : public Agent
{
public:
  typedef boost::shared_ptr<InductionSupervisor> Ptr;
  typedef boost::shared_ptr<const InductionSupervisor> ConstPtr;

  InductionSupervisor(GridClassifier gc, YAML::Node config,
                      const Eigen::VectorXf& up, OnlineLearner* ol,
                      float conf_thresh, std::string output_dir);
  void _run();

protected:
  GridClassifier gc_;
  YAML::Node config_;
  Eigen::VectorXf up_;
  OnlineLearner* ol_;
  float conf_thresh_;
  std::string output_dir_;
};

#endif // INDUCTION_SUPERVISOR_H
