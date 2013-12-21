#ifndef INDUCTION_SUPERVISOR_H
#define INDUCTION_SUPERVISOR_H

#include <online_learning/tbssl.h>

class InductionSupervisor : public Agent
{
public:
  typedef boost::shared_ptr<InductionSupervisor> Ptr;
  typedef boost::shared_ptr<const InductionSupervisor> ConstPtr;

  InductionSupervisor(GridClassifier gc, OnlineLearner* ol,
                      float conf_thresh, std::string output_dir);
  void _run();

protected:
  GridClassifier gc_;
  OnlineLearner* ol_;
  float conf_thresh_;
  std::string output_dir_;
};

#endif // INDUCTION_SUPERVISOR_H
