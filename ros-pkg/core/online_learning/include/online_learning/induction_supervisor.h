#ifndef INDUCTION_SUPERVISOR_H
#define INDUCTION_SUPERVISOR_H

#include <online_learning/tbssl.h>

class InductionSupervisor : public Agent
{
public:
  typedef boost::shared_ptr<InductionSupervisor> Ptr;
  typedef boost::shared_ptr<const InductionSupervisor> ConstPtr;

  InductionSupervisor(OnlineLearner* ol, float conf_thresh, std::string output_dir);
  void train(TrackDataset::Ptr td,
             const std::vector<size_t>& nc,
             double obj_thresh);
  void _run();

protected:
  GridClassifier::Ptr gc_;
  OnlineLearner* ol_;
  float conf_thresh_;
  std::string output_dir_;
};

#endif // INDUCTION_SUPERVISOR_H
