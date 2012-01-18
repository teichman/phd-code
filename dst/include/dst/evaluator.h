#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <dst/segmentation_pipeline.h>

namespace dst
{
  class Evaluator
  {
  public:
    Evaluator(const Eigen::VectorXd& weights, bool frame_to_frame);
    void evaluate(KinectSequence::ConstPtr seq);
    std::string status() const;
    
  private:
    Eigen::VectorXd weights_;
    bool frame_to_frame_;
    double total_hamming_loss_;
    double total_frames_segmented_;
    double total_sequences_segmented_;
  };

}

#endif // EVALUATOR_H
