#include <dst/evaluator.h>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)
#define SUPPRESS_VIS (getenv("SUPPRESS_VIS"))

using namespace std;

namespace dst
{

  Evaluator::Evaluator(const Eigen::VectorXd& weights,
		       bool frame_to_frame) :
    weights_(weights),
    frame_to_frame_(frame_to_frame)
  {
  }
  
  void Evaluator::evaluate(KinectSequence::ConstPtr seq)
  {
    ROS_ASSERT(!seq->segmentations_.empty());

    SegmentationPipeline sp(NUM_THREADS);
    sp.verbose_ = false;
    sp.setWeights(weights_);
    cv::Mat1b pred(seq->segmentations_[0].size(), 127);
    cv::Mat1b empty_seed(seq->seed_images_[0].size(), 127);
    for(size_t i = 0; i < seq->segmentations_.size(); ++i) {
      cout << "Evaluating frame " << i+1 << " / " << seq->segmentations_.size() << endl;      
      // -- Segment the frame.
      if(i == 0) {
	sp.run(seq->seed_images_[i],
	       seq->images_[i],
	       seq->pointclouds_[i],
	       cv::Mat3b(),
	       cv::Mat1b(),
	       KinectCloud::Ptr(),
	       pred,
	       KinectCloud::Ptr());
      }
      else {
	cv::Mat1b prev_seg = pred;
	if(frame_to_frame_)
	  prev_seg = seq->segmentations_[i-1];
	
	sp.run(empty_seed,
	       seq->images_[i],
	       seq->pointclouds_[i],
	       seq->images_[i-1],
	       prev_seg,
	       seq->pointclouds_[i-1],
	       pred,
	       KinectCloud::Ptr());
      }
      ++total_frames_segmented_;

      if(!SUPPRESS_VIS) { 
	cv::imshow("Original image", seq->images_[i]);
	cv::imshow("Prediction", pred);
	cvMoveWindow("Original image", 200, 100);
	cvMoveWindow("Prediction", 600, 100);
	cv::waitKey(10);
      }
      
      // -- Count up Hamming loss.
      for(int y = 0; y < pred.rows; ++y) {
	for(int x = 0; x < pred.cols; ++x) {
	  if(pred(y, x) != seq->segmentations_[i](y, x))
	    ++total_hamming_loss_;
	}
      }
    }

    ++total_sequences_segmented_;
  }

  std::string Evaluator::status() const
  {
    ostringstream oss;
    oss << "Evaluator status" << endl;
    oss << "Frame to frame: " << frame_to_frame_ << endl;
    oss << "Total number of sequences evaluated: " << total_sequences_segmented_ << endl;
    oss << "Total number of frames evaluated: " << total_frames_segmented_ << endl;
    oss << "Mean Hamming loss: " << total_hamming_loss_ / total_frames_segmented_ << endl;
    return oss.str();
  }

} // namespace dst

