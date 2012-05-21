#ifndef BACKGROUND_SUBTRACTOR_H
#define BACKGROUND_SUBTRACTOR_H

#include <xpl_calibration/background_modeler.h>

class BackgroundSubtractor : public pipeline::Pod
{
public:
  DECLARE_POD(BackgroundSubtractor);
  BackgroundSubtractor(std::string name) :
    Pod(name)
  {
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    declareInput<const BackgroundModel*>("BackgroundModel");
        
    declareParam<int>("NumErosions", 5);
    declareParam<int>("NumDilations", 3);
    
    declareOutput<const std::vector< std::vector<int> >*>("ForegroundIndices");
    declareOutput<const std::vector<cv::Mat1b>*>("ForegroundImages"); // 255 is foreground, 0 is background.
  }

  void compute();
  void debug() const;

protected:
  std::vector< std::vector<int> > fg_indices_;
  std::vector<cv::Mat1b> fg_imgs_;

  void findForeground(const rgbd::Cloud& pcd,
		      const BackgroundModel& model,
		      std::vector<int>* indices,
		      cv::Mat1b img) const;
    
};

#endif // BACKGROUND_SUBTRACTOR_H
