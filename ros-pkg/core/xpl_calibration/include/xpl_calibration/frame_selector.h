#ifndef FRAME_SELECTOR_H
#define FRAME_SELECTOR_H

#include <xpl_calibration/common.h>

class FrameSelector : public pl::Pod
{
public:
  DECLARE_POD(FrameSelector);
  FrameSelector(std::string name) :
    Pod(name)
  {
    declareParam<int>("FrameId");
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    declareOutput<cv::Mat3b>("Image");
    declareOutput<rgbd::Cloud::ConstPtr>("Cloud");
  }

  void compute()
  {
    const rgbd::Sequence& seq = *pull<rgbd::Sequence::ConstPtr>("Sequence");
    ROS_ASSERT((int)seq.size() > param<int>("FrameId"));
    
    push<cv::Mat3b>("Image", seq.imgs_[param<int>("FrameId")]);
    push<rgbd::Cloud::ConstPtr>("Cloud", seq.pcds_[param<int>("FrameId")]);
  }
};

#endif // FRAME_SELECTOR_H
