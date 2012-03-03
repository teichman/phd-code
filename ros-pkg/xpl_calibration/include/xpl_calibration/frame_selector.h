#ifndef FRAME_SELECTOR_H
#define FRAME_SELECTOR_H

#include <pipeline/pod.h>
#include <rgbd_sequence/rgbd_sequence.h>

class FrameSelector : public pipeline::Pod
{
  DECLARE_POD(FrameSelector);
  FrameSelector(std::string name) :
    Pod(name)
  {
    declareParam<size_t>("FrameId");
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    declareOutput<cv::Mat3b>("Image");
    declareOutput<rgbd::Cloud::ConstPtr>("Cloud");
  }

  void compute()
  {
    const rgbd::Sequence& seq = *pull<rgbd::Sequence::ConstPtr>("Sequence");
    ROS_ASSERT(seq.size() > param<size_t>("FrameId"));
    
    push<rgbd::Cloud::ConstPtr>("Image", seq.imgs_[param<size_t>("FrameId")]);
    push<rgbd::Cloud::ConstPtr>("Cloud", seq.pcds_[param<size_t>("FrameId")]);
  }
};

#endif // FRAME_SELECTOR_H
