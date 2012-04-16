#ifndef ASUS_VS_VELO_VISUALIZER_H
#define ASUS_VS_VELO_VISUALIZER_H

#include <pcl/common/transforms.h>
#include <rgbd_sequence/vis_wrapper.h>
#include <rgbd_sequence/stream_sequence.h>
#include <xpl_calibration/object_matching_calibrator.h>

typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;

class VeloSequence
{
public:
  typedef boost::shared_ptr<VeloSequence> Ptr;
  typedef boost::shared_ptr<const VeloSequence> ConstPtr;

  std::vector<double> timestamps_;
  
  VeloSequence(std::string root_path);
  rgbd::Cloud::Ptr getCloud(size_t idx) const;
  size_t size() const { return pcd_names_.size(); }
  
protected:
  std::string root_path_;
  std::vector<std::string> pcd_names_;
  std::vector<std::string> clk_names_;
};

class AsusVsVeloVisualizer
{
public:
  AsusVsVeloVisualizer(rgbd::StreamSequence::ConstPtr sseq, VeloSequence::ConstPtr vseq);
  void run();

protected:
  rgbd::StreamSequence::ConstPtr sseq_;
  VeloSequence::ConstPtr vseq_;
  rgbd::VisWrapper vw_;
  int velo_idx_;
  int asus_idx_;
  double offset_;
  double sseq_start_;
  rgbd::Cloud::Ptr velo_;
  rgbd::Cloud::Ptr asus_;
  rgbd::Cloud::Ptr vis_;
  Eigen::Affine3f asus_to_velo_;

  void incrementVeloIdx(int val);
  void incrementOffset(double dt);
  int findAsusIdx(double ts, double* dt_out = NULL) const;
  void sync();
  void align();
  //! Find alignment and sync offset.
  void calibrate();
  void updateDisplay();
  rgbd::Cloud::Ptr filter(rgbd::Cloud::ConstPtr velo) const;
  LossFunction::Ptr getLossFunction() const;
  void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
  Eigen::Affine3f gridSearchTransform(ScalarFunction::Ptr lf) const;
  double gridSearchSync(ScalarFunction::Ptr lf) const;
};

#endif // ASUS_VS_VELO_VISUALIZER_H
