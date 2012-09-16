#ifndef ASUS_VS_VELO_VISUALIZER_H
#define ASUS_VS_VELO_VISUALIZER_H

#include <gperftools/profiler.h>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>
#include <Eigen/Cholesky>
#include <ros/package.h>
#include <matplotlib_interface/matplotlib_interface.h>
#include <rgbd_sequence/vis_wrapper.h>
#include <rgbd_sequence/stream_sequence.h>
#include <xpl_calibration/depth_distortion_learner.h>
#include <xpl_calibration/velo_to_asus_calibrator.h>

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

class AsusVsVeloVisualizer : public GridSearchViewHandler
{
public:
  int skip_;
  int num_pixel_plots_;
  VeloToAsusCalibration cal_;
  rgbd::PrimeSenseModel model_;
  
  AsusVsVeloVisualizer(rgbd::StreamSequence::Ptr sseq, VeloSequence::ConstPtr vseq);
  void run();
  void handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective);
  void saveAll(std::string tag = "") const;
  void saveExtrinsics(std::string tag = "") const;
  void saveIntrinsics(std::string tag = "") const;
  //! Find alignment and sync offset.
  void calibrate();
  void evaluate(std::string eval_path);
  void visualizeDistortion();
  void setColorScheme(std::string name);
  void toggleColorScheme();
  void fitModel();
  void fitFocalLength();
  
protected:
  rgbd::StreamSequence::Ptr sseq_;
  VeloSequence::ConstPtr vseq_;
  rgbd::VisWrapper vw_;
  int velo_idx_;
  int asus_idx_;
  double sseq_start_;
  rgbd::Cloud::Ptr velo_;
  rgbd::Cloud::Ptr asus_;
  rgbd::Cloud::Ptr vis_;
  bool unwarp_;
  std::string color_scheme_;
  double theta_lower_;
  double theta_upper_;

  void singleFrameExtrinsicsSearch();
  bool veloYawValid(double yaw) const;
  void updateVeloBounds();
  void setInitialExtrinsics();
  void incrementVeloIdx(int val);
  void incrementFocalLength(double df);
  void incrementOffset(double dt);
  int findAsusIdx(double ts, double* dt_out = NULL) const;
  void updateDisplay(int velo_idx, const Eigen::Affine3f& transform, double offset);
  rgbd::Cloud::Ptr filterVelo(rgbd::Cloud::ConstPtr velo) const;
  rgbd::Cloud::Ptr filterAsus(rgbd::Cloud::ConstPtr asus) const;
  LossFunction::Ptr getLossFunction() const;
  void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
  void play(bool save);
  void colorPoint(rgbd::Point* pt) const;
  VeloToAsusCalibrator setupCalibrator();
};

#endif // ASUS_VS_VELO_VISUALIZER_H
