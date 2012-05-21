#ifndef ASUS_VS_VELO_VISUALIZER_H
#define ASUS_VS_VELO_VISUALIZER_H

#include <gperftools/profiler.h>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>
#include <Eigen/Cholesky>
#include <matplotlib_interface/matplotlib_interface.h>
#include <rgbd_sequence/vis_wrapper.h>
#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/projector.h>
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

class VeloToAsusCalibration : public Serializable
{
public:
  //! Added to velodyne timestamps.
  double offset_;
  Eigen::Affine3f velo_to_asus_;

  VeloToAsusCalibration();
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
};

class PixelStats
{
public:
  std::vector<double> velo_;
  std::vector<double> asus_;

  void addPoint(double velo, double asus);
  void stats(double* mean, double* stdev, double* num) const;
  bool valid() const;
  void reserve(int num) { velo_.reserve(num); asus_.reserve(num); }
};

class AsusVsVeloVisualizer : public GridSearchViewHandler
{
public:
  VeloToAsusCalibration cal_;
  Eigen::VectorXd weights_;
  
  AsusVsVeloVisualizer(rgbd::StreamSequence::ConstPtr sseq, VeloSequence::ConstPtr vseq);
  void run();
  void handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective);
  
protected:
  rgbd::StreamSequence::ConstPtr sseq_;
  VeloSequence::ConstPtr vseq_;
  rgbd::VisWrapper vw_;
  int velo_idx_;
  int asus_idx_;
  double sseq_start_;
  rgbd::Cloud::Ptr velo_;
  rgbd::Cloud::Ptr asus_;
  rgbd::Cloud::Ptr vis_;
  std::vector< std::vector<PixelStats> > statistics_;
  bool unwarp_;
  
  void incrementVeloIdx(int val);
  void incrementOffset(double dt);
  int findAsusIdx(double ts, double* dt_out = NULL) const;
  void align();
  //! Find alignment and sync offset.
  void calibrate();
  void updateDisplay(int velo_idx, const Eigen::Affine3f& transform, double offset);
  rgbd::Cloud::Ptr filterVelo(rgbd::Cloud::ConstPtr velo) const;
  rgbd::Cloud::Ptr filterAsus(rgbd::Cloud::ConstPtr asus) const;
  LossFunction::Ptr getLossFunction() const;
  void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
  Eigen::Affine3f gridSearchTransform(ScalarFunction::Ptr lf);
  double gridSearchSync(ScalarFunction::Ptr lf);
  void play(bool save);
  void colorPoint(rgbd::Point* pt) const;
  void generateHeatMap();
  void accumulateStatistics();
  void visualizeDistortion();
  void fitModel();
};

#endif // ASUS_VS_VELO_VISUALIZER_H
