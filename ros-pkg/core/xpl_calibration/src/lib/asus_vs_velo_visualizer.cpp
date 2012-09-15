#include <xpl_calibration/asus_vs_velo_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
namespace bfs = boost::filesystem;

VeloSequence::VeloSequence(std::string root_path) :
  root_path_(root_path)
{
  bfs::recursive_directory_iterator it(root_path_), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    ROS_ASSERT(is_regular_file(p));
    if(bfs::extension(p).compare(".pcd") == 0)
      pcd_names_.push_back(p.leaf());
    else if(bfs::extension(p).compare(".clk") == 0)
      clk_names_.push_back(p.leaf());
  }
  sort(pcd_names_.begin(), pcd_names_.end());
  sort(clk_names_.begin(), clk_names_.end());

  // -- Load timestamps.
  timestamps_.resize(clk_names_.size());
  for(size_t i = 0; i < clk_names_.size(); ++i) {
    ifstream fs((root_path_ + "/" + clk_names_[i]).c_str());
    ROS_ASSERT(fs.is_open());
    fs >> timestamps_[i];
    fs.close();
  }
}

Cloud::Ptr VeloSequence::getCloud(size_t idx) const
{
  ROS_ASSERT(idx < size());
  
  Cloud::Ptr pcd(new Cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(root_path_ + "/" + pcd_names_[idx], *pcd);
  pcd->header.stamp.fromSec(timestamps_[idx]);
  return pcd;
}

AsusVsVeloVisualizer::AsusVsVeloVisualizer(rgbd::StreamSequence::Ptr sseq, VeloSequence::ConstPtr vseq) :
  skip_(5),
  num_pixel_plots_(20),
  sseq_(sseq),
  vseq_(vseq),
  vw_("Visualizer", 0.3),
  velo_idx_(0),
  asus_idx_(0),
  sseq_start_(0),
  velo_(new Cloud),
  asus_(new Cloud),
  vis_(new Cloud),
  unwarp_(false)
{
  // Some older sequences have the old type of model which predicted z rather than distance multiplier.
  // We'll just assume that all stream sequences have a default distortion model and that custom
  // models are stored separetely.
  sseq_->model_.resetDepthDistortionModel();
  //ROS_ASSERT(!sseq_->model_.hasDepthDistortionModel());
  cout << "StreamSequence PrimeSenseModel: " << endl;
  cout << sseq_->model_.status("  ");

  sseq_start_ = sseq_->timestamps_[0];
  incrementVeloIdx(2);
  vw_.vis_.registerPointPickingCallback(&AsusVsVeloVisualizer::pointPickingCallback, *this);
  setColorScheme("monitor");
  mpliBegin();
  setInitialExtrinsics();

  model_ = sseq->model_;
  model_.resetDepthDistortionModel();
  updateVeloBounds();
}

void AsusVsVeloVisualizer::setInitialExtrinsics()
{
  // -- Asus looking forward.
  cal_.setVeloToAsus(generateTransform(- M_PI / 2.0, 0, -M_PI / 2.0, 0, 0, 0).inverse());

  // -- Building on the above, this is for the asus looking to the right.
  cal_.setVeloToAsus(generateTransform(M_PI, 0, 0, 0, 0, 0) * cal_.veloToAsus());
  cal_.setVeloToAsus(generateTransform(0, -0.5 * M_PI, 0, 0, 0, 0) * cal_.veloToAsus());
}

bool AsusVsVeloVisualizer::veloYawValid(double yaw) const
{
  // TODO: This doesn't work for edge cases.
  return (yaw > theta_lower_ && yaw < theta_upper_);
}

void AsusVsVeloVisualizer::setColorScheme(std::string name)
{
  ROS_ASSERT(name == "publication" || name == "monitor");
  
  if(name == "publication")
    vw_.vis_.setBackgroundColor(255, 255, 255);
  else if(name == "monitor")
    vw_.vis_.setBackgroundColor(0, 0, 0);

  color_scheme_ = name;
}

void AsusVsVeloVisualizer::toggleColorScheme()
{
  if(color_scheme_ == "publication")
    setColorScheme("monitor");
  else if(color_scheme_ == "monitor")
    setColorScheme("publication");
}

void AsusVsVeloVisualizer::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
  if(event.getPointIndex() == -1)
    return;
    
  Point pt;
  event.getPoint(pt.x, pt.y, pt.z);
  cout << "Selected point: " << pt.x << ", " << pt.y << ", " << pt.z << endl;
  vw_.vis_.removeAllShapes();
  
  Point origin;
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  if(color_scheme_ == "publication")
    vw_.vis_.addArrow<Point, Point>(origin, pt, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, "line");
  else {
    ROS_ASSERT(color_scheme_ == "monitor");
    vw_.vis_.addArrow<Point, Point>(origin, pt, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, "line");
  }
}

void AsusVsVeloVisualizer::colorPoint(rgbd::Point* pt) const
{
  double increment = 3;
  double thresh0 = -3;
  double thresh1 = thresh0 + increment;
  double thresh2 = thresh1 + increment;
  double thresh3 = thresh2 + increment;
  
  double height = -(cal_.asusToVelo() * pt->getVector3fMap())(2);
  
  if(height < thresh0) {
    pt->b = 0;
    pt->g = 0;
    pt->r = 255;
  }
  else if(height >= thresh0 && height < thresh1) {
    int val = (height - thresh0) / (thresh1 - thresh0) * 255.;
    pt->b = val;
    pt->g = val;
    pt->r = 255 - val;
  }
  else if(height >= thresh1 && height < thresh2) {
    int val = (height - thresh1) / (thresh2 - thresh1) * 255.;
    pt->b = 255;
    pt->g = 255 - val;
    pt->r = 0;
  }
  else if(height >= thresh2 && height < thresh3) {
    int val = (height - thresh2) / (thresh3 - thresh2) * 255.;
    pt->b = 255 - val;
    pt->g = val;
    pt->r = 0;
  }
  else {
    pt->b = 0;
    pt->g = 255;
    pt->r = 0;
  }
}

void AsusVsVeloVisualizer::updateDisplay(int velo_idx, const Eigen::Affine3f& transform, double offset)
{
  // -- Get corresponding clouds.
  velo_ = filterVelo(vseq_->getCloud(velo_idx));
  double dt;
  asus_idx_ = findAsusIdx(velo_->header.stamp.toSec() + offset, &dt);
  bool draw_asus = (dt < 0.015);
  if(draw_asus) {
    Frame frame;
    sseq_->readFrame(asus_idx_, &frame);
    if(unwarp_) {
      model_.use_distortion_model_ = true;
      model_.frameToCloud(frame, asus_.get());
    }
    else
      sseq_->model_.frameToCloud(frame, asus_.get());
    
    for(size_t i = 0; i < asus_->size(); ++i)
      colorPoint(&asus_->at(i));
  }

  // -- Draw.
  vw_.vis_.removeAllShapes();
  vis_->clear();
  pcl::transformPointCloud(*velo_, *vis_, transform);
  if(draw_asus)
    *vis_ += *asus_;
    
  vw_.showCloud(vis_);
}

void AsusVsVeloVisualizer::fitModel()
{
  PrimeSenseModel initial_model = sseq_->model_;
  initial_model.resetDepthDistortionModel();
  DepthDistortionLearner ddl(initial_model);
  ddl.use_filters_ = false;
    
  for(size_t i = skip_; i < vseq_->size(); i += skip_) {
    double dt;
    int idx = findAsusIdx(vseq_->timestamps_[i] + cal_.offset_, &dt);
    if(dt > 0.01)
      continue;

    cout << "Adding frame " << i << endl;
    Frame frame;
    sseq_->readFrame(idx, &frame);
    Cloud::Ptr filtered = filterVelo(vseq_->getCloud(i));
    // Cloud::Ptr transformed(new Cloud);
    // pcl::transformPointCloud(*filtered, *transformed, cal_.veloToAsus());
    ddl.addFrame(frame, filtered, cal_.veloToAsus().cast<double>());
  }

  model_ = ddl.fitModel();
  cout << "Learned new depth distortion model." << endl;
}

void AsusVsVeloVisualizer::updateVeloBounds()
{
  Point pt;
  pt.x = 0;
  pt.y = 0;
  pt.z = 10;
  pt.getVector3fMap() = cal_.asusToVelo() * pt.getVector3fMap();

  //cout << pt.getVector3fMap().transpose() << "  ------ " << -atan2(pt.y, pt.x) << endl;
  
  double center = -atan2(pt.y, pt.x);
  double thresh = 45.0 * M_PI / 180.0;  // This needs to be set extra wide so that grid search is always seeing data.

  theta_lower_ = center - thresh;
  theta_upper_ = center + thresh;
}

void AsusVsVeloVisualizer::run()
{
  while(true) {
    updateVeloBounds();
    updateDisplay(velo_idx_, cal_.veloToAsus(), cal_.offset_);
    cout << "velo: " << velo_->header.stamp.toSec() << " " << vseq_->timestamps_[velo_idx_]
	 << ", asus: " << sseq_->timestamps_[asus_idx_] - sseq_start_
	 << ", velo + offset - asus: " << velo_->header.stamp.toSec() + cal_.offset_ - (sseq_->timestamps_[asus_idx_] - sseq_start_) << endl;
      
    char key = vw_.waitKey();
    switch(key) {
    case 27:
      return;
      break;
    case 'o':
      toggleColorScheme();
      break;
    case 'C':
      singleFrameExtrinsicsSearch();
      break;
    case 'm':
      unwarp_ = !unwarp_;
      if(model_.fx_ == 0 || !model_.hasDepthDistortionModel()) {
	cout << "Cannot unwarp without a learned depth distortion model." << endl;
	unwarp_ = false;
      }
      cout << "unwarp_: " << unwarp_ << endl;
      break;
    case 'p':
      play(false);
      break;
    case 'v':
      play(true);
      break;
    // case 'V':
    //   visualizeDistortion();
    //   break;
    case 'M':
      fitModel();
      break;
    case 'O':
      fitFocalLength();
      break;
    case 'c':
      calibrate();
      break;
    // case 'f':
    //   cout << "-- Frame stats --" << endl;
    //   cout << "Velo timestamp: " << setprecision(16) << velo_->header.stamp.toSec() << endl;
    //   cout << "Offset: " << cal_.offset_ << endl;
    //   cout << "Asus timestamp: " << sseq_->timestamps_[asus_idx_] - sseq_start_ << endl;
    //   cout << "velo + offset - asus: " << velo_->header.stamp.toSec() + cal_.offset_ - (sseq_->timestamps_[asus_idx_] - sseq_start_) << endl;
    //   cout << "Loss for this frame: TODO" << endl;
    //   break;
    case 'f':
      incrementFocalLength(10);
      break;
    case 'F':
      incrementFocalLength(-10);
      break;
    case 'S':
      saveAll("manual");
      break;
    case ',':
      incrementVeloIdx(-1);
      break;
    case '.':
      incrementVeloIdx(1);
      break;
    case '<':
      incrementVeloIdx(-10);
      break;
    case '>':
      incrementVeloIdx(10);
      break;
    case '@':
      incrementVeloIdx(-100);
      break;
    case '#':
      incrementVeloIdx(100);
      break;
    case '[':
      incrementOffset(-1.0 / 30.0);
      break;
    case ']':
      incrementOffset(1.0 / 30.0);
      break;
    case '{':
      incrementOffset(-10.0 / 30.0);
      break;
    case '}':
      incrementOffset(10.0 / 30.0);
      break;
    case 'x':
      cal_.setVeloToAsus(generateTransform(2.0 * M_PI / 180.0, 0, 0, 0, 0, 0) * cal_.veloToAsus());
      break;
    case 'X':
      cal_.setVeloToAsus(generateTransform(-2.0 * M_PI / 180.0, 0, 0, 0, 0, 0) * cal_.veloToAsus());
      break;
    case 'y':
      cal_.setVeloToAsus(generateTransform(0, 2.0 * M_PI / 180.0, 0, 0, 0, 0) * cal_.veloToAsus());
      break;
    case 'Y':
      cal_.setVeloToAsus(generateTransform(0, -2.0 * M_PI / 180.0, 0, 0, 0, 0) * cal_.veloToAsus());
      break;
    case 'z':
      cal_.setVeloToAsus(generateTransform(0, 0, 2.0 * M_PI / 180.0, 0, 0, 0) * cal_.veloToAsus());
      break;
    case 'Z':
      cal_.setVeloToAsus(generateTransform(0, 0, -2.0 * M_PI / 180.0, 0, 0, 0) * cal_.veloToAsus());
      break;
    case 'w':
      cal_.setVeloToAsus(generateTransform(0, 0, 0, 0.1, 0, 0) * cal_.veloToAsus());
      break;
    case 's':
      cal_.setVeloToAsus(generateTransform(0, 0, 0, -0.1, 0, 0) * cal_.veloToAsus());
      break;
    case 'a':
      cal_.setVeloToAsus(generateTransform(0, 0, 0, 0, 0.1, 0) * cal_.veloToAsus());
      break;
    case 'd':
      cal_.setVeloToAsus(generateTransform(0, 0, 0, 0, -0.1, 0) * cal_.veloToAsus());
      break;
    case 'E':
      cal_.setVeloToAsus(generateTransform(0, 0, 0, 0, 0, 0.1) * cal_.veloToAsus());
      break;
    case 'e':
      cal_.setVeloToAsus(generateTransform(0, 0, 0, 0, 0, -0.1) * cal_.veloToAsus());
      break;
    default:
      break;
    }
  }
}

void AsusVsVeloVisualizer::incrementVeloIdx(int val)
{
  velo_idx_ += val;
  if(velo_idx_ < 0)
    velo_idx_ = 0;
  if(velo_idx_ >= (int)vseq_->size())
    velo_idx_ = vseq_->size() - 1;
}

void AsusVsVeloVisualizer::incrementOffset(double dt)
{
  cal_.offset_ += dt;
  cout << "Using offset: " << cal_.offset_ << endl;
}

void AsusVsVeloVisualizer::incrementFocalLength(double df)
{
  scopeLockWrite;
  model_.fx_ += df;
  model_.fy_ += df;
  cout << "Focal length: " << model_.fx_ << " " << model_.fx_ << endl;
}

int AsusVsVeloVisualizer::findAsusIdx(double ts, double* dt_out) const
{
  int idx = -1;
  double min_dt = numeric_limits<double>::max();
  for(size_t i = 0; i < sseq_->timestamps_.size(); ++i) {
    double dt = fabs(ts - (sseq_->timestamps_[i] - sseq_start_));
    if(dt < min_dt) {
      min_dt = dt;
      idx = i;
    }
  }
  if(dt_out)
    *dt_out = min_dt;
  
  return idx;
}

rgbd::Cloud::Ptr AsusVsVeloVisualizer::filterAsus(rgbd::Cloud::ConstPtr asus) const
{
  Cloud::Ptr filtered(new Cloud(*asus));
  Point nopt;
  nopt.x = std::numeric_limits<float>::quiet_NaN();
  nopt.y = std::numeric_limits<float>::quiet_NaN();
  nopt.z = std::numeric_limits<float>::quiet_NaN();
  for(size_t i = 0; i < filtered->size(); ++i)
    if(isFinite(filtered->at(i)) && filtered->at(i).z > 8)
      filtered->at(i) = nopt;
  assert(filtered->isOrganized());
  return filtered;
}

rgbd::Cloud::Ptr AsusVsVeloVisualizer::filterVelo(rgbd::Cloud::ConstPtr velo) const
{
  rgbd::Cloud::Ptr filtered(new Cloud);
  filtered->height = 1;
  assert(!filtered->isOrganized());
  
  filtered->reserve(velo->size() / 10.0);
  for(size_t i = 0; i < velo->size(); ++i) {
    const Point& pt = velo->at(i);
    if(!isFinite(pt))
      continue;
    double range = pt.getVector3fMap().norm();
    if(range > 15 || range < 0.5)
      continue;
    
    double yaw = -atan2(pt.y, pt.x);
    if(!veloYawValid(yaw))
      continue;
    
    filtered->push_back(pt);
  }

  filtered->header.stamp.fromSec(velo->header.stamp.toSec());
  return filtered;
}

void AsusVsVeloVisualizer::calibrate(std::string eval_path)
{
  VeloToAsusCalibrator calibrator(model_, this);
  
  // -- Choose Velodyne keyframes.
  int num_keyframes = 3000;
  int buffer = 100;
  int spacing = 5;
  int idx = buffer;
  while(true) {
    updateDisplay(idx, cal_.veloToAsus(), cal_.offset_);

    rgbd::Cloud::Ptr pcd = filterVelo(vseq_->getCloud(idx));
    // Apply initial transform.  Grid search will return a transform to apply on top of the initial one.
    pcl::transformPointCloud(*pcd, *pcd, cal_.veloToAsus());  
    // Apply initial sync offset.  Grid search will return an update to add to cal_.offset_.
    pcd->header.stamp.fromSec(pcd->header.stamp.toSec() + cal_.offset_);  
    calibrator.pcds_.push_back(pcd);

    idx += spacing;
    if((int)calibrator.pcds_.size() >= num_keyframes)
      break;
    if(idx > ((int)vseq_->size() - buffer))
      break;
  }
  cout << "Loaded " << calibrator.pcds_.size() << " velodyne keyframes." << endl;
  updateDisplay(velo_idx_, cal_.veloToAsus(), cal_.offset_);
  
  // -- Load Asus frames in the vicinity of the Velodyne keyframes.
  int window = 30;
  for(size_t i = 0; i < calibrator.pcds_.size(); ++i) {
    int idx = findAsusIdx(calibrator.pcds_[i]->header.stamp.toSec());
    cout << "- Loading nearby asus frames for velo keyframe " << i << endl;
    cout << "  ";
    for(int j = max(0, idx - window); j <= min(idx + window, (int)sseq_->size()); ++j) {
      Frame frame;
      sseq_->readFrame(j, &frame);
      frame.timestamp_ -= sseq_start_;
      calibrator.frames_.push_back(frame);
      cout << j << " ";
    }
    cout << endl;
  }

  // -- Run grid search over extrinsics and apply updates.
  double final_mde;
  VeloToAsusCalibration cal = calibrator.search(&final_mde);
  cout << "Done calibrating." << endl;
  cout << "Final mde: " << final_mde << endl;
  cout << "Calibration incremental update: " << endl;
  cout << cal.status("  ");
  cal_.offset_ += cal.offset_;
  cal_.setVeloToAsus(cal.veloToAsus() * cal_.veloToAsus());
  cout << "Final extrinsic Velo to Asus calibration: " << endl;
  cout << cal_.status("  ");

  cal_.save("calibration-autosave");

  if(eval_path != "") { 
    ofstream fs(eval_path.c_str());
    fs << final_mde << endl;
    fs.close();
    cout << "Save final mde to " << eval_path << endl;
  }
}

void AsusVsVeloVisualizer::singleFrameExtrinsicsSearch()
{
  VeloToAsusCalibrator calibrator(model_, this);

  Cloud::Ptr pcd = filterVelo(vseq_->getCloud(velo_idx_));
  pcl::transformPointCloud(*pcd, *pcd, cal_.veloToAsus());
  pcd->header.stamp.fromSec(0);
  calibrator.pcds_.push_back(pcd);
  
  Frame frame;
  sseq_->readFrame(asus_idx_, &frame);
  frame.timestamp_ = 0;
  calibrator.frames_.push_back(frame);

  VeloToAsusCalibration cal = calibrator.search();
  cout << "Done calibrating." << endl;
  cout << "Calibration incremental update: " << endl;
  cout << cal.status("  ");
  cal_.offset_ += cal.offset_;
  cal_.setVeloToAsus(cal.veloToAsus() * cal_.veloToAsus());
  cout << "Final extrinsic Velo to Asus calibration: " << endl;
  cout << cal_.status("  ");
}

void AsusVsVeloVisualizer::handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective)
{
  cout << "Grid search improvement.  New objective: " << objective << endl;

  ROS_ASSERT(x.rows() == 7);
  double dt = x(0);
  Affine3f incremental_transform = generateTransform(x(1), x(2), x(3), x(4), x(5), x(6));
  
  updateDisplay(velo_idx_, incremental_transform * cal_.veloToAsus(), cal_.offset_ + dt);
  static int num = 0;
  ostringstream oss;
  oss << "gridsearch" << setw(5) << setfill('0') << num << ".png";
  vw_.vis_.saveScreenshot(oss.str());
  ++num;
}

void AsusVsVeloVisualizer::play(bool save)
{
  for(; velo_idx_ < (int)vseq_->size(); ++velo_idx_) {
    updateDisplay(velo_idx_, cal_.veloToAsus(), cal_.offset_);
    char key = vw_.waitKey(20);
    if(key != 0)
      break;
    
    if(save) { 
      ostringstream oss;
      oss << "video" << setw(5) << setfill('0') << velo_idx_ << ".png";
      vw_.vis_.saveScreenshot(oss.str());
    }
  }
}

void AsusVsVeloVisualizer::saveExtrinsics(std::string tag) const
{
  cal_.save("extrinsics" + tag);
  cout << "Saved calibration to \"" << "extrinsics" << tag << "\"" << endl;
}

void AsusVsVeloVisualizer::saveIntrinsics(std::string tag) const
{
  string filename = "intrinsics" + tag;
  model_.save(filename);
  cout << "Saved depth distortion model to \"" << filename << "\"" << endl;
  cout << model_.status("  ");
}

void AsusVsVeloVisualizer::saveAll(std::string tag) const
{
  saveExtrinsics(tag);
  saveIntrinsics(tag);
}

void AsusVsVeloVisualizer::fitFocalLength()
{
  DepthDistortionLearner ddl(model_);
  
  for(size_t i = skip_; i < vseq_->size(); i += skip_) {
    double dt;
    int idx = findAsusIdx(vseq_->timestamps_[i] + cal_.offset_, &dt);
    if(dt > 0.01)
      continue;

    cout << "Adding frame " << i << endl;
    Frame frame;
    sseq_->readFrame(idx, &frame);
    Cloud::Ptr filtered = filterVelo(vseq_->getCloud(i));
    // Cloud::Ptr transformed(new Cloud);
    // pcl::transformPointCloud(*filtered, *transformed, cal_.veloToAsus());
    ddl.addFrame(frame, filtered, cal_.veloToAsus().cast<double>());
  }

  model_ = ddl.fitFocalLength();
  cout << "Learned new focal length." << endl;
}

// void AsusVsVeloVisualizer::visualizeDistortion()
// {
//   if(ddl_.statistics_.empty()) {
//     cout << "You must accumulate statistics first." << endl;
//     return;
//   }
//   cout << "Visualizing distortion." << endl;
  
//   // -- Generate a heat map.
//   int width = asus_->width;
//   int height = asus_->height;
//   Eigen::MatrixXd mean(height, width);
//   Eigen::MatrixXd stdev(height, width);
//   Eigen::MatrixXd counts(height, width);
//   mean.setZero();
//   stdev.setZero();
//   counts.setZero();
//   for(int y = 0; y < mean.rows(); ++y) 
//     for(int x = 0; x < mean.cols(); ++x) 
//       ddl_.statistics_[y][x].stats(&mean.coeffRef(y, x), &stdev.coeffRef(y, x), &counts.coeffRef(y, x));
  
//   mpliExport(mean);
//   mpliExport(stdev);
//   mpliExport(counts);
//   mpliPrintSize();
//   mpliExecuteFile(ros::package::getPath("xpl_calibration") + "/plot_multipliers_image.py");

//   // -- Range vs u image.
//   {
//     double max_range = 13;
//     int num_range_bins = 500;
//     double bin_size = max_range / (double)num_range_bins;
    
//     Eigen::ArrayXXd mean(num_range_bins, width);
//     Eigen::ArrayXXd counts(num_range_bins, width);
//     mean.setZero();
//     counts.setZero();
//     for(size_t y = 0; y < ddl_.statistics_.size(); ++y)  {
//       ROS_ASSERT((int)ddl_.statistics_[y].size() == width);
//       for(size_t x = 0; x < ddl_.statistics_[y].size(); ++x) {
// 	for(size_t i = 0; i < ddl_.statistics_[y][x].asus_.size(); ++i) { 
// 	  double range = ddl_.statistics_[y][x].velo_[i];
// 	  int range_bin = (max_range - range) / bin_size;
// 	  if(range_bin >= num_range_bins || range_bin < 0)
// 	    continue;
	  
// 	  double mult = range / ddl_.statistics_[y][x].asus_[i];
// 	  counts(range_bin, x) += 1;
// 	  mean(range_bin, x) += mult;
// 	}
//       }
//     }
//     mean /= counts;
//     for(int y = 0; y < mean.rows(); ++y)
//       for(int x = 0; x < mean.cols(); ++x)
// 	if(isnan(mean(y, x)))
// 	  mean(y, x) = 1;
        
//     Eigen::ArrayXXd stdev(num_range_bins, width);
//     stdev.setZero();
//     for(size_t y = 0; y < ddl_.statistics_.size(); ++y)  { 
//       for(size_t x = 0; x < ddl_.statistics_[y].size(); ++x) {
// 	for(size_t i = 0; i < ddl_.statistics_[y][x].asus_.size(); ++i) { 
// 	  double range = ddl_.statistics_[y][x].velo_[i];
// 	  int range_bin = (max_range - range) / bin_size;
// 	  if(range_bin >= num_range_bins || range_bin < 0)
// 	    continue;
	  
// 	  double mult = range / ddl_.statistics_[y][x].asus_[i];
// 	  stdev(range_bin, x) += pow(mult - mean(range_bin, x), 2);
// 	}
//       }
//     }
//     stdev = (stdev / counts).sqrt();
//     for(int y = 0; y < stdev.rows(); ++y)
//       for(int x = 0; x < stdev.cols(); ++x)
// 	if(isnan(stdev(y, x)))
// 	  stdev(y, x) = 0;

//     mpliExport(max_range);
//     mpliExport(bin_size);
//     mpliNamedExport("mean", mean);
//     mpliNamedExport("stdev", stdev);
//     mpliNamedExport("counts", counts);
//     mpliPrintSize();
//     mpliExecuteFile(ros::package::getPath("xpl_calibration") + "/plot_u_range_multipliers.py");
//   }
  
//   // -- For some random pixels, generate a scatter plot of asus range vs velo range.
//   srand(0);  // All runs should produce plots for the same set of pixels.
//   for(int i = 0; i < num_pixel_plots_; ++i) {
//     int u = rand() % width;
//     int v = rand() % height;
//     const PixelStats& ps = ddl_.statistics_[v][u];
//     if(!ps.valid())
//       continue;

//     cout << "Plotting distortion for pixel " << u << " " << v << endl;
//     mpliNamedExport("velo", ps.velo_);
//     mpliNamedExport("asus", ps.asus_);
//     mpliNamedExport<int>("width", width);
//     mpliNamedExport<int>("height", height);
//     mpliExport(u);
//     mpliExport(v);
//     mpliExecuteFile(ros::package::getPath("xpl_calibration") + "/plot_beam_scatter.py");
//   }
// }

