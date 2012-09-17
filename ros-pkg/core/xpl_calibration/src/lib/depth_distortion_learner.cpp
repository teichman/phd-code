#include <xpl_calibration/depth_distortion_learner.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
namespace bfs = boost::filesystem;

#define DDL_INCR (getenv("DDL_INCR") ? atoi(getenv("DDL_INCR")) : 1)
#define REGULARIZATION (getenv("REGULARIZATION") ? atof(getenv("REGULARIZATION")) : 0.0)
//#define VISUALIZE

DepthDistortionLearner::DepthDistortionLearner(const PrimeSenseModel& initial_model) :
  initial_model_(initial_model),
  use_filters_(true),
  coverage_map_(0.05, initial_model.height_, initial_model.width_)
{
}

void DepthDistortionLearner::addFrame(Frame frame,
				      rgbd::Cloud::ConstPtr pcd,
				      const Eigen::Affine3d& transform)
{
  coverage_map_.addFrame(frame);
  frames_.push_back(frame);
  pcds_.push_back(pcd);
  transforms_.push_back(transform);
}

PrimeSenseModel DepthDistortionLearner::fitFocalLength()
{
  ROS_WARN("DepthDistortionLearner::fitFocalLength currently does not use depth distortion model correction on frames_.");
  FocalLengthMDE::Ptr objective(new FocalLengthMDE(initial_model_, frames_, pcds_, transforms_, 0.1));
  GridSearch gs(1);
  gs.verbose_ = true;
  gs.objective_ = objective;
  gs.num_scalings_ = 6;
  gs.max_resolutions_ << 10;
  gs.grid_radii_ << 5;
  gs.scale_factors_ << 0.5;

  ArrayXd init(1);
  init(0) = initial_model_.fx_;

  cout << "Initial f: " << init(0) << endl;
  cout << "Initial objective: " << objective->eval(init) << endl;
  ArrayXd x = gs.search(init);
  cout << "GridSearch solution: " << x.transpose() << endl;
  cout << "Final objective: " << objective->eval(x) << endl;

  PrimeSenseModel model = initial_model_;
  model.fx_ = x(0);
  model.fy_ = x(0);
  return model;
}

bool normalTest(const PrimeSenseModel& model, const DepthMat& mapdepth, int uc, int vc, int size, cv::Mat3b* visualization)
{
  ROS_ASSERT(mapdepth(vc, uc) != 0);

  Point pt_center;
  ProjectivePoint ppt;
  ppt.u_ = uc;
  ppt.v_ = vc;
  ppt.z_ = mapdepth(vc, uc);
  model.project(ppt, &pt_center);

  int min_u = max(0, uc - size);
  int max_u = min((int)mapdepth.cols(), uc + size);
  int min_v = max(0, vc - size);
  int max_v = min((int)mapdepth.rows(), vc + size);
  
  vector<Vector3f> vecs;
  vecs.reserve((max_u - min_u) * (max_v - min_v));
  Matrix3f X = Matrix3f::Zero();
  for(ppt.u_ = min_u; ppt.u_ < max_u; ++ppt.u_) {
    for(ppt.v_ = min_v; ppt.v_ < max_v; ++ppt.v_) {
      if(mapdepth(ppt.v_, ppt.u_) == 0)
	continue;

      Point pt;
      ppt.z_ = mapdepth(ppt.v_, ppt.u_);
      model.project(ppt, &pt);
      Vector3f vec = pt.getVector3fMap() - pt_center.getVector3fMap();
      X += vec * vec.transpose();
      vecs.push_back(vec);
    }
  }

  if(vecs.size() < 5)
    return false;
  
  X /= (float)vecs.size();
  pcl::Normal normal;
  pcl::solvePlaneParameters(X, 
			    normal.normal[0],
			    normal.normal[1],
			    normal.normal[2],
			    normal.curvature);
    
  pcl::flipNormalTowardsViewpoint(pt_center, 0, 0, 0,
				  normal.normal[0],
				  normal.normal[1],
				  normal.normal[2]);

  // -- If we don't have a very stable surface normal in this region, then
  //    we don't want to use it.
  if(!isfinite(normal.normal[0]) || !isfinite(normal.normal[1]) || !isfinite(normal.normal[2]) ||
     isnan(normal.normal[0]) || isnan(normal.normal[1]) || isnan(normal.normal[2]))
  {
    (*visualization)(vc, uc) = cv::Vec3b(255, 0, 255);
    return false;
  }

  Vector3f eignormal;
  eignormal(0) = normal.normal[0];
  eignormal(1) = normal.normal[1];
  eignormal(2) = normal.normal[2];
  ROS_ASSERT(fabs(eignormal.norm() - 1.0) < 1e-6);
  int num_inliers = 0;
  float thresh = 0.04;
  for(size_t i = 0; i < vecs.size(); ++i)
    if(fabs(eignormal.dot(vecs[i])) < thresh)
      ++num_inliers;

  float frac_inliers = (double)num_inliers / vecs.size();
  //cout << "curvature: " << curvature << ", num_inliers: " << num_inliers << ", frac_inliers: " << frac_inliers << endl;
  if(frac_inliers < 0.8) {
    (*visualization)(vc, uc) = cv::Vec3b(255, 0, 255);
    return false;
  }

  // if(curvature > 0.001)
  //   return false;
  
  // -- If we are seeing this point from an oblique angle, then we don't
  //    want to use it.
  Vector3f zaxis;
  zaxis << 0, 0, -1;  // Normal is pointed towards us.
  double theta = acos(eignormal.dot(zaxis));
  if(theta > 45.0 * M_PI / 180.0) {
    (*visualization)(vc, uc) = cv::Vec3b(0, 255, 255);
    return false;
  }
    
  return true;
}

cv::Mat3b visualizeMultipliers(const MatrixXd& multipliers)
{
  double min_mult = 0.85;
  double max_mult = 1.15;
  
  cv::Mat3b vis(cv::Size(multipliers.cols(), multipliers.rows()), cv::Vec3b(0, 0, 0));
  for(int y = 0; y < multipliers.rows(); ++y) {
    for(int x = 0; x < multipliers.cols(); ++x) {
      double m = multipliers(y, x);
      if(m == 0) {
	vis(y, x) = cv::Vec3b(255, 255, 255);
	continue;
      }
      
      if(m < min_mult) {
	ROS_WARN_STREAM("Low multiplier of " << m);
	vis(y, x) = cv::Vec3b(0, 0, 255);
      }
      else if(m > max_mult) {
	ROS_WARN_STREAM("High multiplier of " << m);
	vis(y, x) = cv::Vec3b(255, 0, 0);
      }
      else {
	if(m < 1)
	  vis(y, x)[2] = (1.0 - (m - min_mult) / (1.0 - min_mult)) * 255;
	else
	  vis(y, x)[0] = (m - 1.0) / (max_mult - 1.0) * 255;
      }
    }
  }
  
  return vis;
}

void DepthDistortionLearner::computeMultiplierMap(const PrimeSenseModel& model,
						  const DepthMat& depth,
						  const DepthMat& mapdepth,
						  Eigen::MatrixXd* multipliers,
						  cv::Mat3b* visualization) const
{
  ROS_ASSERT(multipliers->rows() == depth.rows());
  ROS_ASSERT(multipliers->cols() == depth.cols());
  ROS_ASSERT(visualization->rows == depth.rows());
  ROS_ASSERT(visualization->cols == depth.cols());
  multipliers->setZero();
  *visualization = cv::Vec3b(0, 0, 0);

  cv::Mat1b mask(depth.rows(), mapdepth.cols());
  mask = 0;
  if(use_filters_) {
    for(int y = 0; y < mask.rows; ++y)
      for(int x = 0; x < mask.cols; ++x)
	if(mapdepth(y, x) != 0)
	  mask(y, x) = 255;
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 4);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 15);
  }
            
  double min_mult = 0.85;
  double max_mult = 1.15;

  ProjectivePoint ppt;
  Point pt;
  for(ppt.v_ = 0; ppt.v_ < depth.rows(); ++ppt.v_) {
    for(ppt.u_ = 0; ppt.u_ < depth.cols(); ++ppt.u_) {
      // Reject points with no data.
      if(mapdepth(ppt.v_, ppt.u_) == 0 || depth(ppt.v_, ppt.u_) == 0) {
	(*visualization)(ppt.v_, ppt.u_) = cv::Vec3b(255, 255, 255);
	continue;
      }

      // Reject points on the edge of the map.
      if(use_filters_ && mask(ppt.v_, ppt.u_) == 0) {
	(*visualization)(ppt.v_, ppt.u_) = cv::Vec3b(255, 255, 0);
	continue;
      }

      // Reject points that have unstable surface normals or which we are viewing from
      // an oblique angle.
      if(use_filters_ && !normalTest(model, mapdepth, ppt.u_, ppt.v_, 10, visualization))
	continue;
      
      ppt.z_ = mapdepth(ppt.v_, ppt.u_);
      model.project(ppt, &pt);
      double mapdist = pt.getVector3fMap().norm();
      ppt.z_ = depth(ppt.v_, ppt.u_);
      model.project(ppt, &pt);
      double measdist = pt.getVector3fMap().norm();
		
      // If the range is completely off, assume it's due to misalignment and not distortion.
      // This is the only filter that should be on for the Velodyne data.
      double mult = mapdist / measdist;
      if(mult > max_mult || mult < min_mult) {
	//ROS_WARN_STREAM("Multiplier out of acceptable range: " << mult);
	(*visualization)(ppt.v_, ppt.u_) = cv::Vec3b(127, 127, 127);
	continue;
      }

      multipliers->coeffRef(ppt.v_, ppt.u_) = mult;

      // Color the multiplier.
      if(mult < min_mult) {
	ROS_WARN_STREAM("Low multiplier of " << mult);
	(*visualization)(ppt.v_, ppt.u_) = cv::Vec3b(0, 0, 255);
      }
      else if(mult > max_mult) {
	ROS_WARN_STREAM("High multiplier of " << mult);
	(*visualization)(ppt.v_, ppt.u_) = cv::Vec3b(255, 0, 0);
      }
      else {
	if(mult < 1)
	  (*visualization)(ppt.v_, ppt.u_)[2] = (1.0 - (mult - min_mult) / (1.0 - min_mult)) * 255;
	else
	  (*visualization)(ppt.v_, ppt.u_)[0] = (mult - 1.0) / (max_mult - 1.0) * 255;
      }
    }
  }
 }

PrimeSenseModel DepthDistortionLearner::fitModel()
{
  ROS_ASSERT(frames_.size() == pcds_.size());
  ROS_ASSERT(frames_.size() == transforms_.size());

  CoverageMap2 cmap2(frames_[0].depth_->rows(), frames_[0].depth_->cols(), 0, 12, 12);
  int num_features = initial_model_.numFeatures();
  vector<MatrixXd> xxts(frames_.size(), MatrixXd::Zero(num_features, num_features));
  vector<VectorXd> bs(frames_.size(), VectorXd::Zero(num_features));
  VectorXi num_tr_ex_frame = VectorXi::Zero(frames_.size());
#ifndef VISUALIZE
#pragma omp parallel for
#endif 
  for(size_t i = 0; i < frames_.size(); i += DDL_INCR) {
    MatrixXd& xxt = xxts[i];
    VectorXd& b = bs[i];

    ScopedTimer st("total for frame");
    PrimeSenseModel localmodel = initial_model_;
    HighResTimer hrt;
    
    cout << "Accumulating training set for depth distortion model fit, frame " << i << " / " << frames_.size() << endl;
    Frame mapframe;
    hrt.reset("transforming"); hrt.start();
    rgbd::Cloud transformed;
    pcl::transformPointCloud(*pcds_[i], transformed, transforms_[i].cast<float>());
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;

    hrt.reset("projecting"); hrt.start();
    localmodel.cloudToFrame(transformed, &mapframe);
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;

    hrt.reset("Computing multiplier map"); hrt.start();
    const DepthMat& depth = *frames_[i].depth_;
    const DepthMat& mapdepth = *mapframe.depth_;
    MatrixXd multipliers(depth.rows(), depth.cols());
    cv::Mat3b visualization(depth.rows(), depth.cols());
    computeMultiplierMap(localmodel, depth, mapdepth, &multipliers, &visualization);
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
    
    #ifdef VISUALIZE
    cv::imshow("multipliers and filters", visualization);
    cv::imshow("multipliers", visualizeMultipliers(multipliers));
    cv::imshow("depth", frames_[i].depthImage());
    cv::imshow("mapdepth", mapframe.depthImage());
    cv::waitKey(10);
    if(DDL_INCR != 1)
      cv::waitKey();
    #endif

    hrt.reset("Accumulating xxt"); hrt.start();
    ProjectivePoint ppt;
    VectorXd f(localmodel.numFeatures());
    for(ppt.v_ = 0; ppt.v_ < multipliers.rows(); ++ppt.v_) {
      for(ppt.u_ = 0; ppt.u_ < multipliers.cols(); ++ppt.u_) {
	if(multipliers(ppt.v_, ppt.u_) == 0)
	  continue;
	
	ppt.z_ = depth(ppt.v_, ppt.u_);
	localmodel.computeFeatures(ppt, &f);

	xxt += f * f.transpose();  // This is the slow part, but apparently eigen does a good job.
	// for(int j = 0; j < xxt.cols(); ++j)
	//   for(int k = 0; k < xxt.rows(); ++k)
	//     xxt.coeffRef(k, j) += f.coeffRef(j) * f.coeffRef(k);
		
	b += f * multipliers(ppt.v_, ppt.u_);
	++num_tr_ex_frame(i);
	cmap2.increment(ppt.v_, ppt.u_, (double)ppt.z_ * 0.001);
      }
    }
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  }	

  string dir = ".ddl";
  if(!bfs::exists(dir))
    bfs::create_directory(dir);
  else
    ROS_ASSERT(bfs::is_directory(dir));
  cmap2.saveVisualizations(dir);
  cout << "Saved coverage visualization to " << dir << endl;
  
  MatrixXd xxt = MatrixXd::Zero(num_features, num_features);
  VectorXd b = VectorXd::Zero(num_features);
  int num_tr_ex = num_tr_ex_frame.sum();
  for(size_t i = 0; i < frames_.size(); ++i) {
    xxt += xxts[i];
    b += bs[i];
  }

  cout << "Running polynomial regression with " << num_tr_ex << " training examples." << endl;
  xxt /= (double)num_tr_ex;
  b /= (double)num_tr_ex;
  cout << "xxt: " << endl << xxt << endl;
  cout << "b: " << b.transpose() << endl;

  // -- Fit the model.
  PrimeSenseModel model = initial_model_;
  xxt += MatrixXd::Identity(xxt.rows(), xxt.cols()) * REGULARIZATION;
  model.weights_ = xxt.ldlt().solve(b);
  return model;
}

Eigen::VectorXd DepthDistortionLearner::regress(const Eigen::MatrixXd& X, const Eigen::VectorXd& Y) const
{
  MatrixXd xxt = X * X.transpose();
  VectorXd b = X*Y;
  return xxt.ldlt().solve(b);
}

Eigen::VectorXd DepthDistortionLearner::regressRegularized(const Eigen::MatrixXd& X, const Eigen::VectorXd& Y) const
{
  double gamma = 0.1;
  MatrixXd xxtr = X * X.transpose() + MatrixXd::Identity(X.rows(), X.rows()) * gamma;
  VectorXd b = X*Y;
  return xxtr.ldlt().solve(b);
}

void PixelStats::addPoint(double velo, double asus)
{
  velo_.push_back(velo);
  asus_.push_back(asus);
}

void PixelStats::stats(double* mean, double* stdev, double* num) const
{
  *num = velo_.size();
  
  if(!valid()) {
    *mean = 1;
    *stdev = 0;
    return;
  }
  
  *mean = 0;
  for(size_t i = 0; i < velo_.size(); ++i)
    *mean += velo_[i] / asus_[i];
  *mean /= (double)velo_.size();
  
  *stdev = 0;
  for(size_t i = 0; i < velo_.size(); ++i)
    *stdev += pow(velo_[i] / asus_[i] - *mean, 2);
  *stdev /= (double)velo_.size();
  *stdev = sqrt(*stdev);
}

bool PixelStats::valid() const
{
  return velo_.size() > 5;
}

size_t DepthDistortionLearner::size() const
{
  ROS_ASSERT(frames_.size() == pcds_.size());
  return frames_.size();
}



CoverageMap2::CoverageMap2(int rows, int cols, double min_dist, double max_dist, int num_bins) :
  min_counts_(20),
  min_dist_(min_dist),
  max_dist_(max_dist),
  counts_(num_bins, MatrixXi::Zero(rows, cols)),
  width_((max_dist_ - min_dist_) / num_bins)
{
}

void CoverageMap2::increment(int v, int u, double dist)
{
  int idx = floor((dist - min_dist_) / width_);
  if(idx < 0 || idx >= (int)counts_.size())
    return;

  lock();
  ++counts_[idx](v, u);
  unlock();
}

void CoverageMap2::saveVisualizations(const std::string& dir) const
{
  ROS_ASSERT(bfs::is_directory(dir));

  for(size_t i = 0; i < counts_.size(); ++i) {
    cv::Mat3b vis = visualizeSlice(counts_[i]);
    ostringstream oss;
    oss << dir << "/coverage_" << setw(2) << setfill('0') << i << ".png";
    cv::imwrite(oss.str(), vis);
  }
}

cv::Mat3b CoverageMap2::visualizeSlice(const Eigen::MatrixXi& counts) const
{
  cv::Mat3b vis(cv::Size(counts.cols(), counts.rows()));
  for(int y = 0; y < vis.rows; ++y) {
    for(int x = 0; x < vis.cols; ++x) {
      double val = min(1.0, ((double)counts(y, x) / min_counts_));
      uchar g = val * 255;
      uchar r = (1.0 - val) * 255;
      vis(y, x) = cv::Vec3b(0, g, r);
    }
  }
  return vis;
}
 


CoverageMap::CoverageMap(double scale, int rows, int cols) :
  min_dist_(0.8),
  max_dist_(10),
  scale_(scale),
  rows_(rows),
  cols_(cols)
{
  bins_.resize(rows * scale);
  for(size_t y = 0; y < bins_.size(); ++y) {
    bins_[y].resize(cols * scale);
    for(size_t x = 0; x < bins_[y].size(); ++x) {
      bins_[y][x].reserve(1e5);
    }
  }
}

void CoverageMap::addFrame(rgbd::Frame frame)
{
  const DepthMat& depth = *frame.depth_;
  for(int y = 0; y < depth.rows(); ++y) {
    for(int x = 0; x < depth.cols(); ++x) {
      int v = ((double)y / depth.rows()) * rows();
      int u = ((double)x / depth.cols()) * cols();
      if(u < 0)
	u = 0;
      if(u >= (int)cols())
	u = cols();
      if(v < 0)
	v = 0;
      if(v >= (int)rows())
	v = rows();
      bins_[v][u].push_back(depth(y, x) * 0.001);
    }
  }
}

cv::Mat3b CoverageMap::computeImage() const
{
  // cv::Mat3b vis(cv::Size(cols_, rows_));
  // for(int y = 0; y < rows_; ++y) {
  //   for(int x = 0; x < cols_; ++x) {
  //     int u = x * scale_;
  //     int v = y * scale_;
  //     vis(y, x) = colorizeBin(bins_[v][u]);
  //   }
  // }
  // return vis;
  
  cv::Mat3b small(rows(), cols());
  for(size_t y = 0; y < rows(); ++y) {
    for(size_t x = 0; x < cols(); ++x) {
      small(y, x) = colorizeBin(bins_[y][x]);
    }
  }

  cv::Mat3b img;
  cv::resize(small, img, cv::Size(cols_, rows_), cv::INTER_NEAREST);
  return img;
}

cv::Vec3b CoverageMap::colorizeBin(const vector<double>& bin) const
{
  int num_bins = 10;

  VectorXd counts = VectorXd::Zero(num_bins);
  double range = max_dist_ - min_dist_;
  double width = range / num_bins;
  for(size_t i = 0; i < bin.size(); ++i) {
    int idx = floor((bin[i] - min_dist_) / width);
    if(idx < 0)
      idx = 0;
    if(idx >= num_bins)
      idx = num_bins;
    ++counts(idx);
  }

  int filled_criterion = 20;
  int num_filled = 0;
  for(int i = 0; i < counts.rows(); ++i)
    if(counts(i) > filled_criterion)
      ++num_filled;

  double frac_filled = (double)num_filled / num_bins;
  return cv::Vec3b(0, 255 * frac_filled, 255 * (1.0 - frac_filled));
}

void CoverageMap::clear()
{
  for(size_t y = 0; y < rows(); ++y) 
    for(size_t x = 0; x < cols(); ++x)
      bins_[y][x].clear();
}




// DescriptorAccumulator::DescriptorAccumulator(int rows, int cols, double scale, int bins_per_cell,
// 					     double min_dist, double max_dist, uint64_t max_bytes) :
//   rows_(rows),
//   cols_(cols),
//   scale_(0.05),
//   min_dist_(0.8),
//   max_dist_(10)
// {
//   PrimeSenseModel model;
//   double bytes_per_descriptor = model.numFeatures() * sizeof(double);
//   double max_descriptors = max_bytes / bytes_per_descriptor;
//   double total_num_bins = (rows * scale) * (cols * scale) * bins_per_cell;
//   double max_descriptors_per_bin = max_descriptors / total_num_bins;
//   max_per_bin_ = max_descriptors_per_bin;
//   ROS_DEBUG_STREAM("DescriptorAccumulator using " << max_per_bin_ << " descriptors per bin.");

//   bins_.resize(rows * scale);
//   ys_.resize(rows * scale);
//   for(size_t y = 0; y < bins_.size(); ++y) {
//     bins_[y].resize(cols * scale);
//     ys_.resize(cols * scale);
//     for(size_t x = 0; x < bins_[y].size(); ++x) {
//       bins_[y][x].reserve(max_per_bin_);
//       ys_.reserve(max_per_bin_);
//     }
//   }
// }

// void DescriptorAccumulator::addFrame(rgbd::Frame measurement, rgbd::Frame map)
// {
//   if(model_.hasDepthDistortionModel()) {
//     ROS_FATAL("DescriptorAccumulator should not have a model_ with a depth distortion model already built in to it.");
//     ROS_FATAL_STREAM(model_.weights_);
//     abort();
//   }

//   double min_mult = 0.8;
//   double max_mult = 1.2;
  
//   const DepthMat& depth = *measurement.depth_;
//   const DepthMat& mapdepth = *map.depth_;
//   ProjectivePoint ppt;
//   Point pt;
//   for(ppt.v_ = 0; ppt.v_ < depth.rows(); ++ppt.v_) {
//     for(ppt.u_ = 0; ppt.u_ < depth.cols(); ++ppt.u_) {
//       if(mapdepth(ppt.v_, ppt.u_) == 0 || depth(ppt.v_, ppt.u_) == 0)
// 	continue;

//       ppt.z_ = mapdepth(ppt.v_, ppt.u_);
//       model_.project(ppt, &pt);
//       double mapdist = pt.getVector3fMap().norm();
//       ppt.z_ = depth(ppt.v_, ppt.u_);
//       model_.project(ppt, &pt);
//       double measdist = pt.getVector3fMap().norm();
      
//       // If the range is completely off, assume it's due to misalignment and not distortion.
//       double mult = mapdist / measdist;
//       if(mult > max_mult || mult < min_mult)
// 	continue;

//       VectorXd descriptor = model_.computeFeatures(ppt));
      
//       // -- Compute indices into the accumulator.
//       int v = ((double)ppt.v_ / depth.rows()) * rows();
//       int u = ((double)ppt.u_ / depth.cols()) * cols();
//       if(u < 0)
// 	u = 0;
//       if(u >= (int)cols())
// 	u = cols();
//       if(v < 0)
// 	v = 0;
//       if(v >= (int)rows())
// 	v = rows();

//       cells_[v][u].addDescriptor(measdist, descriptor, mult);
//   }
//   }
// }

// cv::Mat3b DescriptorAccumulator::computeImage() const
// {
//   cv::Mat3b small(rows(), cols());
//   for(size_t y = 0; y < rows(); ++y)
//     for(size_t x = 0; x < cols(); ++x)
//       small(y, x) = cells_[y][x].color();

//   cv::Mat3b img;
//   cv::resize(small, img, cv::Size(cols_, rows_), cv::INTER_NEAREST);
//   return img;
// }

// cv::Vec3b DescriptorAccumulator::colorizeBin(const std::vector<Eigen::VectorXd>& bin) const
// {
//   int num_bins = 10;

//   VectorXd counts = VectorXd::Zero(num_bins);
//   double range = max_dist_ - min_dist_;
//   double width = range / num_bins;
//   for(size_t i = 0; i < bin.size(); ++i) {
//     int idx = floor((bin[i] - min_dist_) / width);
//     if(idx < 0)
//       idx = 0;
//     if(idx >= num_bins)
//       idx = num_bins;
//     ++counts(idx);
//   }

//   int filled_criterion = 20;
//   int num_filled = 0;
//   for(int i = 0; i < counts.rows(); ++i)
//     if(counts(i) > filled_criterion)
//       ++num_filled;

//   double frac_filled = (double)num_filled / num_bins;
//   return cv::Vec3b(0, 255 * frac_filled, 255 * (1.0 - frac_filled));
// }

// void DescriptorAccumulator::clear()
// {
//   for(size_t y = 0; y < rows(); ++y) 
//     for(size_t x = 0; x < cols(); ++x)
//       bins_[y][x].clear();
// }


// DescriptorAccumulatorCell::DescriptorAccumulatorCell(double min_dist, double max_dist, int num_bins, size_t max_per_bin) :
//   min_dist_(min_dist),
//   max_dist_(max_dist),
//   num_bins_(num_bins),
//   max_per_bin_(max_per_bin)
// {
//   width_ = (max_dist_ - min_dist_) / (double)num_bins_;
// }

// void DescriptorAccumulatorCell::addDescriptor(double measured_distance, const VectorXd& descriptor, double multiplier)
// {
//   int idx = floor((measured_distance - min_dist_) / width_);
//   idx = min((int)bins_.size() - 1, max(0, idx));
//   vector<VectorXd>& bin = bins_[idx];
//   vector<double>& multipliers = multipliers_[idx];
  
//   // -- If this bin is full, replace one at random.
//   //    Otherwise append.    
//   if(bin.size() == max_per_bin_) {
//     size_t idx = rand() % bin.size();
//     bin[idx] = descriptor;
//     multipliers[idx] = multiplier;
//   }
//   else {
//     bin.push_back(descriptor);
//     multipliers.push_back(multiplier);
//   }
// }

// cv::Vec3b DescriptorAccumulatorCell::color() const
// {
  
// } 

