#include <xpl_calibration/object_matching_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

void ObjectMatchingCalibrator::compute()
{
  const Sequence& seq0 = *pull<Sequence::ConstPtr>("Sequence0");
  const Sequence& seq1 = *pull<Sequence::ConstPtr>("Sequence1");
  const ObjectClouds& objects0 = *pull<const ObjectClouds*>("Objects0");
  const ObjectClouds& objects1 = *pull<const ObjectClouds*>("Objects1");
  ROS_ASSERT(seq0.size() == seq1.size());
  ROS_ASSERT(seq0.size() == objects0.size());
  ROS_ASSERT(seq0.size() == objects1.size());

  // -- Find a good starting point using centroid RANSAC.

  // Compute centroids for all objects.
  computeCentroids(objects0, &centroids0_);
  computeCentroids(objects1, &centroids1_);
  
  // RANSAC.
  int num_iterations = param<int>("NumRansacIters");
  int num_correspondences = param<int>("NumCorrespondences");
  Affine3f best_transform = Affine3f::Identity();
  int best_num_inliers = 0;
  for(int i = 0; i < num_iterations; ++i) {
    cout << "RANSAC iter " << i << endl;
    pcl::TransformationFromCorrespondences tfc;
    for(int j = 0; j < num_correspondences; ++j)
      sampleCorrespondence(seq0, seq1, centroids0_, centroids1_, &tfc);
    
    // Compute transform.
    Affine3f transform = tfc.getTransformation();

    // Count up inliers.
    int num_inliers = countInliers(transform, centroids0_, centroids1_,
				   param<double>("CentroidThreshold"));
    
    
    // Save this transform if it's the best.
    if(num_inliers > best_num_inliers) {
      best_num_inliers = num_inliers;
      best_transform = transform;
      cout << "Best transform has " << num_inliers << " inliers." << endl;
    }
  }
  rough_transform_ = best_transform;

  cout << "Num inliers before refinement: " << best_num_inliers << endl;
  int new_num_inliers = countInliers(rough_transform_, centroids0_, centroids1_,
				     param<double>("CentroidThreshold"), &ransac_refined_transform_);
  cout << "Num inliers after refinement: " << new_num_inliers << endl;

  // -- Using the above as a starting point, now run ICP on all
  //    corresponding object clouds simultaneously.
  icp_refined_transform_ = alignInlierModels(centroids0_, centroids1_);				     
				     
  // -- Alternating grid search.
  
  // // Match floating objects to reference scenes.
  // vector<Cloud::Ptr> objs1;
  // for(size_t i = 0; i < objects1.size(); ++i)
  //   for(size_t j = 0; j < objects1[i].size(); ++j) { 
  //     objs1.push_back(Cloud::Ptr(new Cloud(*objects1[i][j])));
  //     objs1.back()->header = objects1[i][j]->header;
  //   }
  // vector<Cloud::Ptr> pcds1;
  // downsampleAndTransform(objs1, icp_refined_transform_, &pcds1);

  // // Match floating scenes to reference scenes.
  vector<Cloud::Ptr> pcds1;
  downsampleAndTransform(seq1->pcds_, icp_refined_transform_, &pcds1);

  
  double sync;
  Affine3f incremental;
  gridSearch(seq0.pcds_, pcds1, &incremental, &sync);
  gridsearch_transform_ = incremental * icp_refined_transform_;

  push<double>("SyncOffset", sync);
  push<const Affine3f*>("RansacRoughTransform", &rough_transform_);
  push<const Affine3f*>("RansacRefinedTransform", &ransac_refined_transform_);
  push<const Affine3f*>("IcpRefinedTransform", &icp_refined_transform_);
  push<const Affine3f*>("GridSearchTransform", &gridsearch_transform_);
}

void ObjectMatchingCalibrator::downsampleAndTransform(const std::vector<Cloud::Ptr>& source,
						      const Eigen::Affine3f& transform,
						      std::vector<Cloud::Ptr>* destination) const
{
  destination->clear();
  
  double downsample = param<double>("Downsampling");
  for(size_t i = 0; i < source.size(); ++i) {
    const Cloud& src = *source[i];
    Cloud::Ptr dst(new Cloud);
    dst->header = src.header;
    dst->reserve(src.size());
    for(size_t j = 0; j < src.size(); ++j)
      if(((double)rand() / (double)RAND_MAX) > downsample)
	dst->push_back(src[j]);

    pcl::transformPointCloud(*dst, *dst, transform);
    destination->push_back(dst);
  }
}

void ObjectMatchingCalibrator::gridSearch(const std::vector<rgbd::Cloud::Ptr>& pcds0,
					  const std::vector<rgbd::Cloud::Ptr>& pcds1,
					  Eigen::Affine3f* final_transform,
					  double* final_sync) const
{
  *final_transform = Affine3f::Identity();
  *final_sync = 0;

  LossFunction::Ptr lf(new LossFunction(param<double>("DistanceThreshold"),
					param<double>("TimeCorrespondenceThreshold"),
					pcds0, pcds1));
  
  int iter = 0;
  while(true) {    
    if(debug_) {
      ostringstream oss;
      oss << getDebugPath() << "-gsiter" << setw(4) << setfill('0') << iter << ".pcd";
      Cloud overlay = *pcds0[0];
      overlay += *pcds1[0];
      pcl::io::savePCDFileBinary(oss.str(), overlay);
    }
    ++iter;
    
    double best_dt = gridSearchSync(lf);
    *final_sync += best_dt;
    for(size_t i = 0; i < pcds1.size(); ++i) {
      double ts = pcds1[i]->header.stamp.toSec() + best_dt;
      pcds1[i]->header.stamp.fromSec(ts);
    }
    cout << "Best sync offset so far is " << *final_sync << endl;
    
    // Grid search to find the next step.
    // rx, ry, rz, tx, ty, tz
    Affine3f incremental_transform = gridSearchTransform(lf);
    *final_transform = incremental_transform * (*final_transform);
    cout << "Transforming clouds." << endl;
    for(size_t i = 0; i < pcds1.size(); ++i) {
      pcl::transformPointCloud(*pcds1[i], *pcds1[i], incremental_transform);
    }
    cout << "Done" << endl;

    double delta = (incremental_transform.matrix() - Matrix4f::Identity()).norm();
    if(delta < 0.01)
      break;
  }
}

Eigen::Affine3f ObjectMatchingCalibrator::gridSearchTransform(LossFunction::Ptr lf) const
{
  cout << "Starting grid search over transform." << endl;
  GridSearch gs(6);
  gs.max_passes_ = 1;
  gs.objective_ = lf;
  double ar = 2.0 * M_PI / 180.0;
  double tr = 0.1;
  gs.ranges_ << ar, ar, ar, tr, tr, tr;
  double minrr = 1.0 * M_PI / 180.0;
  double minrt = 0.05;
  gs.min_resolutions_ << minrr, minrr, minrr, minrt, minrt, minrt;
  double maxrr = minrr;
  double maxrt = minrt;
  gs.max_resolutions_ << maxrr, maxrr, maxrr, maxrt, maxrt, maxrt;
  double smr = 0.8;
  double smt = 0.8;
  gs.scale_multipliers_ << smr, smr, smr, smt, smt, smt;
  VectorXd init = VectorXd::Zero(6);
  VectorXd x = gs.solve(init);
  cout << "GridSearch solution: " << x.transpose() << endl;
  return generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));
}

double ObjectMatchingCalibrator::gridSearchSync(LossFunction::Ptr lf) const
{
  GridSearch gs(1);
  gs.objective_ = lf;
  gs.max_passes_ = 1;
  gs.ranges_ << 0.08;
  gs.min_resolutions_ << 0.02;
  gs.max_resolutions_ << 0.02;
  gs.scale_multipliers_ << 0.5;
  VectorXd init = VectorXd::Zero(1);
  cout << "Starting grid search over offset." << endl;
  VectorXd x = gs.solve(init);
  return x(0);
}
  
Eigen::Affine3f generateTransform(double rx, double ry, double rz,
				  double tx, double ty, double tz)

{
  Affine3f Rx, Ry, Rz, T;
  Rx = Eigen::AngleAxisf(rx, Vector3f(1, 0, 0));
  Ry = Eigen::AngleAxisf(ry, Vector3f(0, 1, 0));
  Rz = Eigen::AngleAxisf(rz, Vector3f(0, 0, 1));
  T = Eigen::Translation3f(tx, ty, tz);
  return T * Rz * Ry * Rx;
}

Cloud::Ptr ObjectMatchingCalibrator::visualizeInliers(const Eigen::Affine3f& transform) const
{

  vector<Vector3f> inlier_centroids0;
  vector<Vector3f> inlier_centroids1;
  countInliers(transform, centroids0_, centroids1_,
	       param<double>("CentroidThreshold"),
	       NULL, NULL, NULL,
	       &inlier_centroids0, &inlier_centroids1);

  Cloud seq1;
  pcl::transformPointCloud(*pull<Sequence::ConstPtr>("Sequence1")->pcds_[0],
			   seq1, transform);
   
  ROS_ASSERT(inlier_centroids1.size() == inlier_centroids0.size());
  Cloud::Ptr pcd(new Cloud);
  *pcd = *pull<Sequence::ConstPtr>("Sequence0")->pcds_[0];
  *pcd += seq1;

  pcd->reserve(pcd->size() + inlier_centroids0.size());
  for(size_t i = 0; i < inlier_centroids1.size(); ++i) {
    Point pt;
    pt.r = rand() % 255;
    pt.g = rand() % 255;
    pt.b = rand() % 255;
    
    Vector3f c = transform * inlier_centroids1[i];
    pt.x = c(0);
    pt.y = c(1);
    pt.z = c(2);
    pcd->push_back(pt);

    pt.x = inlier_centroids0[i](0);
    pt.y = inlier_centroids0[i](1);
    pt.z = inlier_centroids0[i](2);
    pcd->push_back(pt);
  }

  return pcd;
}


void ObjectMatchingCalibrator::debug() const
{
  pcl::io::savePCDFileBinary(getDebugPath() + "-ransac-rough.pcd", *visualizeInliers(rough_transform_));
  pcl::io::savePCDFileBinary(getDebugPath() + "-ransac-refined.pcd", *visualizeInliers(ransac_refined_transform_));
  pcl::io::savePCDFileBinary(getDebugPath() + "-icp.pcd", *visualizeInliers(icp_refined_transform_));
  pcl::io::savePCDFileBinary(getDebugPath() + "-gridsearch.pcd", *visualizeInliers(gridsearch_transform_));

  // -- Visualize grid search.
  for(size_t i = 0; i < gs_history_.size(); ++i) {
    VectorXd x = gs_history_[i];
    Cloud pcd1;
    Affine3f transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)) * icp_refined_transform_;
    pcl::transformPointCloud(*pull<Sequence::ConstPtr>("Sequence1")->pcds_[0],
			     pcd1, transform);
    Cloud overlay;
    overlay = *pull<Sequence::ConstPtr>("Sequence0")->pcds_[0];
    overlay += pcd1;
  }
}

void ObjectMatchingCalibrator::computeCentroids(const ObjectClouds& objects,
						std::vector< std::vector<Eigen::Vector3f> >* centroids) const
{
  centroids->clear();
  centroids->resize(objects.size());
  double total_num_pts = 0;
  for(size_t i = 0; i < objects.size(); ++i) {
    centroids->at(i).resize(objects[i].size(), Vector3f::Zero());
    for(size_t j = 0; j < objects[i].size(); ++j) {
      const Cloud& obj = *objects[i][j];
      double num = 0;
      for(size_t k = 0; k < obj.size(); ++k) { 
	if(!isinf(obj[k].x)) { 
	  centroids->at(i)[j] += obj[k].getVector3fMap();
	  ++num;
	}
      }
      centroids->at(i)[j] /= num;
      total_num_pts += num;
    }
  }

  cout << "Mean num points in objects: " << total_num_pts / (double)objects.size() << endl;
}

void ObjectMatchingCalibrator::sampleCorrespondence(const rgbd::Sequence& seq0,
						    const rgbd::Sequence& seq1,
						    const std::vector< std::vector<Eigen::Vector3f> >& centroids0,
						    const std::vector< std::vector<Eigen::Vector3f> >& centroids1,
						    pcl::TransformationFromCorrespondences* tfc) const
{
  int iter = 0;
  while(true) {
    ROS_ASSERT(iter < 10000);
    ++iter;
    
    // -- Randomly sample corresponding centroids.
    int frame_idx = rand() % seq0.size();
    const Cloud& pcd0 = *seq0.pcds_[frame_idx];
    const Cloud& pcd1 = *seq1.pcds_[frame_idx];
    double ts0 = pcd0.header.stamp.toSec();
    double ts1 = pcd1.header.stamp.toSec();
    ROS_ASSERT(fabs(ts0 - ts1) < 0.05); // See thresh in CalibrationPipelineDynamic.
    
    const vector<Vector3f>& c0 = centroids0[frame_idx];
    const vector<Vector3f>& c1 = centroids1[frame_idx];
    if(c0.empty() || c1.empty())
      continue;

    tfc->add(c1[rand() % c1.size()], c0[rand() % c0.size()]);
    return;
  }
}

  
int ObjectMatchingCalibrator::countInliers(const Eigen::Affine3f& transform,
					   const std::vector< std::vector<Eigen::Vector3f> >& centroids0,
					   const std::vector< std::vector<Eigen::Vector3f> >& centroids1,
					   double ransac_thresh,
					   Eigen::Affine3f* refined_transform,
					   std::vector<Cloud::ConstPtr>* inlier_clouds0,
					   std::vector<Cloud::ConstPtr>* inlier_clouds1,
					   std::vector<Vector3f>* inlier_centroids0,
					   std::vector<Vector3f>* inlier_centroids1) const
{
  const ObjectClouds& objects0 = *pull<const ObjectClouds*>("Objects0");
  const ObjectClouds& objects1 = *pull<const ObjectClouds*>("Objects1");
  if(inlier_clouds0 && inlier_clouds1) { 
    inlier_clouds0->clear();
    inlier_clouds0->reserve(objects0.size());
    inlier_clouds1->clear();
    inlier_clouds1->reserve(objects1.size());
  }
  if(inlier_centroids0 && inlier_centroids1) { 
    inlier_centroids0->clear();
    inlier_centroids0->reserve(objects0.size());
    inlier_centroids1->clear();
    inlier_centroids1->reserve(objects1.size());
  }
  
  pcl::TransformationFromCorrespondences tfc;
  int num_inliers = 0;
  for(size_t i = 0; i < centroids1.size(); ++i) {
    if(centroids0.empty())
      continue;
    for(size_t j = 0; j < centroids1[i].size(); ++j) {
      Vector3f transformed = transform * centroids1[i][j];

      double best_dist = numeric_limits<double>::max();
      int best_idx = -1;
      for(size_t k = 0; k < centroids0[i].size(); ++k) {
	double dist = (centroids0[i][k] - transformed).norm();
	if(dist < best_dist) {
	  best_dist = dist;
	  best_idx = k;
	}
      }

      if(best_dist < ransac_thresh) { 
	++num_inliers;
	tfc.add(centroids1[i][j], centroids0[i][best_idx]);
	if(inlier_clouds0 && inlier_clouds1) { 
	  inlier_clouds0->push_back(objects0[i][best_idx]);
	  inlier_clouds1->push_back(objects1[i][j]);
	}
	if(inlier_centroids0 && inlier_centroids1) { 
	  inlier_centroids0->push_back(centroids0[i][best_idx]);
	  inlier_centroids1->push_back(centroids1[i][j]);
	}
      }
    }
  }

  if(refined_transform)
    *refined_transform = tfc.getTransformation();
  return num_inliers;
}


Affine3f ObjectMatchingCalibrator::alignInlierModels(const vector< vector<Vector3f> >& centroids0,
						     const vector< vector<Vector3f> >& centroids1) const
{
  // -- Get inlier objects.
  vector<Cloud::ConstPtr> objects0;
  vector<Cloud::ConstPtr> objects1_const;
  countInliers(rough_transform_, centroids0, centroids1,
	       param<double>("CentroidThreshold"),
	       NULL, &objects0, &objects1_const);
  ROS_ASSERT(objects1_const.size() == objects0.size());
  
  // -- Move all objects in sensor1 by rough_transform_.
  vector<Cloud::Ptr> objects1(objects1_const.size());
  for(size_t i = 0; i < objects1_const.size(); ++i) { 
    objects1[i] = Cloud::Ptr(new Cloud);
    pcl::transformPointCloud(*objects1_const[i], *objects1[i], rough_transform_);
    ROS_ASSERT(!objects1[i]->isOrganized());
  }
  
  // -- Make KdTrees for all objects in sensor 0.
  vector<KdTree::Ptr> trees0(objects0.size());
  for(size_t i = 0; i < objects0.size(); ++i) {
    trees0[i] = KdTree::Ptr(new KdTree);
    trees0[i]->setInputCloud(objects0[i]);
  }
  
  // -- Run ICP.
  double icp_thresh = param<double>("DistanceThreshold");
  vector<int> indices;
  vector<float> distances;
  Affine3f overall_transform = rough_transform_;
  int iter = 0;
  double downsample = param<double>("Downsampling");
  while(true) {
    ++iter;
    
    pcl::TransformationFromCorrespondences tfc;
    for(size_t i = 0; i < objects1.size(); ++i) {
      const Cloud& obj1 = *objects1[i];
      const Cloud& obj0 = *objects0[i];
      for(size_t j = 0; j < obj1.size(); ++j) {
	if(((double)rand() / (double)RAND_MAX) < downsample)
	  continue;
	
	indices.clear();
	distances.clear();
	trees0[i]->nearestKSearch(obj1[j], 1, indices, distances);
	if(indices.empty() || distances[0] > icp_thresh)
	  continue;
	
	tfc.add(obj1[j].getVector3fMap(), obj0[indices[0]].getVector3fMap());
      }
    }
    
    Affine3f incremental_transform = tfc.getTransformation();
    overall_transform = incremental_transform * overall_transform;
    double delta = (incremental_transform.matrix() - Matrix4f::Identity()).norm();
    cout << "ICP iter " << iter << ", delta " << delta << endl;
    if(delta < 0.01)
      break;
    
    // -- Move all the objects in sensor 1 by incremental_transform.
    for(size_t i = 0; i < objects1.size(); ++i)
      pcl::transformPointCloud(*objects1[i], *objects1[i], incremental_transform);
  }
  
  return overall_transform;
}

LossFunction::LossFunction(double max_dist,
			   double dt_thresh,
			   const std::vector<rgbd::Cloud::Ptr>& pcds0,
			   const std::vector<rgbd::Cloud::Ptr>& pcds1) :
  max_dist_(max_dist),
  dt_thresh_(dt_thresh),
  pcds0_(pcds0),
  pcds1_(pcds1)
{
  trees0_.resize(pcds0_.size());
  for(size_t i = 0; i < pcds0_.size(); ++i) {
    ROS_ASSERT(pcds0_[i]->isOrganized()); // For projection.
    trees0_[i] = KdTree::Ptr(new KdTree);
    trees0_[i]->setInputCloud(pcds0_[i]);
  }
}

double LossFunction::eval(const Eigen::VectorXd& x)
{
  Affine3f transform = Affine3f::Identity();
  double sync = 0;
  if(x.rows() == 1) {
    sync = x(0);
  }
  else if(x.rows() == 6) {
    transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));
  }

  double val = 0;
  double count = 0;
  for(size_t i = 0; i < pcds1_.size(); ++i) {
    const Cloud& pcd1 = *pcds1_[i];
    double ts1 = pcd1.header.stamp.toSec() + sync;
    int idx = seek(ts1);
    if(idx == -1)
      continue;
    
    val += computeLoss(trees0_[idx], *pcds0_[idx], pcd1, transform);
    ++count;
  }
  
  return val / count;
}
			   
double LossFunction::computeLoss(KdTree::Ptr tree0, const Cloud& pcd0,
				 const Cloud& pcd1, const Eigen::Affine3f& transform) const
				 
{
  vector<int> indices;
  vector<float> distances;
  double count = 0;
  double val = 0;
  Point transformed;
  for(size_t i = 0; i < pcd1.size(); ++i) {
    const Point& pt = pcd1[i];
    if(!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z))
      continue;
    ++count;
    transformed.getVector3fMap() = transform * pt.getVector3fMap();

    int u, v;
    int idx = projectPoint(pcd0, transformed, &u, &v);
    double fsv = max_dist_;
    if(idx != -1 && pcl_isfinite(pcd0[idx].z))
      fsv = fmin(max_dist_, fmax(0.0, pcd0[idx].z - pt.z));
    val += fsv;

    indices.clear();
    distances.clear();
    tree0->nearestKSearch(transformed, 1, indices, distances);
    if(indices.empty() || distances[0] > max_dist_)
      val += max_dist_;
    else { 
      val += distances[0];
      // const Point& pt0 = pcd0[indices[0]];
      // val += sqrt(pow((float)(pt.r-pt0.r)/255.0, 2.0) +
      // 		  pow((float)(pt.b-pt0.b)/255.0, 2.0) +
      // 		  pow(((float)pt.g-pt0.g)/255.0, 2.0));
    }
  }

  return val / count;
}

int projectPoint(const rgbd::Cloud& pcd, const rgbd::Point& pt,
		 int* u, int* v)
{
  ROS_ASSERT(pcd.isOrganized());
  ROS_ASSERT(pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z));

  double f = 525; // Oh yes I did.
  double centerX = (pcd.width >> 1 );
  double centerY = (pcd.height >> 1);

  *u = f * pt.x / pt.z + centerX;
  *v = f * pt.y / pt.z + centerY;

  int idx = *v * pcd.width + *u;
  if(*u < 0 || *u >= (int)pcd.width)
    idx = -1;
  if(*v < 0 || *v >= (int)pcd.height)
    idx = -1;

  return idx;
}

int LossFunction::seek(double ts1) const
{
  int idx = -1;
  double min = numeric_limits<double>::max();
  // TODO: This could be faster than linear search.
  for(size_t i = 0; i < pcds0_.size(); ++i) {
    double ts0 = pcds0_[i]->header.stamp.toSec();
    double dt = fabs(ts0 - ts1);
    if(dt < min) {
      min = dt;
      idx = i;
    }
  }

  if(min < dt_thresh_)
    return idx;
  else
    return -1;
}


  
