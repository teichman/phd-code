#include <xpl_calibration/object_matching_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

void ObjectMatchingCalibrator::extractScenes(const ObjectClouds& objects,
					     std::vector<Cloud::Ptr>* scenes) const
{
  scenes->clear();
  
  for(size_t i = 0; i < objects.size(); ++i) {
    if(objects[i].empty())
      continue;
    
    Cloud::Ptr scene(new Cloud);
    scene->header = objects[i][0]->header;
    for(size_t j = 0; j < objects[i].size(); ++j)
      *scene += *objects[i][j];

    ROS_ASSERT(scene->size() > 0);
    scenes->push_back(scene);
  }
}

void ObjectMatchingCalibrator::getScenes(std::vector<KdTree::Ptr>* trees0,
					 std::vector<Cloud::ConstPtr>* scenes0,
					 std::vector<Cloud::Ptr>* scenes1) const
{
  trees0->clear();
  scenes0->clear();
  scenes1->clear();
  
  const ObjectClouds& objects0 = *pull<const ObjectClouds*>("Objects0");
  const ObjectClouds& objects1 = *pull<const ObjectClouds*>("Objects1");
  extractScenes(objects1, scenes1);
  vector<Cloud::Ptr> tmp;
  extractScenes(objects0, &tmp);
  for(size_t i = 0; i < tmp.size(); ++i)
    scenes0->push_back(tmp[i]);
  
  // -- Make KdTrees for all reference scenes.
  for(size_t i = 0; i < scenes0->size(); ++i) {
    KdTree::Ptr tree(new KdTree);
    tree->setInputCloud(scenes0->at(i));
    trees0->push_back(tree);
  }
}

void ObjectMatchingCalibrator::compute()
{
  checkInput();
  ransac_transform_ = centroidRansac();

  vector<KdTree::Ptr> trees0;
  vector<Cloud::ConstPtr> scenes0;
  vector<Cloud::Ptr> scenes1;
  getScenes(&trees0, &scenes0, &scenes1);

  for(size_t i = 0; i < scenes1.size(); ++i)
    pcl::transformPointCloud(*scenes1[i], *scenes1[i], ransac_transform_);
  visualizeScenes("ransac", scenes0, scenes1);
  
  outer_iter_ = 0;
  icp_transform_ = ransac_transform_;
  sync_ = 0;
  while(true) {
    cout << "========================================" << endl;
    cout << "Outer iteration " << outer_iter_ << endl;
    cout << "========================================" << endl;

    Affine3f incremental = updateICP(trees0, scenes0, scenes1);
    icp_transform_ = incremental * icp_transform_;
    double delta = (incremental.matrix() - Matrix4f::Identity()).norm();
    
    cout << "Incremental transform: " << endl << incremental.matrix() << endl;
    cout << "Current final transform: " << endl << icp_transform_.matrix() << endl;
    cout << "Outer delta: " << delta << endl;
    if(debug_) {
      ostringstream oss;
      oss << "icp-outer-iter" << setw(4) << setfill('0') << outer_iter_;
      visualizeScenes(oss.str(), scenes0, scenes1);
    }
    
    if(delta < param<double>("ICPTransformThreshold"))
      break;

    double dt = updateSync(trees0, scenes0, scenes1);
    sync_ += dt;
    cout << "dt: " << dt << endl;
    cout << "Current sync offset: " << sync_ << endl;
    if(debug_) {
      ostringstream oss;
      oss << "sync-iter" << setw(4) << setfill('0') << outer_iter_;
      visualizeScenes(oss.str(), scenes0, scenes1);
    }
    
    ++outer_iter_;
  }

  visualizeTransform("icp-final", icp_transform_);
  
  push<double>("SyncOffset", sync_);
  push<const Affine3f*>("CentroidRansacTransform", &ransac_transform_);
  push<const Affine3f*>("IcpTransform", &icp_transform_);
  final_transform_ = icp_transform_;
  push<const Affine3f*>("FinalTransform", &final_transform_);
}

double ObjectMatchingCalibrator::updateSync(const std::vector<KdTree::Ptr>& trees0,
					    const std::vector<Cloud::ConstPtr>& scenes0,
					    const std::vector<Cloud::Ptr>& scenes1) const
{
  SyncLossFunction::Ptr slf(new SyncLossFunction(trees0, scenes0, scenes1,
						 param<double>("TimeCorrespondenceThreshold"),
						 param<double>("CentroidThreshold")));
  double dt = gridSearchSync(slf);
  for(size_t i = 0; i < scenes1.size(); ++i) {
    double ts = scenes1[i]->header.stamp.toSec();
    scenes1[i]->header.stamp.fromSec(ts + dt);
  }
  return dt;
}

SyncLossFunction::SyncLossFunction(const std::vector<KdTree::Ptr>& trees0,
				   const std::vector<Cloud::ConstPtr>& scenes0,
				   const std::vector<Cloud::Ptr>& scenes1,
				   double dt_thresh,
				   double max_dist) :
  trees0_(trees0),
  scenes0_(scenes0),
  scenes1_(scenes1),
  dt_thresh_(dt_thresh),
  max_dist_(max_dist)
{
}

double SyncLossFunction::eval(const VectorXd& x) const
{
  ROS_ASSERT(x.rows() == 1);
  double dt = x(0);

  double count = 0;
  double val = 0;
  for(size_t i = 0; i < scenes1_.size(); ++i) {
    int idx = seek(scenes0_, dt + scenes1_[i]->header.stamp.toSec(), dt_thresh_);
    if(idx == -1)
      continue;
    ++count;
    
    KdTree::Ptr tree0 = trees0_[idx];
    const Cloud& pcd0 = *scenes0_[idx];
    const Cloud& pcd1 = *scenes1_[i];
    val += computeLoss(tree0, pcd0, pcd1);
  }
  return val / count;
}

double SyncLossFunction::computeLoss(KdTree::Ptr tree0, const Cloud& pcd0, const Cloud& pcd1) const
{
  vector<int> indices;
  vector<float> distances;
  double count = 0;
  double val = 0;
  for(size_t i = 0; i < pcd1.size(); ++i) {
    const Point& pt = pcd1[i];
    if(!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z))
      continue;
    ++count;

    indices.clear();
    distances.clear();
    tree0->nearestKSearch(pt, 1, indices, distances);
    if(indices.empty() || distances[0] > max_dist_)
      val += max_dist_;
    else 
      val += distances[0];
  }
  return val / count;
}

Eigen::Affine3f ObjectMatchingCalibrator::centroidRansac() const
{
  // Compute centroids for all objects.
  const ObjectClouds& objects0 = *pull<const ObjectClouds*>("Objects0");
  const ObjectClouds& objects1 = *pull<const ObjectClouds*>("Objects1");
  std::vector< std::vector<Eigen::Vector3f> > centroids0;
  std::vector< std::vector<Eigen::Vector3f> > centroids1;
  computeCentroids(objects0, &centroids0);
  computeCentroids(objects1, &centroids1);
  const Sequence& seq0 = *pull<Sequence::ConstPtr>("Sequence0");
  const Sequence& seq1 = *pull<Sequence::ConstPtr>("Sequence1");

  // RANSAC.
  int num_iterations = param<int>("NumRansacIters");
  int num_correspondences = param<int>("NumCorrespondences");
  Affine3f best_transform = Affine3f::Identity();
  int best_num_inliers = 0;
  for(int i = 0; i < num_iterations; ++i) {
    cout << "RANSAC iter " << i << endl;
    pcl::TransformationFromCorrespondences tfc;
    for(int j = 0; j < num_correspondences; ++j)
      sampleCorrespondence(seq0, seq1, centroids0, centroids1, &tfc);
    
    // Compute transform.
    Affine3f transform = tfc.getTransformation();

    // Count up inliers.
    int num_inliers = countInliers(transform, centroids0, centroids1,
				   param<double>("CentroidThreshold"));
    
    
    // Save this transform if it's the best.
    if(num_inliers > best_num_inliers) {
      best_num_inliers = num_inliers;
      best_transform = transform;
      cout << "Best transform has " << num_inliers << " inliers." << endl;
    }
  }

  Affine3f refined;
  cout << "Num inliers before refinement: " << best_num_inliers << endl;
  int new_num_inliers = countInliers(best_transform, centroids0, centroids1,
				     param<double>("CentroidThreshold"), &refined);
  cout << "Num inliers after refinement: " << new_num_inliers << endl;

  if(debug_) {
    visualizeTransform("ransac-rough", best_transform);
    visualizeTransform("ransac-refined", refined);
    visualizeInliers("ransac-inliers-rough", best_transform);
    visualizeInliers("ransac-inliers-refined", refined);
  }
  return refined;
}

void ObjectMatchingCalibrator::checkInput() const
{
  const Sequence& seq0 = *pull<Sequence::ConstPtr>("Sequence0");
  const Sequence& seq1 = *pull<Sequence::ConstPtr>("Sequence1");
  const ObjectClouds& objects0 = *pull<const ObjectClouds*>("Objects0");
  const ObjectClouds& objects1 = *pull<const ObjectClouds*>("Objects1");
  ROS_ASSERT(seq0.size() == seq1.size());
  ROS_ASSERT(seq0.size() == objects0.size());
  ROS_ASSERT(seq0.size() == objects1.size());
}

double ObjectMatchingCalibrator::gridSearchSync(ScalarFunction::Ptr lf) const
{
  GridSearch gs(1);
  gs.objective_ = lf;
  gs.max_passes_ = 1;
  gs.ranges_ << 0.05;
  gs.min_resolutions_ << 0.003;
  gs.max_resolutions_ << 0.01;
  gs.scale_multipliers_ << 0.5;
  VectorXd init = VectorXd::Zero(1);
  cout << "Starting grid search over offset." << endl;
  VectorXd x = gs.solve(init);
  return x(0);
}


void ObjectMatchingCalibrator::visualizeScenes(const std::string& name,
					       const std::vector<Cloud::ConstPtr>& scenes0,
					       const std::vector<Cloud::Ptr>& scenes1) const
{
  for(size_t i = 0; i < scenes1.size(); ++i) {
    int idx = seek(scenes0, scenes1[i]->header.stamp.toSec(), param<double>("TimeCorrespondenceThreshold"));
    if(idx == -1)
      continue;

    Cloud overlay = *scenes0[idx];
    for(size_t j = 0; j < overlay.size(); ++j) {
      overlay[j].r = 255;
      overlay[j].g = 0;
      overlay[j].b = 0;
    }
    
    overlay += *scenes1[i];
    for(size_t j = scenes0[idx]->size(); j < overlay.size(); ++j) {
      overlay[j].r = 0;
      overlay[j].g = 0;
      overlay[j].b = 255;
    }

    ostringstream oss;
    oss << getDebugPath() << "-" << name << "-scene" << setw(4) << setfill('0') << i << ".pcd";
    cout << "Saving to " << oss.str() << endl;
    pcl::io::savePCDFileBinary(oss.str(), overlay);
  }
}


void ObjectMatchingCalibrator::visualizeTransform(const std::string& name, const Eigen::Affine3f& transform) const
{
  Cloud overlay;
  pcl::transformPointCloud(*pull<Sequence::ConstPtr>("Sequence1")->pcds_[0], overlay, transform);
  overlay += *pull<Sequence::ConstPtr>("Sequence0")->pcds_[0];
  pcl::io::savePCDFileBinary(getDebugPath() + "-" + name + ".pcd", overlay);
}

void ObjectMatchingCalibrator::visualizeInliers(const std::string& name, const Eigen::Affine3f& transform) const
{
  const ObjectClouds& objects0 = *pull<const ObjectClouds*>("Objects0");
  const ObjectClouds& objects1 = *pull<const ObjectClouds*>("Objects1");
  std::vector< std::vector<Eigen::Vector3f> > centroids0;
  std::vector< std::vector<Eigen::Vector3f> > centroids1;
  computeCentroids(objects0, &centroids0);
  computeCentroids(objects1, &centroids1);
  vector<Vector3f> inlier_centroids0;
  vector<Vector3f> inlier_centroids1;
  countInliers(transform, centroids0, centroids1,
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

  pcl::io::savePCDFileBinary(getDebugPath() + "-" + name + ".pcd", *pcd);
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
	if(pcl_isfinite(obj[k].x)) { 
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

int seek(const std::vector<Cloud::ConstPtr>& scenes0, double ts1, double dt_thresh)
{
  int idx = -1;
  double min = numeric_limits<double>::max();
  // TODO: This could be faster than linear search.
  for(size_t i = 0; i < scenes0.size(); ++i) {
    double ts0 = scenes0[i]->header.stamp.toSec();
    double dt = fabs(ts0 - ts1);
    if(dt < min) {
      min = dt;
      idx = i;
    }
  }

  if(min < dt_thresh)
    return idx;
  else
    return -1;
}

Affine3f ObjectMatchingCalibrator::updateICP(const std::vector<KdTree::Ptr>& trees0,
					     const std::vector<Cloud::ConstPtr>& scenes0,
					     const std::vector<Cloud::Ptr>& scenes1) const
{
  Affine3f transform = Affine3f::Identity();
  
  double max_dist = param<double>("ICPDistanceThreshold");
  vector<int> indices;
  vector<float> distances;
  int iter = 0;
  double downsample = param<double>("ICPDownsampling");
  while(true) {
    if(debug_) {
      ostringstream oss;
      oss << "icp-outer-iter"  << setw(4) << setfill('0') << outer_iter_
	  << "-inner-iter" << setw(4) << setfill('0') << iter;
      visualizeScenes(oss.str(), scenes0, scenes1);
    }
    ++iter;
    
    pcl::TransformationFromCorrespondences tfc;
    for(size_t i = 0; i < scenes1.size(); ++i) {
      int idx = seek(scenes0, scenes1[i]->header.stamp.toSec(),
		     param<double>("TimeCorrespondenceThreshold"));
      if(idx == -1)
	continue;
      
      KdTree::Ptr tree0 = trees0[idx];
      const Cloud& pcd0 = *scenes0[idx];
      const Cloud& pcd1 = *scenes1[i];
            
      for(size_t j = 0; j < pcd1.size(); ++j) {
	if(((double)rand() / (double)RAND_MAX) < downsample)
	  continue;
	
	indices.clear();
	distances.clear();
	tree0->nearestKSearch(pcd1[j], 1, indices, distances);
	if(indices.empty() || distances[0] > max_dist)
	  continue;
	
	tfc.add(pcd1[j].getVector3fMap(), pcd0[indices[0]].getVector3fMap());
      }
    }
    
    Affine3f incremental = tfc.getTransformation();
    transform = incremental * transform;
    double delta = (incremental.matrix() - Matrix4f::Identity()).norm();
    cout << "  Inner ICP iter " << iter << ", delta " << delta << endl;
    if(delta < param<double>("ICPTransformThreshold"))
      break;
    
    // -- Move all the scenes in sensor 1 by incremental.
    for(size_t i = 0; i < scenes1.size(); ++i)
      pcl::transformPointCloud(*scenes1[i], *scenes1[i], incremental);
  }
  
  return transform;
}
