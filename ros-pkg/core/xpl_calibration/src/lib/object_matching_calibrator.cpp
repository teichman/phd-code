#include <xpl_calibration/object_matching_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

/************************************************************
 * ObjectMatchingCalibrator
 ************************************************************/

Cloud::Ptr downsampleCloud(const Cloud& orig, double ds)
{
  Cloud::Ptr pcd(new Cloud);
  pcd->reserve(orig.size());
  pcd->header = orig.header;
  for(size_t i = 0; i < orig.size(); ++i) {
    if(((double)rand() / (double)RAND_MAX) < ds)
      continue;
    pcd->push_back(orig[i]);
  }

  ROS_ASSERT(!pcd->empty());
  return pcd;
}

void ObjectMatchingCalibrator::accumulateObjects(const ObjectClouds& objects,
						 std::vector<Cloud::Ptr>* pcds) const
{
  pcds->clear();
  for(size_t i = 0; i < objects.size(); ++i) {
    if(objects[i].empty())
      continue;
    
    Cloud::Ptr pcd(new Cloud);
    pcd->header = objects[i][0]->header;

    double downsample = param<double>("Downsampling");
    for(size_t j = 0; j < objects[i].size(); ++j) {
      pcd->reserve(pcd->size() + objects[i][j]->size());
      for(size_t k = 0; k < objects[i][j]->size(); ++k) {
	if(((double)rand() / (double)RAND_MAX) < downsample)
	  continue;
	pcd->push_back(objects[i][j]->at(k));
      }
    }

    ROS_ASSERT(pcd->size() > 0);
    pcds->push_back(pcd);
  }
}

void ObjectMatchingCalibrator::getData(std::vector<KdTree::Ptr>* trees0,
					 std::vector<Cloud::ConstPtr>* pcds0,
					 std::vector<Cloud::Ptr>* pcds1) const
{
  trees0->clear();
  pcds0->clear();
  pcds1->clear();

  // -- Get floating objects, with downsampling.
  const ObjectClouds& objects1 = *pull<const ObjectClouds*>("Objects1");
  accumulateObjects(objects1, pcds1);
  // Also get one floating frame.
  //const Sequence& seq1 = *pull<Sequence::ConstPtr>("Sequence1");
  //pcds1->push_back(downsampleCloud(*seq1.pcds_[seq1.pcds_.size() / 2], param<double>("Downsampling")));
  //pcds1->push_back(downsampleCloud(*seq1.pcds_[seq1.pcds_.size() / 2], 0.0));

  // -- Get entire reference frames without downsampling.
  const Sequence& seq0 = *pull<Sequence::ConstPtr>("Sequence0");
  for(size_t i = 0; i < seq0.pcds_.size(); ++i)
    pcds0->push_back(seq0.pcds_[i]);
  
  // -- Make KdTrees for all reference pcds.
  for(size_t i = 0; i < pcds0->size(); ++i) {
    KdTree::Ptr tree(new KdTree);
    tree->setInputCloud(pcds0->at(i));
    trees0->push_back(tree);
  }

  cout << "Pcd sizes: " << endl;
  cout << trees0->size() << endl;
  cout << pcds0->size() << endl;
  cout << pcds1->size() << endl;
}

void ObjectMatchingCalibrator::compute()
{
  checkInput();
  ransac_transform_ = centroidRansac();

  vector<KdTree::Ptr> trees0;
  vector<Cloud::ConstPtr> pcds0;
  vector<Cloud::Ptr> pcds1;
  getData(&trees0, &pcds0, &pcds1);

  for(size_t i = 0; i < pcds1.size(); ++i)
    pcl::transformPointCloud(*pcds1[i], *pcds1[i], ransac_transform_);
  
  outer_iter_ = 0;
  transform_ = ransac_transform_;
  sync_ = 0;
  while(true) {
    cout << "========================================" << endl;
    cout << "Outer iteration " << outer_iter_ << endl;
    cout << "========================================" << endl;

    Affine3f incremental;
    if(param<string>("TransformSearchMethod").compare("ICP") == 0)
      incremental = updateTransformICP(trees0, pcds0, pcds1);
    else if(param<string>("TransformSearchMethod").compare("GridSearch") == 0)
      incremental = updateTransformGS(trees0, pcds0, pcds1);
    else
      abort();
    transform_ = incremental * transform_;
    double delta = (incremental.matrix() - Matrix4f::Identity()).norm();
    
    cout << "Incremental transform: " << endl << incremental.matrix() << endl;
    cout << "Current final transform: " << endl << transform_.matrix() << endl;
    cout << "Outer delta: " << delta << endl;
    if(debug_) {
      ostringstream oss;
      oss << "transform-outer-iter" << setw(4) << setfill('0') << outer_iter_;
      visualizeResult(oss.str(), transform_, sync_);
    }
    
    if(delta < param<double>("TransformThreshold"))
      break;

    double dt = updateSync(trees0, pcds0, pcds1);
    sync_ += dt;
    cout << "dt: " << dt << endl;
    cout << "Current sync offset: " << sync_ << endl;
    if(debug_) {
      ostringstream oss;
      oss << "sync-outer-iter" << setw(4) << setfill('0') << outer_iter_;
      visualizeResult(oss.str(), transform_, sync_);
    }
    
    ++outer_iter_;
  }

  visualizeResult("final-transform", transform_, sync_);
  
  push<const Affine3f*>("CentroidRansacTransform", &ransac_transform_);
  push<const Affine3f*>("FinalTransform", &transform_);
  push<double>("SyncOffset", sync_);
}

double ObjectMatchingCalibrator::updateSync(const std::vector<KdTree::Ptr>& trees0,
					    const std::vector<Cloud::ConstPtr>& pcds0,
					    const std::vector<Cloud::Ptr>& pcds1) const
{
  LossFunction::Ptr lf(new LossFunction(trees0, pcds0, pcds1, getParams()));
  double dt = gridSearchSync(lf);
  for(size_t i = 0; i < pcds1.size(); ++i) {
    double ts = pcds1[i]->header.stamp.toSec();
    pcds1[i]->header.stamp.fromSec(ts + dt);
  }
  return dt;
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
    visualizeResult("ransac-rough", best_transform, 0);
    visualizeResult("ransac-refined", refined, 0);
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
  gs.max_resolutions_ << 0.025;
  gs.grid_radii_ << 2;
  gs.scale_factors_ << 0.5;
  gs.num_scalings_ = 4;
  
  cout << "Starting grid search over offset." << endl;
  ArrayXd x = gs.search(ArrayXd::Zero(1));
  return x(0);
}

void ObjectMatchingCalibrator::visualizeResult(const std::string& name, const Eigen::Affine3f& transform, double sync) const
{
  const Sequence& seq0 = *pull<Sequence::ConstPtr>("Sequence0");
  const Sequence& seq1 = *pull<Sequence::ConstPtr>("Sequence1");

  double max_dt = 0.3; // More lenient than that used for calibration.
  for(size_t i = 0; i < seq1.size(); i+=100) {
    int idx = seek(seq0.pcds_, seq1.pcds_[i]->header.stamp.toSec() + sync, max_dt);
    if(idx == -1)
      continue;
    
    Cloud overlay;
    pcl::transformPointCloud(*seq1.pcds_[i], overlay, transform);
    overlay += *seq0.pcds_[idx];

    ostringstream oss;
    oss << getDebugPath() << "-" << name << "-overlay" << setw(4) << setfill('0') << i << ".pcd";
    pcl::io::savePCDFileBinary(oss.str(), overlay);
  }
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
    ROS_ASSERT(iter < 100000);
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

Eigen::Affine3f ObjectMatchingCalibrator::gridSearchTransform(ScalarFunction::Ptr lf) const
{
  cout << "Starting grid search over transform." << endl;
  GridSearch gs(6);
  gs.objective_ = lf;
  gs.num_scalings_ = 5;
  double maxrr = 2.0 * M_PI / 180.0;
  double maxrt = 0.2;
  gs.max_resolutions_ << maxrr, maxrr, maxrr, maxrt, maxrt, maxrt;
  int gr = 3;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.

  ArrayXd x = gs.search(ArrayXd::Zero(6));
  cout << "GridSearch solution: " << x.transpose() << endl;
  return generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));
}

Affine3f ObjectMatchingCalibrator::updateTransformGS(const std::vector<KdTree::Ptr>& trees0,
						     const std::vector<Cloud::ConstPtr>& pcds0,
						     const std::vector<Cloud::Ptr>& pcds1) const
{
  Affine3f transform = Affine3f::Identity();
  int iter = 0;
  while(true) {
    // -- Use grid search to find the next best move.
    LossFunction::Ptr lf(new LossFunction(trees0, pcds0, pcds1, getParams()));
    Affine3f incremental = gridSearchTransform(lf);
    transform = incremental * transform;
    
    // -- Apply this move to the data in sensor 1.
    for(size_t i = 0; i < pcds1.size(); ++i)
      pcl::transformPointCloud(*pcds1[i], *pcds1[i], incremental);

    // -- If we haven't moved much, stop.
    double delta = (incremental.matrix() - Matrix4f::Identity()).norm();
    cout << "  Grid search move iter " << iter << ", delta " << delta << endl;
    if(delta < param<double>("TransformThreshold"))
      break;
    
    ++iter;
  }

  return transform;
}

Affine3f ObjectMatchingCalibrator::updateTransformICP(const std::vector<KdTree::Ptr>& trees0,
						      const std::vector<Cloud::ConstPtr>& pcds0,
						      const std::vector<Cloud::Ptr>& pcds1) const
{
  Affine3f transform = Affine3f::Identity();
  
  double max_dist = param<double>("DistanceThreshold");
  vector<int> indices;
  vector<float> distances;
  int iter = 0;
  while(true) {
    if(debug_) {
      ostringstream oss;
      oss << "icp-outer-iter"  << setw(4) << setfill('0') << outer_iter_
	  << "-inner-iter" << setw(4) << setfill('0') << iter;
    }
    ++iter;
    
    pcl::TransformationFromCorrespondences tfc;
    for(size_t i = 0; i < pcds1.size(); ++i) {
      int idx = seek(pcds0, pcds1[i]->header.stamp.toSec(),
		     param<double>("TimeCorrespondenceThreshold"));
      if(idx == -1)
	continue;
      
      KdTree::Ptr tree0 = trees0[idx];
      const Cloud& pcd0 = *pcds0[idx];
      const Cloud& pcd1 = *pcds1[i];
            
      for(size_t j = 0; j < pcd1.size(); ++j) {
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
    if(delta < param<double>("TransformThreshold"))
      break;
    
    // -- Move all clouds in sensor 1 by incremental.
    for(size_t i = 0; i < pcds1.size(); ++i)
      pcl::transformPointCloud(*pcds1[i], *pcds1[i], incremental);
  }
  
  return transform;
}


/************************************************************
 * LossFunction
 ************************************************************/

LossFunction::LossFunction(const std::vector<KdTree::Ptr>& trees0,
			   const std::vector<Cloud::ConstPtr>& pcds0,
			   const std::vector<Cloud::Ptr>& pcds1,
			   const pipeline::Params& params) :
  trees0_(trees0),
  pcds0_(pcds0),
  pcds1_(pcds1),
  use_fsv_(true)
{
  ROS_ASSERT(trees0_.size() == pcds0_.size());
  for(size_t i = 0; i < pcds0_.size(); ++i)
    ROS_ASSERT(pcds0_[i]->isOrganized());
  for(size_t i = 0; i < pcds1_.size(); ++i)
    ROS_ASSERT(!pcds1_[i]->isOrganized());

  dt_thresh_ = params.get<double>("TimeCorrespondenceThreshold");
  max_dist_ = params.get<double>("DistanceThreshold");
  fx_ = params.get<double>("Seq0Fx");
  fy_ = params.get<double>("Seq0Fy");
  cx_ = params.get<double>("Seq0Cx");
  cy_ = params.get<double>("Seq0Cy");
}

double LossFunction::eval(const VectorXd& x) const
{
  ROS_ASSERT(x.rows() == 1 || x.rows() == 6);
  double dt = 0;
  Affine3f transform = Affine3f::Identity();
  if(x.rows() == 1)
    dt = x(0);
  else
    transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));
    
  double count = 0;
  double val = 0;
  Cloud transformed;
  for(size_t i = 0; i < pcds1_.size(); ++i) {
    int idx = seek(pcds0_, dt + pcds1_[i]->header.stamp.toSec(), dt_thresh_);
    if(idx == -1)
      continue;
    ++count;
    
    KdTree::Ptr tree0 = trees0_[idx];
    const Cloud& pcd0 = *pcds0_[idx];
    const Cloud& pcd1 = *pcds1_[i];
    if(x.rows() == 1)
      val += computeLoss(tree0, pcd0, pcd1);
    else {
      transformed.clear();
      pcl::transformPointCloud(pcd1, transformed, transform);
      val += computeLoss(tree0, pcd0, transformed);
    }
  }
  return val / count;
}

double LossFunction::computeLoss(KdTree::Ptr tree0, const Cloud& pcd0, const Cloud& pcd1) const
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

    if(use_fsv_) {
      int u, v;
      int idx = projectPoint(pcd0, pt, &u, &v);
      double fsv = max_dist_;
      if(idx != -1 && pcl_isfinite(pcd0[idx].z))
	fsv = fmin(max_dist_, fmax(0.0, pcd0[idx].z - pt.z));
      val += fsv;
    }
        
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

int LossFunction::projectPoint(const rgbd::Cloud& pcd, const rgbd::Point& pt,
			       int* u, int* v) const
{
  ROS_ASSERT(pcd.isOrganized());
  ROS_ASSERT(pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z));

  *u = fx_ * pt.x / pt.z + cx_;
  *v = fy_ * pt.y / pt.z + cy_;
  
  int idx = *v * pcd.width + *u;
  if(*u < 0 || *u >= (int)pcd.width)
    idx = -1;
  if(*v < 0 || *v >= (int)pcd.height)
    idx = -1;
  
  return idx;
}


/************************************************************
 * Helper functions
 ************************************************************/

int seek(const std::vector<Cloud::Ptr>& pcds0, double ts1, double dt_thresh)
{
  vector<Cloud::ConstPtr> cc;
  for(size_t i = 0; i < pcds0.size(); ++i)
    cc.push_back(pcds0[i]);

  return seek(cc, ts1, dt_thresh);
}

int seek(const std::vector<Cloud::ConstPtr>& pcds0, double ts1, double dt_thresh)
{
  int idx = -1;
  double min = numeric_limits<double>::max();
  // TODO: This could be faster than linear search.
  for(size_t i = 0; i < pcds0.size(); ++i) {
    double ts0 = pcds0[i]->header.stamp.toSec();
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