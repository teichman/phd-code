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
  //    TODO: Make this use CorrespondenceManager.
  
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

   // -- Set up CorrespondenceManager.
  cm_.clear();
  cm_.dt_thresh_ = param<double>("TimeCorrespondenceThreshold");
  cm_.centroid_thresh_ = param<double>("CentroidThreshold");
  cm_.dist_thresh_ = param<double>("DistanceThreshold");

  // Add reference objects without changing them.
  for(size_t i = 0; i < objects0.size(); ++i)
    for(size_t j = 0; j < objects0[i].size(); ++j)
      cm_.addReferenceObject(objects0[i][j]);

  // Add other objects, transformed as learned above.
  for(size_t i = 0; i < objects1.size(); ++i) { 
    for(size_t j = 0; j < objects1[i].size(); ++j) {
      Cloud::Ptr obj = Cloud::Ptr(new Cloud);
      //pcl::transformPointCloud(*objects1[i][j], *obj, ransac_refined_transform_);
      pcl::transformPointCloud(*objects1[i][j], *obj, icp_refined_transform_);
      cm_.addFloatingObject(obj);
    }
  }
  double loss0 = cm_.computeLoss(0.2);
  cout << "Initial loss: " << loss0 << endl;
  cm_.applyTimeOffset(100);
  double loss1 = cm_.computeLoss(0.2);
  cout << "Loss with bad time offset: " << loss1 << endl;
  cm_.applyTimeOffset(0);
  double loss2 = cm_.computeLoss(0.2);
  cout << "Initial loss: " << loss2 << endl;
  cm_.applyTransform(generateTransform(0, 0, 0, 100, 0, 0));
  double loss3 = cm_.computeLoss(0.2);
  cout << "Loss with bad transform: " << loss3 << endl;
  cm_.applyTransform(generateTransform(0, 0, 0, 0, 0, 0));
  double loss4 = cm_.computeLoss(0.2);
  cout << "Initial loss: " << loss4 << endl;

  
  // -- Grid search.
  //    dt, rx, ry, rz, tx, ty, tz
  GridSearch gs(7);
  gs.objective_ = LossFunction::Ptr(new LossFunction(&cm_));
  double ar = 20.0 * M_PI / 180.0;
  double tr = 2.0;
  gs.ranges_ << 0.2, ar, ar, ar, tr, tr, tr;
  double minrr = 0.2 * M_PI / 180.0;
  double minrt = 0.02;
  gs.min_resolutions_ << 0.01, minrr, minrr, minrr, minrt, minrt, minrt;
  double maxrr = 10.0 * M_PI / 180.0;
  double maxrt = 1.0;
  gs.max_resolutions_ << 0.1, maxrr, maxrr, maxrr, maxrt, maxrt, maxrt;
  double smr = 0.8;
  double smt = 0.8;
  gs.scale_multipliers_ << 0.8, smr, smr, smr, smt, smt, smt;
  
  VectorXd init = VectorXd::Zero(7);
  VectorXd x = gs.solve(init);
  double best_dt = x(0);
  cout << "Final GridSearch solution: " << x.transpose() << endl;
  gridsearch_transform_ = generateTransform(x(1), x(2), x(3), x(4), x(5), x(6)) * icp_refined_transform_;
  gs_history_ = gs.history_;
  ROS_ASSERT(!gs_history_.empty());
  
  push("SyncOffset", best_dt);
  push<const Affine3f*>("RansacRoughTransform", &rough_transform_);
  push<const Affine3f*>("RansacRefinedTransform", &ransac_refined_transform_);
  push<const Affine3f*>("IcpRefinedTransform", &icp_refined_transform_);
  push<const Affine3f*>("GridSearchTransform", &gridsearch_transform_);
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
    Affine3f transform = generateTransform(x(1), x(2), x(3), x(4), x(5), x(6)) * icp_refined_transform_;
    pcl::transformPointCloud(*pull<Sequence::ConstPtr>("Sequence1")->pcds_[0],
			     pcd1, transform);
    Cloud overlay;
    overlay = *pull<Sequence::ConstPtr>("Sequence0")->pcds_[0];
    overlay += pcd1;

    ostringstream oss;
    oss << getDebugPath() << "-gsiter" << setw(4) << setfill('0') << i << ".pcd";
    pcl::io::savePCDFileBinary(oss.str(), overlay);

    for(size_t j = 0; j < cm_.correspondences_.size(); ++j) {
      Correspondence c = cm_.correspondences_[j];
      if(!c.obj0_ || !c.obj1_)
	continue;

      Cloud overlay;
      overlay = *c.obj0_->pcd_;
      Cloud obj1;
      pcl::transformPointCloud(*c.obj1_->pcd_, obj1, transform);
      overlay += obj1;
      ostringstream oss;
      oss << getDebugPath() << "-object" << setw(4) << setfill('0') << j << "-gsiter" << i << ".pcd";
      pcl::io::savePCDFileBinary(oss.str(), overlay);
    }
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
  while(true) {
    cout << "ICP iter " << iter << endl;
    ++iter;
    
    pcl::TransformationFromCorrespondences tfc;
    for(size_t i = 0; i < objects1.size(); ++i) {
      const Cloud& obj1 = *objects1[i];
      const Cloud& obj0 = *objects0[i];
      double downsample = 0.1;
      for(size_t j = 0; j < obj1.size(); ++j) {
	if(((double)rand() / (double)RAND_MAX) > downsample)
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
    if(isAlmostIdentity(incremental_transform))
      break;

    // -- Move all the objects in sensor 1 by incremental_transform.
    for(size_t i = 0; i < objects1.size(); ++i)
      pcl::transformPointCloud(*objects1[i], *objects1[i], incremental_transform);
  }

  return overall_transform;
}

bool ObjectMatchingCalibrator::isAlmostIdentity(const Eigen::Affine3f& trans) const
{
  return (trans.matrix() - Matrix4f::Identity()).norm() < 0.001;
}


/************************************************************
 * CorrespondenceManager
 ************************************************************/

CorrespondenceManager::CorrespondenceManager(double dt_thresh, double centroid_thresh, double dist_thresh) :
  dt_thresh_(dt_thresh),
  centroid_thresh_(centroid_thresh),
  dist_thresh_(dist_thresh)
{
}

void CorrespondenceManager::addReferenceObject(rgbd::Cloud::ConstPtr pcd)
{
  objects0_.push_back(ReferenceObject::Ptr(new ReferenceObject(pcd)));
}

void CorrespondenceManager::addFloatingObject(rgbd::Cloud::Ptr pcd)
{
  objects1_.push_back(FloatingObject::Ptr(new FloatingObject(pcd)));
}      
    
double CorrespondenceManager::computeLoss(double downsample)
{
  computeCorrespondences();
  
  double loss = 0;
  for(size_t i = 0; i < correspondences_.size(); ++i) {
    double l = correspondences_[i].computeLoss(dist_thresh_, downsample);
    if(i == 170)
      cout << "loss for 170: " << l << endl;
    loss += l;
  }

  if(correspondences_.empty())
    return std::numeric_limits<double>::max();
  else
    return loss / (double)correspondences_.size();
}

void CorrespondenceManager::applyTimeOffset(double dt)
{
  for(size_t i = 0; i < objects1_.size(); ++i)
    objects1_[i]->timestamp_ = objects1_[i]->pcd_->header.stamp.toSec() + dt;
}

void CorrespondenceManager::applyTransform(const Eigen::Affine3f& transform)
{
  for(size_t i = 0; i < objects1_.size(); ++i) {
    size_t before = objects1_[i]->pcd_->size();
    pcl::transformPointCloud(*objects1_[i]->ref_pcd_, *objects1_[i]->pcd_, transform);
    ROS_ASSERT(before == objects1_[i]->pcd_->size());

    objects1_[i]->centroid_ = computeCentroid(*objects1_[i]->pcd_);
  }
}

void CorrespondenceManager::clear()
{
  objects0_.clear();
  objects1_.clear();
  correspondences_.clear();
}

void CorrespondenceManager::computeCorrespondences()
{
  //ScopedTimer st("CorrespondenceManager::computeCorrespondences");
  correspondences_.clear();
  correspondences_.reserve(objects1_.size());
  // TODO: O(n^2).  This could be faster.
  vector<bool> marked0(objects0_.size(), false);
  vector<double> distances;
  vector<double> dts;
  distances.resize(objects0_.size());
  dts.resize(objects0_.size());
  int num_found = 0;
  for(size_t i = 0; i < objects1_.size(); ++i) {
    for(size_t j = 0; j < objects0_.size(); ++j) {
      if(marked0[j]) { 
	dts[j] = numeric_limits<double>::max();
	distances[j] = numeric_limits<double>::max();
      }
      else { 
	dts[j] = fabs(objects1_[i]->timestamp_ - objects0_[j]->timestamp_);
	distances[j] = (objects1_[i]->centroid_ - objects0_[j]->centroid_).norm();
      }
    }

    int idx = -1;
    double best = numeric_limits<double>::max();
    ROS_ASSERT(!(best < best));
    for(size_t j = 0;  j < distances.size(); ++j) {
      if(dts[j] < dt_thresh_ && distances[j] < centroid_thresh_ && distances[j] < best) {
	best = distances[j];
	idx = j;
      }
    }
    if(idx > -1) { 
      marked0[idx] = true;
      correspondences_.push_back(Correspondence(objects0_[idx], objects1_[i]));
      ++num_found;
      // cout << "Added correspondence with dt " << dts[idx] << " and dist " << best << endl;
      // cout << "  " << objects1_[i]->timestamp_ << " " << objects0_[idx]->timestamp_ << endl;
    }
    else
      correspondences_.push_back(Correspondence(ReferenceObject::Ptr(), objects1_[i]));
  }
  cout << "Found " << num_found << " correspondences." << endl;
}


/************************************************************
 * Correspondence
 ************************************************************/

Correspondence::Correspondence(ReferenceObject::Ptr obj0, FloatingObject::Ptr obj1) :
  obj0_(obj0),
  obj1_(obj1)
{
  ROS_ASSERT(obj1_);
}

double Correspondence::computeLoss(double max_dist, double downsample) const
{
  double max_term_loss = max_dist + 0.1;
  const Cloud& pcd1 = *obj1_->pcd_;
  if(!obj0_) {
    return pcd1.size() * max_term_loss;
  }

  double loss = 0;
  vector<int> indices;
  vector<float> distances;
  double count = 0;
  for(size_t i = 0; i < pcd1.size(); ++i) {
    if(((double)rand() / (double)RAND_MAX) > downsample)
      continue;

    ++count;
    indices.clear();
    distances.clear();
    obj0_->tree_->nearestKSearch(pcd1[i], 1, indices, distances);
    if(indices.empty() || distances[0] > max_dist)
      loss += max_term_loss;
    else {
      const Point& pt0 = obj0_->pcd_->at(indices[0]);
      const Point& pt1 = pcd1[i];
      double dc = 0;
      double r0 = pt0.r / 255.0;
      double r1 = pt1.r / 255.0;
      double g0 = pt0.g / 255.0;
      double g1 = pt1.g / 255.0;
      double b0 = pt0.b / 255.0;
      double b1 = pt1.b / 255.0;
      dc = sqrt((r0-r1)*(r0-r1) + (g0-g1)*(g0-g1) + (b0-b1)*(b0-b1));
      loss += min(max_term_loss, distances[0] + dc / (10.0 * sqrt(3)));
    }
  }
  return loss / count;
}


/************************************************************
 * Object
 ************************************************************/

ReferenceObject::ReferenceObject(rgbd::Cloud::ConstPtr pcd) :
  pcd_(pcd),
  timestamp_(pcd_->header.stamp.toSec()),
  centroid_(computeCentroid(*pcd)),
  tree_(KdTree::Ptr(new KdTree))
{
  tree_->setInputCloud(pcd_);
}

FloatingObject::FloatingObject(rgbd::Cloud::Ptr pcd) :
  pcd_(pcd),
  timestamp_(pcd_->header.stamp.toSec()),
  centroid_(computeCentroid(*pcd_))
{
  ref_pcd_ = Cloud::Ptr(new Cloud);
  *ref_pcd_ = *pcd_;
}

Eigen::Vector3f computeCentroid(const Cloud& pcd)
{
  Vector3f centroid = Vector3f::Zero();
  double num = 0;
  for(size_t i = 0; i < pcd.size(); ++i) { 
    if(!isinf(pcd[i].x)) { 
      centroid += pcd[i].getVector3fMap();
      ++num;
    }
  }
  return centroid / num;
}


/************************************************************
 * LossFunction
 ************************************************************/

LossFunction::LossFunction(CorrespondenceManager* cm) :
  vis_("GridSearch"),
  cm_(cm)
{
}

double LossFunction::eval(const Eigen::VectorXd& x) const
{
  cm_->applyTimeOffset(x(0));

  // cout << " ------------------------------------------------------------" << endl;
  // cout << x.transpose() << endl;
  Affine3f transform = generateTransform(x(1), x(2), x(3), x(4), x(5), x(6));
  //cout << endl << transform.matrix() << endl;
  cm_->applyTransform(transform);

  double loss = cm_->computeLoss(0.2);
  //cout << " -- " << loss << endl;
    
  //for(size_t i = 0; i < cm_->correspondences_.size(); ++i) {
  for(size_t i = 170; i == 170; ++i) {
    if(!cm_->correspondences_[i].obj0_ || !cm_->correspondences_[i].obj1_)
      continue;

    cout << "Corr " << i << endl;
    Cloud::Ptr overlay(new Cloud);
    *overlay = *cm_->correspondences_[i].obj0_->pcd_;
    *overlay += *cm_->correspondences_[i].obj1_->pcd_;
    vis_.showCloud(overlay);
    cin.ignore();
  }
    
  return loss;
}
