#include <xpl_calibration/object_matching_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

void ObjectMatchingCalibrator::compute()
{
  const Sequence& seq0 = *pull<Sequence::ConstPtr>("Sequence0");
  const Sequence& seq1 = *pull<Sequence::ConstPtr>("Sequence1");
  const Objects& objects0 = *pull<const Objects*>("Objects0");
  const Objects& objects1 = *pull<const Objects*>("Objects1");
  ROS_ASSERT(seq0.size() == seq1.size());
  ROS_ASSERT(seq0.size() == objects0.size());
  ROS_ASSERT(seq0.size() == objects1.size());
  
  // -- Compute centroids for all objects.
  vector< vector<Vector3f> > centroids0;
  vector< vector<Vector3f> > centroids1;
  computeCentroids(objects0, &centroids0);
  computeCentroids(objects1, &centroids1);

  // -- RANSAC.
  int num_iterations = param<int>("NumRansacIters");
  int num_correspondences = param<int>("NumCorrespondences");
  Affine3f best_transform = Affine3f::Identity();
  int best_num_inliers = 0;
  for(int i = 0; i < num_iterations; ++i) {
    cout << "RANSAC iter " << i << endl;
    pcl::TransformationFromCorrespondences tfc;
    for(int j = 0; j < num_correspondences; ++j)
      sampleCorrespondence(seq0, seq1, centroids0, centroids1, &tfc);
    
    // -- Compute transform.
    Affine3f transform = tfc.getTransformation();

    // -- Count up inliers.
    Affine3f refined_transform;
    int num_inliers = countInliers(transform, centroids0, centroids1,
				   param<double>("Threshold"),
				   &refined_transform);
    
    
    // -- Save this transform if it's the best.
    if(num_inliers > best_num_inliers) {
      best_num_inliers = num_inliers;
      best_transform = refined_transform;
      cout << "Best transform has " << num_inliers << " inliers." << endl;
    }
  }
  rough_transform_ = best_transform;


  // -- Using the above as a starting point, now run ICP on all
  //    corresponding object clouds simultaneously.
  refined_transform_ = alignInlierModels(centroids0, centroids1);

  push<const Affine3f*>("RoughTransform", &rough_transform_);
  push<const Affine3f*>("RefinedTransform", &refined_transform_);
}

void ObjectMatchingCalibrator::debug() const
{

}

void ObjectMatchingCalibrator::computeCentroids(const Objects& objects,
						std::vector< std::vector<Eigen::Vector3f> >* centroids) const
{
  centroids->clear();
  centroids->resize(objects.size());
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
    }
  }
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
					   double thresh,
					   Eigen::Affine3f* refined_transform,
					   std::vector<Cloud::ConstPtr>* inliers0,
					   std::vector<Cloud::ConstPtr>* inliers1) const
{
  const Objects& objects0 = *pull<const Objects*>("Objects0");
  const Objects& objects1 = *pull<const Objects*>("Objects1");
  if(inliers0 && inliers1) { 
    inliers0->clear();
    inliers0->reserve(objects0.size());
    inliers1->clear();
    inliers1->reserve(objects1.size());
  }
  
  TransformationFromCorrespondences tfc;
  int num_inliers = 0;
  for(size_t i = 0; i < centroids1.size(); ++i) {
    if(centroids0.empty())
      continue;
    for(size_t j = 0; j < centroids1[i].size(); ++j) {
      Vector3f transformed = transform * centroids1[i][j];
      for(size_t k = 0; k < centroids0[i].size(); ++k) {
	if((centroids0[i][k] - transformed).norm() < thresh) { 
	  ++num_inliers;
	  tfc.add(centroids1[i][j], centroids0[i][k]);
	  if(inliers0 && inliers1) { 
	    inliers0->push_back(objects0[i][k]);
	    inliers1->push_back(objects1[i][j]);
	  }
	  break;
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
	       param<double>("Threshold"),
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
  double thresh = param<double>("Threshold");
  vector<int> indices;
  vector<float> distances;
  Affine3f overall_transform = rough_transform_;
  int iter = 0;
  while(true) {
    cout << "ICP iter " << iter << endl;
    ++iter;
    
    TransformationFromCorrespondences tfc;
    for(size_t i = 0; i < objects1.size(); ++i) {
      const Cloud& obj1 = *objects1[i];
      const Cloud& obj0 = *objects0[i];
      for(size_t j = 0; j < obj1.size(); ++j) {
	indices.clear();
	distances.clear();
	trees0[i]->nearestKSearch(obj1[j], 1, indices, distances);
	if(indices.empty() || distances[0] > thresh)
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
