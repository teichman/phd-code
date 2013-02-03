#include <xpl_calibration/orb_matcher.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

void OrbMatcher::compute()
{
  transforms_.clear();
  matches_.clear();

  PackedDescriptorsConstPtr descr0;
  PackedDescriptorsConstPtr descr1;
  pull("Descriptors0", &descr0);
  pull("Descriptors1", &descr1);
  const vector<cv::KeyPoint>& keypoints0 = *pull<KeyPointsConstPtr>("Keypoints0");
  const vector<cv::KeyPoint>& keypoints1 = *pull<KeyPointsConstPtr>("Keypoints1");
  Cloud::ConstPtr cloud0 = pull<Cloud::ConstPtr>("Cloud0");
  Cloud::ConstPtr cloud1 = pull<Cloud::ConstPtr>("Cloud1");

  // -- Compute Orb matches.
  matches_.resize(keypoints0.size());
  NaiveDescriptorDatabase dd(descr1);
  vector<int> valid; // keypoints in frame 0 that have matches and 3D points.
  ROS_ASSERT((int)keypoints0.size() == descr0->cols());
  for(size_t i = 0; i < matches_.size(); ++i) {
    matches_[i].reserve(descr1->cols());
    dd.query(descr0->col(i), &matches_[i], 75);
    if(!matches_[i].empty() && !isnan(getPoint(keypoints0[i], *cloud0).x))
      valid.push_back(i);
  }

  for(int i = 0; i < param<int>("NumSamples"); ++i) {
     // -- Sample 3 orb points in ref img and precompute their distances to each other.
    int idx0 = valid[rand() % valid.size()];
    int idx1 = valid[rand() % valid.size()];
    int idx2 = valid[rand() % valid.size()];
    pcl::PointXYZRGB r0 = getPoint(keypoints0[idx0], *cloud0);
    pcl::PointXYZRGB r1 = getPoint(keypoints0[idx1], *cloud0);
    pcl::PointXYZRGB r2 = getPoint(keypoints0[idx2], *cloud0);
    double d01 = pcl::euclideanDistance(r0, r1);
    double d02 = pcl::euclideanDistance(r0, r2);
    double d12 = pcl::euclideanDistance(r1, r2);

    // -- Sample sets of 3 correspondences using the orb matches.
    vector<int>& matches0 = matches_[idx0];
    vector<int>& matches1 = matches_[idx1];
    vector<int>& matches2 = matches_[idx2];
    for(size_t j = 0; j < matches0.size(); ++j) {
      pcl::PointXYZRGB t0 = getPoint(keypoints1[matches0[j]], *cloud1);
      if(isnan(t0.x))
        continue;
      
      for(size_t k = 0; k < matches1.size(); ++k) {
        pcl::PointXYZRGB t1 = getPoint(keypoints1[matches1[k]], *cloud1);
        if(isnan(t1.x))
          continue;
        if(fabs(pcl::euclideanDistance(t0, t1) - d01) > param<double>("DistanceThresh"))
          continue;


        for(size_t l = 0; l < matches2.size(); ++l) {
          pcl::PointXYZRGB t2 = getPoint(keypoints1[matches2[l]], *cloud1);
          if(isnan(t2.x))
            continue;
          if(fabs(pcl::euclideanDistance(t0, t2) - d02) > param<double>("DistanceThresh"))
            continue;
          if(fabs(pcl::euclideanDistance(t1, t2) - d12) > param<double>("DistanceThresh"))
            continue;

          pcl::TransformationFromCorrespondences tfc;
          tfc.add(t0.getVector3fMap(), r0.getVector3fMap());
          tfc.add(t1.getVector3fMap(), r1.getVector3fMap());
          tfc.add(t2.getVector3fMap(), r2.getVector3fMap());

          Affine3f transform = tfc.getTransformation();
          if((transform * t0.getVector3fMap() - r0.getVector3fMap()).norm() > 0.1 ||
             (transform * t1.getVector3fMap() - r1.getVector3fMap()).norm() > 0.1 ||
             (transform * t2.getVector3fMap() - r2.getVector3fMap()).norm() > 0.1)
            continue;

          transforms_.push_back(transform);
        }
      }
    }
  }

  // -- Now accept only those transforms that have a sufficient number of inliers
  //    among the other orb keypoints.
  keypoint_cloud0_->clear();
  for(size_t i = 0; i < keypoints0.size(); ++i)
    keypoint_cloud0_->push_back(getPoint(keypoints0[i], *cloud0));
  keypoint_cloud1_->clear();
  for(size_t i = 0; i < keypoints1.size(); ++i)
    keypoint_cloud1_->push_back(getPoint(keypoints1[i], *cloud1));

  pcl::KdTreeFLANN<Point> kdtree0;
  kdtree0.setInputCloud(keypoint_cloud0_);
  
  pruned_.clear();
  vector<int> indices;
  vector<float> distances;
  for(size_t i = 0; i < transforms_.size(); ++i) {
    transformed_keypoint_cloud1_.clear();
    transformPointCloud(*keypoint_cloud1_, transformed_keypoint_cloud1_, transforms_[i]);

    int num_inliers = 0;
    for(size_t j = 0; j < transformed_keypoint_cloud1_.size(); ++j) {
      indices.clear();
      distances.clear();
      kdtree0.nearestKSearch(transformed_keypoint_cloud1_[j], 1, indices, distances);
      if(distances.size() != 0 && distances[0] < param<double>("DistanceThresh"))
        ++num_inliers;
    }

    //cout << "min num inliers: " << (int)(param<double>("MinInlierPercent") * valid.size()) << endl;
    if(num_inliers > (int)(param<double>("MinInlierPercent") * valid.size()))
      pruned_.push_back(transforms_[i]);
  }
  
  push<TransformsConstPtr>("Transforms", &pruned_);
}

pcl::PointXYZRGB OrbMatcher::getPoint(const cv::KeyPoint& keypoint, const Cloud& pcd) const
{
  int y = keypoint.pt.y;
  int x = keypoint.pt.x;
  size_t idx = y * pcd.width + x;
  return pcd[idx];
}

void OrbMatcher::debug() const
{
  cout << *this << endl;
  cout << "Found " << transforms_.size() << " candidate transforms." << endl;
  cout << "Found " << pruned_.size() << " candidate transforms after pruning." << endl;

  cv::Mat3b img0 = pull<cv::Mat3b>("Image0");
  cv::Mat3b img1 = pull<cv::Mat3b>("Image1");
  
  cv::Size sz(img1.cols * 2, img1.rows);
  cv::Mat3b vis(sz);
  for(int y = 0; y < vis.rows; ++y) {
    for(int x = 0; x < vis.cols; ++x) {
      if(x < img0.cols)
        vis(y, x) = img0(y, x);
      else
        vis(y, x) = img1(y, x - img0.cols);
    }
  }

  const vector<cv::KeyPoint>& keypoints0 = *pull<KeyPointsConstPtr>("Keypoints0");
  const vector<cv::KeyPoint>& keypoints1 = *pull<KeyPointsConstPtr>("Keypoints1");

  for(size_t i = 0; i < matches_.size(); ++i) {
    for(size_t j = 0; j < matches_[i].size(); ++j) {
      cv::Point2f offset(img0.cols, 0);
      cv::line(vis, keypoints1[matches_[i][j]].pt + offset, keypoints0[i].pt, cv::Scalar(255, 0, 0));
    }
  }
          
  cv::imwrite(debugBasePath() + "-matches.png", vis);
}
