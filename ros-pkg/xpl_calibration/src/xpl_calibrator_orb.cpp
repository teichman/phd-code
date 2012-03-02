#include <xpl_calibration/xpl_calibrator_orb.h>
#include <pcl/common/transformation_from_correspondences.h>

using namespace std;
using namespace Eigen;
using namespace pcl;


#define VISUALIZE (getenv("VISUALIZE") ? atoi(getenv("VISUALIZE")) : 0)

XplCalibratorOrb::XplCalibratorOrb()
{
}

pcl::PointXYZRGB XplCalibratorOrb::getPoint(const cv::KeyPoint& keypoint, const RGBDCloud& pcd) const
{
  int y = keypoint.pt.y;
  int x = keypoint.pt.x;
  size_t idx = y * pcd.width + x;
  return pcd[idx];
}

pcl::PointXYZRGB XplCalibratorOrb::samplePoint(const std::vector<cv::KeyPoint>& keypoints,
					 const RGBDCloud& pcd,
					 int* idx = NULL) const
{
  for(int i = 0; i < 1000; ++i) { 
    size_t kpidx = rand() % keypoints.size();
    pcl::PointXYZRGB pt = getPoint(keypoints[kpidx], pcd);
    if(!isnan(pt.x)) {
      if(idx) *idx = kpidx;
      return pt;
    }
  }
  
  pcl::PointXYZRGB bad;
  bad.x = -1;
  bad.y = -1;
  bad.z = -1;
  if(idx) *idx = -1;
  return bad;
}

Eigen::Affine3f XplCalibratorOrb::calibrate(RGBDSequence::ConstPtr refseq,
					    RGBDSequence::ConstPtr tarseq) const
{
  RGBDCloud::Ptr ref = refseq->pcds_[0];
  RGBDCloud::Ptr tar = tarseq->pcds_[0];

  // -- Compute orb features.
  HighResTimer hrt("Orb");
  cv::Mat3b ref_img = refseq->imgs_[0];
  cv::Mat3b tar_img = tarseq->imgs_[0];
  OrbExtractor oe;
  vector<cv::KeyPoint> ref_keypoints;
  OrbExtractor::PackedDescriptorsPtr ref_descr = oe.extractOrb(ref_img, &ref_keypoints);
  vector<cv::KeyPoint> tar_keypoints;
  OrbExtractor::PackedDescriptorsPtr tar_descr = oe.extractOrb(tar_img, &tar_keypoints);
  if(VISUALIZE) {
    cv::Mat3b ref_vis = ref_img.clone();
    for(size_t i = 0; i < ref_keypoints.size(); ++i)
      cv::circle(ref_vis, ref_keypoints[i].pt, 2, cv::Scalar(0, 0, 255), -1);
    cout << "Got " << ref_keypoints.size() << " orb features." << endl;
    cv::imshow("orb ref", ref_vis);

    cv::Mat3b tar_vis = tar_img.clone();
    for(size_t i = 0; i < tar_keypoints.size(); ++i)
      cv::circle(tar_vis, tar_keypoints[i].pt, 2, cv::Scalar(0, 0, 255), -1);
    cout << "Got " << tar_keypoints.size() << " orb features." << endl;
    cv::imshow("orb tar", tar_vis);

    cv::waitKey();
  }

  // -- Find matches between the two images.
  hrt.reset("LSH");
  hrt.start();
  //LSHDescriptorDatabase dd(tar_descr);
  NaiveDescriptorDatabase dd(tar_descr);
  vector< vector<int> > matches(ref_keypoints.size());
  vector<int> valid; // keypoints in ref that have matches & 3D points.
  ROS_ASSERT((int)ref_keypoints.size() == ref_descr->cols());
  for(size_t i = 0; i < matches.size(); ++i) {
    matches[i].reserve(tar_descr->cols());
    dd.query(ref_descr->col(i), &matches[i], 75);
    if(!matches[i].empty() && !isnan(getPoint(ref_keypoints[i], *ref).x))
      valid.push_back(i);
  }
  hrt.stop();
  cout << hrt.report() << endl;
  cout << "Found " << valid.size() << " points with at least one match." << endl;

  if(VISUALIZE) {
    cv::Size sz(tar_img.cols * 2, tar_img.rows);
    cv::Mat3b vis(sz);
    for(int y = 0; y < vis.rows; ++y) {
      for(int x = 0; x < vis.cols; ++x) {
	if(x < ref_img.cols)
	  vis(y, x) = ref_img(y, x);
	else
	  vis(y, x) = tar_img(y, x - ref_img.cols);
      }
    }

    for(size_t i = 0; i < matches.size(); ++i) {
      for(size_t j = 0; j < matches[i].size(); ++j) {
	cv::Point2f offset(ref_img.cols, 0);
	cv::line(vis, tar_keypoints[matches[i][j]].pt + offset, ref_keypoints[i].pt, cv::Scalar(255, 0, 0));
      }
    }
	  
    cv::imshow("matches", vis);
    cv::waitKey();
  }
  
  // -- Compute kdtrees and normals.
  hrt.reset("KdTrees");
  hrt.start();
  search::KdTree<pcl::PointXYZRGB>::Ptr ref_tree(new search::KdTree<pcl::PointXYZRGB>);
  ref_tree->setInputCloud(ref);
  search::KdTree<pcl::PointXYZRGB>::Ptr tar_tree(new search::KdTree<pcl::PointXYZRGB>);
  tar_tree->setInputCloud(tar);
  hrt.stop();
  cout << hrt.report() << endl;

  hrt.reset("Normals");
  hrt.start();
  
  NormalEstimation<pcl::PointXYZRGB, Normal> ref_ne;
  ref_ne.setSearchMethod(ref_tree);
  ref_ne.setInputCloud(ref);
  //ref_ne.setRadiusSearch(0.3);
  ref_ne.setKSearch(15);
  PointCloud<Normal>::Ptr ref_normals(new PointCloud<Normal>);
  ref_ne.compute(*ref_normals);
  
  NormalEstimation<pcl::PointXYZRGB, Normal> tar_ne;
  tar_ne.setSearchMethod(tar_tree);
  tar_ne.setInputCloud(tar);
  //tar_ne.setRadiusSearch(0.3);
  tar_ne.setKSearch(15);
  PointCloud<Normal>::Ptr tar_normals(new PointCloud<Normal>);
  tar_ne.compute(*tar_normals);

  hrt.stop();
  cout << hrt.report() << endl;
  
  TransformValidator tv;
  double thresh = 0.04;
  for(int i = 0; i < 10000; ++i) {
    //cout << "Sample " << i << endl;
    
    // -- Sample 3 orb points in ref img.
    int idx0 = valid[rand() % valid.size()];
    int idx1 = valid[rand() % valid.size()];
    int idx2 = valid[rand() % valid.size()];
    
    pcl::PointXYZRGB r0 = getPoint(ref_keypoints[idx0], *ref);
    pcl::PointXYZRGB r1 = getPoint(ref_keypoints[idx1], *ref);
    pcl::PointXYZRGB r2 = getPoint(ref_keypoints[idx2], *ref);
    double d01 = pcl::euclideanDistance(r0, r1);
    double d02 = pcl::euclideanDistance(r0, r2);
    double d12 = pcl::euclideanDistance(r1, r2);
    vector<int>& matches0 = matches[idx0];
    vector<int>& matches1 = matches[idx1];
    vector<int>& matches2 = matches[idx2];
    
    for(size_t j = 0; j < matches0.size(); ++j) {
      pcl::PointXYZRGB t0 = getPoint(tar_keypoints[matches0[j]], *tar);
      if(isnan(t0.x))
	continue;
      
      for(size_t k = 0; k < matches1.size(); ++k) {
	pcl::PointXYZRGB t1 = getPoint(tar_keypoints[matches1[k]], *tar);
	if(isnan(t1.x))
	  continue;
	if(fabs(pcl::euclideanDistance(t0, t1) - d01) > thresh)
	  continue;


	for(size_t l = 0; l < matches2.size(); ++l) {
	  pcl::PointXYZRGB t2 = getPoint(tar_keypoints[matches2[l]], *tar);
	  if(isnan(t2.x))
	    continue;
	  if(fabs(pcl::euclideanDistance(t0, t2) - d02) > thresh)
	    continue;
	  if(fabs(pcl::euclideanDistance(t1, t2) - d12) > thresh)
	    continue;

	  TransformationFromCorrespondences tfc;
	  tfc.add(t0.getVector3fMap(), r0.getVector3fMap());
	  tfc.add(t1.getVector3fMap(), r1.getVector3fMap());
	  tfc.add(t2.getVector3fMap(), r2.getVector3fMap());

	  Affine3f transform = tfc.getTransformation();
	  if((transform * t0.getVector3fMap() - r0.getVector3fMap()).norm() > 0.1 ||
	     (transform * t1.getVector3fMap() - r1.getVector3fMap()).norm() > 0.1 ||
	     (transform * t2.getVector3fMap() - r2.getVector3fMap()).norm() > 0.1)
	    continue;

	  tv.candidates_.push_back(transform);
	  //cout << "Added candidate transform.  Total: " << tv.candidates_.size() << endl;
	}
      }
    }
  }

  cout << "Total candidate transforms: " << tv.candidates_.size() << endl;
  tv.ref_pcd_ = ref;
  tv.tar_pcd_ = tar;
  tv.ref_normals_ = ref_normals;
  tv.tar_normals_ = tar_normals;
  tv.ref_tree_ = ref_tree;
  tv.tar_tree_ = tar_tree;

  return tv.compute();
}

