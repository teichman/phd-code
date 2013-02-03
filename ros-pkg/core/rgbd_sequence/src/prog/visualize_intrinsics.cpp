#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/primesense_model.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::visualization;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: visualize_intrinsics SEQ" << endl;
  oss << "  SEQ is a particular test sequence." << endl;
  return oss.str();
}

unsigned char g_key;
char waitKey()
{
  g_key = 0;
  while(true) {
    usleep(1e3);
    if(g_key != 0)
      break;
  }
  return g_key;
}

void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(event.keyDown())
    g_key = event.getKeyCode();
}

VectorXf fitPlane(Cloud::ConstPtr pcd, double tol)
{
  SampleConsensusModelPlane<Point>::Ptr plane(new SampleConsensusModelPlane<Point>(pcd));
  RandomSampleConsensus<Point> ransac(plane);
  ransac.setDistanceThreshold(tol);
  ransac.computeModel();
  std::vector<int> inliers;
  ransac.getInliers(inliers);
  VectorXf raw_coefs;
  ransac.getModelCoefficients(raw_coefs);
  VectorXf coefs;
  plane->optimizeModelCoefficients(inliers, raw_coefs, coefs);

  // Planes should not be aligned with the camera focal plane.
  if(coefs(2) > 0.9) {
    Cloud::Ptr chopped(new Cloud);
    for(size_t i = 0; i < pcd->size(); ++i)
      if(fabs(coefs.dot(pcd->at(i).getVector4fMap()) + coefs(3)) > tol)
        chopped->push_back(pcd->at(i));
    return fitPlane(chopped, tol);
  }
    
  return coefs;
}
 
int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }

  // -- Show original cloud.
  StreamSequence sseq;
  sseq.load(argv[1]);
  Frame frame;
  sseq.getFrame(1, &frame);
  // Start with the model saved during recording, but allow it to be changed later.
  PrimeSenseModel model = sseq.model_;
  
  
  Cloud pcd;
  sseq.model_.project(frame, &pcd);
  CloudViewer vis("Cloud");
  vis.registerKeyboardCallback(keyboardCallback);
  
  int f_increment = 10;
  int c_increment = 10;
  bool done = false;
  while(!done) {
    cout << "PrimeSenseModel: " << sseq.model_.status("  ") << endl;
    sseq.frameToCloud(frame, pcd.get());
    vis.showCloud(pcd);

    switch(waitKey()) {
    case 'q':
      exit(0);
      break;
    case 'd':
      done = true;
      break;
    case ',':
      proj.fx_ -= f_increment;
      proj.fy_ -= f_increment;
      break;
    case '.':
      proj.fx_ += f_increment;
      proj.fy_ += f_increment;
      break;
    case '<':
      proj.cx_ -= c_increment;
      //proj.cy_ -= c_increment;
      break;
    case '>':
      proj.cx_ += c_increment;
      //proj.cy_ += c_increment;
      break;
    default:
      break;
    }
  }

  // -- Apply a z threshold.
  Cloud::Ptr zthresh(new Cloud);
  zthresh->reserve(pcd->size());
  for(size_t i = 0; i < pcd->size(); ++i) {
    if(!isFinite(pcd->at(i)))
      continue;
    if(pcd->at(i).z > 1.0)
      continue;
    zthresh->push_back(pcd->at(i));
  }
  vis.showCloud(zthresh);
  waitKey();

  // -- Get the 2 biggest connected components.
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(zthresh);
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(1000);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(zthresh);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  cout << "Found " << cluster_indices.size() << " clusters." << endl;
  ROS_ASSERT(cluster_indices.size() == 2);
  
  Cloud::Ptr cluster0(new Cloud);
  for(size_t i = 0; i < cluster_indices[0].indices.size(); ++i)
    cluster0->push_back(zthresh->at(cluster_indices[0].indices[i]));
  Cloud::Ptr cluster1(new Cloud);
  for(size_t i = 0; i < cluster_indices[1].indices.size(); ++i)
    cluster1->push_back(zthresh->at(cluster_indices[1].indices[i]));

  Cloud::Ptr clusters(new Cloud);
  *clusters = *cluster0;
  *clusters += *cluster1;
  vis.showCloud(clusters);
  waitKey();
    
  // -- Fit planes to each cluster.
  double tol = 0.005;
  VectorXf coefs0 = fitPlane(cluster0, tol);
  VectorXf coefs1 = fitPlane(cluster1, tol);
  cout << coefs0.transpose() << endl;
  cout << coefs1.transpose() << endl;

  for(size_t i = 0; i < cluster0->size(); ++i) { 
    if(fabs(coefs0.dot(cluster0->at(i).getVector4fMap()) + coefs0(3)) < tol) {
      cluster0->at(i).r = 255;
      cluster0->at(i).g = 0;
      cluster0->at(i).b = 0;
    }
  }
  for(size_t i = 0; i < cluster1->size(); ++i) { 
    if(fabs(coefs1.dot(cluster1->at(i).getVector4fMap()) + coefs1(3)) < tol) {
      cluster1->at(i).r = 0;
      cluster1->at(i).g = 0;
      cluster1->at(i).b = 255;
    }
  }
  *clusters = *cluster0;
  *clusters += *cluster1;
  vis.showCloud(clusters);
  waitKey();
  
  // -- Measure the distance between the two planes.
  //    Do this by getting the average distance from inliers in one plane to the other plane.
  double dist = 0;
  double num = 0;
  for(size_t i = 0; i < cluster0->size(); ++i) { 
    if(fabs(coefs0.dot(cluster0->at(i).getVector4fMap()) + coefs0(3)) < tol) { 
      dist += fabs(coefs1.dot(cluster0->at(i).getVector4fMap()) + coefs1(3));
      ++num;
    }
  }
  for(size_t i = 0; i < cluster1->size(); ++i) { 
    if(fabs(coefs1.dot(cluster1->at(i).getVector4fMap()) + coefs1(3)) < tol) { 
      dist += fabs(coefs0.dot(cluster1->at(i).getVector4fMap()) + coefs0(3));
      ++num;
    }
  }
  // 21 inches = 0.533 meters.
  cout << "Distance between planes: " << dist / num << " meters." << endl;
  cout << "Factor: " << dist / (num * 0.533) << endl;
  
  return 0;
}
