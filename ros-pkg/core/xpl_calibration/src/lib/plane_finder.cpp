#include <xpl_calibration/plane_finder.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

#define SRAND (getenv("SRAND") ? atoi(getenv("SRAND")) : 0)
#define VISUALIZE (getenv("VISUALIZE") ? atoi(getenv("VISUALIZE")) : 0)

PlaneFinder::PlaneFinder(size_t min_inliers,
                         double angle_thresh,
                         double distance_thresh) :
  min_inliers_(min_inliers),
  angle_thresh_(angle_thresh),
  distance_thresh_(distance_thresh)
{
}

void PlaneFinder::compute(const Cloud& pcd,
                          const pcl::PointCloud<pcl::Normal>& normals)
{
  ScopedTimer st("PlaneFinder::compute");
  ROS_ASSERT(UNTOUCHED == -2); // enum isn't guaranteed to work with negative numbers.
  normals_.clear();
  colors_.clear();
  img_centroids_.clear();
  assignments_.clear();
  assignments_.resize(pcd.size(), UNTOUCHED);
  
  // -- Find planes.
  vector<bool> marked(pcd.size(), false);
  IndicesPtr unused(new vector<int>(pcd.size()));
  int num_planes = 0;
  for(size_t i = 0; i < pcd.size(); ++i) {
    ROS_ASSERT(assignments_[i] != PROCESSING);
    if(isnan(pcd[i].x))
      assignments_[i] = NODATA;
    else if(assignments_[i] == UNTOUCHED) { 
      bool found = findPlanarSurface(pcd, normals, i, num_planes);
      if(found)
        ++num_planes;
    }
  }

  // -- Visualize the planes.
  if(VISUALIZE) {
    cv::Mat1b zbuf(cv::Size(pcd.width, pcd.height), 0);
    for(int y = 0; y < zbuf.rows; ++y) {
      for(int x = 0; x < zbuf.cols; ++x) {
        int idx = y * zbuf.cols + x;
        if(!isnan(pcd[idx].z))
          zbuf(y, x) = 255.0 * pcd[idx].z / 7.0;
      }
    }
    cv::imshow("depth", zbuf);
    cv::waitKey(10);
        
    srand(SRAND);
    ColorWheel cw(num_planes);
    cv::Mat3b vis(cv::Size(pcd.width, pcd.height), cv::Vec3b(0, 0, 0));
    for(int y = 0; y < vis.rows; ++y) {
      for(int x = 0; x < vis.cols; ++x) {
        int idx = y * vis.cols + x;
        if(assignments_[idx] >= 0)
          vis(y, x) = cw.getColor(assignments_[idx]);
        else if(assignments_[idx] == NONE)
          vis(y, x) = cv::Vec3b(0, 0, 0);
        else if(assignments_[idx] == NODATA)
          vis(y, x) = cv::Vec3b(0, 0, 0);
      }
    }
    cv::imshow("planes", vis);
    cv::waitKey();
  }

  // -- Get the image centroids.
  for(int i = 0; i < num_planes; ++i) { 
    double cy = 0;
    double cx = 0;
    double num = 0;
    for(size_t j = 0; j < pcd.size(); ++j) {
      if(assignments_[j] == i) {
        int y = j / pcd.width;
        int x = j - y * pcd.width;
        cy += y;
        cx += x;
        ++num;
      }
    }
    cy /= num;
    cx /= num;
    img_centroids_.push_back(cv::Point2i(cx, cy));
  }
}

bool PlaneFinder::findPlanarSurface(const Cloud& pcd,
                                    const PointCloud<Normal>& normals,
                                    size_t center_idx,
                                    int new_plane_id)
{
  queue<size_t> que;
  que.push(center_idx);
  vector<size_t> inliers;
  inliers.push_back(center_idx);
  vector<int> indices;
  Vector3f acc_normal = normals[center_idx].getNormalVector3fMap();
  Vector3f running_normal = acc_normal;
  double acc_constant = running_normal.dot(pcd[center_idx].getVector3fMap());
  double running_constant = acc_constant;
  
  while(!que.empty()) {
    size_t idx = que.front();
    que.pop();
    indices.clear();
    getInPlaneNeighbors(pcd, normals, idx, running_normal, running_constant, &indices);
    //getInPlaneNeighbors(pcd, normals, idx, normals[center_idx].getNormalVector3fMap(), &indices);

    for(size_t i = 0; i < indices.size(); ++i) {
      if(assignments_[indices[i]] == UNTOUCHED) { 
        inliers.push_back(indices[i]);
        que.push(indices[i]);
        assignments_[indices[i]] = PROCESSING;
        acc_normal += normals[indices[i]].getNormalVector3fMap();
        acc_constant += running_normal.dot(pcd[indices[i]].getVector3fMap());
        running_normal = acc_normal / (double)inliers.size();
        running_normal.normalize();
        running_constant = acc_constant / (double)inliers.size();
      }
    }
  }

  if(inliers.size() < min_inliers_) { 
    for(size_t i = 0; i < inliers.size(); ++i)
      assignments_[inliers[i]] = NONE;
    return false;
  }
  else {
    normals_.push_back(running_normal);
    for(size_t i = 0; i < inliers.size(); ++i)
      assignments_[inliers[i]] = new_plane_id;
    return true;
  }
}

void PlaneFinder::getInPlaneNeighbors(const Cloud& pcd,
                                      const PointCloud<Normal>& normals,
                                      size_t center_idx,
                                      const Vector3f& plane_normal,
                                      double plane_constant,
                                      vector<int>* indices)
{

  int center_y = center_idx / pcd.width;
  int center_x = center_idx - center_y * pcd.width;
  ImageRegionIterator iri(cv::Size(pcd.width, pcd.height), 2);
  for(iri.setCenter(cv::Point2i(center_x, center_y)); !iri.done(); ++iri) {
    if(iri.index() == (int)center_idx)
      continue;
    
    Vector3f n1 = normals[iri.index()].getNormalVector3fMap();
    double ptpdist = fabs(plane_normal.dot(pcd[iri.index()].getVector3fMap()) - plane_constant);
    if(acos(fabs(n1.dot(plane_normal))) < angle_thresh_ && ptpdist < distance_thresh_)
      indices->push_back(iri.index());
  }
}
    
