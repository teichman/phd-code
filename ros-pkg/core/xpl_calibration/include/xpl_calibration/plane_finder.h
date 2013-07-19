#ifndef ORGANIZED_CONNECTED_COMPONENTS_H
#define ORGANIZED_CONNECTED_COMPONENTS_H

#include <queue>
#include <pcl/common/distances.h>
#include <bag_of_tricks/high_res_timer.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <bag_of_tricks/image_region_iterator.h>

class OrganizedConnectedComponents
{
public:
  enum {
    PROCESSING = -1,
    UNTOUCHED = -2,
    NONE = -3,
    NODATA = -4
  };
    
  //! 0, 1, 2, ... are assignments of pixel i to that component.
  //! Negative numbers are special codes.  See the enum.
  std::vector<int> assignments_;
  std::vector< std::vector<int> > indices_;
  //! Number of inliers in each plane.
  //std::vector<int> num_inliers_;
  
  OrganizedConnectedComponents(size_t min_inliers,
                               double distance_thresh);
  void compute(const rgbd::Cloud& cloud);
           
protected:
  size_t min_inliers_;
  double distance_thresh_;

  bool findComponent(const rgbd::Cloud& pcd,
                     size_t center_idx,
                     int new_id);
  
  void getNeighbors(const rgbd::Cloud& pcd,
                    size_t center_idx,
                    std::vector<int>* indices);
};

#endif // ORGANIZED_CONNECTED_COMPONENTS_H
