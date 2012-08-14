/*
 * =====================================================================================
 *
 *       Filename:  view_reconstruction.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  08/13/2012 11:25:36 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <slam_interface/types.h>
#include <slam_interface/rgbdslam_interface.h>
#include <rgbd_sequence/stream_sequence.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
  

int usage(char** argv)
{
  cout << "USEAGE: " << argv[0] << " seq_dir traj_file" << endl;
  cout << "Loads slam trajectory file and transforms pointclouds in sequence." << endl;
  return 1;
}

using namespace slam_interface;
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud_t;

int main(int argc, char** argv)
{
  if(argc < 3)
    return usage(argv);
  string seq_dir = argv[1];
  string traj_file = argv[2];
  //Load slam info
  RgbdSlamInterface::Ptr rsi(new RgbdSlamInterface(traj_file));
  // Load sequence
  rgbd::StreamSequence::Ptr seq(new rgbd::StreamSequence);
  seq->load(seq_dir);
  cout << "Seq has " << seq->size() << " frames" << endl;
  cout << "Interf has " << rsi->size() << " frames" << endl;
  // Make sure they're the same size
  ROS_ASSERT(seq->size() == rsi->size());
  // Visualize all in turn
  pcl::visualization::CloudViewer cloud_viewer("cloud");
  cv::namedWindow("image");
  Cloud_t::Ptr cloud_total(new Cloud_t);
  //Need to filter to keep from freezing
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setLeafSize(0.02, 0.02, 0.02);

  for(int i = 0; i < seq->size(); i++)
  {
    cout << "On cloud " << i << endl;
    Cloud_t::Ptr cloud = seq->getCloud(i);
    //Transform cloud to canonical frame
    Eigen::Affine3f trans = rsi->getTransform(i);
    Cloud_t::Ptr transformed(new Cloud_t);
    pcl::transformPointCloud(*cloud, *transformed, trans);
    *cloud_total += *transformed;
    vg.setInputCloud(cloud_total);
    Cloud_t::Ptr cloud_filtered(new Cloud_t);
    vg.filter(*cloud_filtered);
    *cloud_total = *cloud_filtered;
    //View as if the camera were static
    Cloud_t::Ptr transformed_total(new Cloud_t);
    pcl::transformPointCloud(*cloud_total, *transformed_total, trans.inverse());
    cloud_viewer.showCloud(transformed_total);
    cv::imshow("image", seq->getImage(i));
    cv::waitKey();
  }
}


