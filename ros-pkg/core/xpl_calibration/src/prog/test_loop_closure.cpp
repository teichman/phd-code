/*
 * =====================================================================================
 *
 *       Filename:  test_loop_closure.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  09/05/2012 04:35:17 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <xpl_calibration/loop_closer.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <Eigen/Eigen>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <cloud_basics/vis_wrapper.h>

using namespace std;
using namespace rgbd;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data")
    ;

  p.add("sseq", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Using " << opts["sseq"].as<string>() << endl;
  StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory (opts["sseq"].as<string> ());  
  LoopCloser lc(sseq);
  lc.fine_tune_ = false;
  lc.visualize_ = true;
  lc.max_feature_dist_ = 500;
  lc.keypoints_per_frame_ = 250;
  lc.min_pairwise_keypoint_dist_ = 0.04; //cm apart
  lc.min_ransac_inliers_ = 10; //Need at least this many inliers to be considered a valid
  lc.min_ransac_inlier_percent_ = 0.1;
  lc.ransac_max_inlier_dist_ = 0.02; //Need to be within 2 cm to be considered an inlier
  lc.num_ransac_samples_ = 1000;
  lc.icp_max_avg_dist_ = 0.03; //Avg pt-to-pt distance required
  lc.icp_max_inlier_dist_ = 0.1; // Highest distance allowed to be considered an inlier
  lc.icp_inlier_percent_ = 0.3; // At least this percentage of points must be inliers
  lc.ftype_ = ORB;
  lc.k_ = 2;
  lc.min_time_offset_ = 30;
  lc.verification_type_ = MDE;
  lc.max_mde_ = 0.1; //Without color was 0.02
  lc.harris_thresh_ = 0.01;
  lc.use_3d_sift_ = true;
  lc.fpfh_radius_ = 0.02;
  lc.harris_margin_ = 50;
  lc.step_ = 2;
  size_t step = 15;
  cloud_basics::VisWrapper vis;
  for(size_t i = 0; i < sseq->size(); i+= step)
  {
    Frame frame;
    sseq->readFrame(i, &frame);
    vector<size_t> targets;
    vector<Eigen::Affine3f> transforms;
    if( lc.getLinkHypotheses(frame, i, targets, transforms ) )
    {
      cout << "Found " << targets.size() << " candidate targets" << endl;
      Cloud::Ptr cloud_cur(new Cloud); sseq->model_.frameToCloud(frame, cloud_cur.get());
      cout << "On time " << i << endl;
      for(size_t j = 0; j < targets.size(); j++)
      {
        cout << targets[j] << endl;
        Frame frame_prev; sseq->readFrame(targets[j], &frame_prev);
        Cloud::Ptr cloud_prev(new Cloud); sseq->model_.frameToCloud(frame_prev, cloud_prev.get());
        vis.holdOn();
        vis.showCloud(cloud_prev);
        Cloud::Ptr cloud_trans(new Cloud); pcl::transformPointCloud(*cloud_cur, *cloud_trans, transforms[j]);
        string cur_handle = vis.showCloud(cloud_cur);
        vis.waitKey();
        vis.updateCloud(cur_handle, cloud_trans);
        vis.waitKey();
        vis.clearAll();
        
      }
      cout << endl;
    }
    cv::imshow("Current frame", frame.img_);
    cv::waitKey(30);
  }
  return 0;
}
