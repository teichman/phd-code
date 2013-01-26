#include <boost/program_options.hpp>
#include <rgbd_sequence/primesense_model.h>
#include <xpl_calibration/slam_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data")
    ("traj", bpo::value<string>()->required(), "Trajectory")
    ;

  p.add("sseq", 1).add("traj", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] SSEQ TRAJ" << endl;  
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Load data.
  StreamSequence sseq;
  sseq.load(opts["sseq"].as<string>());
  Trajectory traj;
  traj.load(opts["traj"].as<string>());

  // -- Build map.
  Cloud::Ptr map = SlamCalibrator::buildMap(sseq, traj, MAX_RANGE_MAP, 0.01);

  // -- Visualize.
  for(size_t i = 0; i < traj.size(); ++i) {
    if(!traj.exists(i))
      continue;
    Affine3f transform = traj.get(i).inverse().cast<float>();
    Frame measurement;
    sseq.readFrame(i, &measurement);
    cv::imshow("Measurement", measurement.depthImage());

    Frame mapframe;
    mapframe.depth_ = DepthMatPtr(new DepthMat(measurement.depth_->rows(), measurement.depth_->cols()));
    mapframe.depth_->setZero();
//    double stdev_thresh = numeric_limits<double>::max();
    //double stdev_thresh = 0.03;
    sseq.model_.estimateMapDepth(*map, transform, measurement, mapframe.depth_.get());
				 
    cv::imshow("Map estimate", mapframe.depthImage());

    Frame naive_mapframe;
    Cloud transformed;
    transformPointCloud(*map, transformed, transform);
    sseq.model_.cloudToFrame(transformed, &naive_mapframe);
    cv::imshow("Naive map", naive_mapframe.depthImage());

    cv::waitKey();
  }
    
  
  return 0;
}
