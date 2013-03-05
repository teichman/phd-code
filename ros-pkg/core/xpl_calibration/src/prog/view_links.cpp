
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <gperftools/profiler.h>
#include <pose_graph_slam/pose_graph_slam.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <xpl_calibration/utility_functions.h>
#include <pcl/common/transforms.h>
#include <xpl_calibration/trajectory.h>
#include <xpl_calibration/primesense_slam.h>
#include <xpl_calibration/link_visualizer.h>

using namespace std;
using namespace g2o;
using namespace rgbd;

typedef pcl::PointXYZ PointBW_t;
typedef pcl::PointCloud<PointBW_t> CloudBW_t;
typedef pcl::PointXYZRGB Point_t;
typedef pcl::PointCloud<Point_t> Cloud_t;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data.")
    ("posegraph", bpo::value<string>()->required(), "Input path for the complete pose graph.")
    ;

  p.add("sseq",1).add("posegraph", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ POSEGRAPH [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }
  cout << "Using " << opts["sseq"].as<string>() << endl;
  cout << "Using pose graph " << opts["posegraph"].as<string>() << endl;
  StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory (opts["sseq"].as<string> ());
  PoseGraphSlam::Ptr slam(new PoseGraphSlam);
  slam->load(opts["posegraph"].as<string>());
  LinkVisualizer lv(sseq, slam);
  lv.run();


  return 0;
}

