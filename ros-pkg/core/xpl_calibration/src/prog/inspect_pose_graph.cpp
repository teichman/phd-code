
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <gperftools/profiler.h>
#include <pose_graph_slam/pose_graph_slam.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <xpl_calibration/utility_functions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <xpl_calibration/trajectory.h>
#include <xpl_calibration/primesense_slam.h>

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
    ("otraj", bpo::value<string>(), "Out DIR for the final trajectory.")
    ("opcd", bpo::value<string>(), "Out DIR for the final map pcd.")
    ("ograph", bpo::value<string>(), "Out PATH for the final posegraph.")
    ("trans_threshold", bpo::value<double>(), "Edge error threshold")
    ("rot_threshold", bpo::value<double>(), "Edge error threshold")

    ("visualize", "Visualize incrementally removed edges")
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
  string otraj = opts.count("otraj") ? opts["otraj"].as<string>() : "";
  string opcd = opts.count("opcd") ? opts["opcd"].as<string>() : "";
  double trans_threshold = opts.count("trans_threshold") ?
    opts["trans_threshold"].as<double>() :
    PrimeSenseSlam::defaultParams().get<double>("edge_trans_threshold");
  double rot_threshold = opts.count("rot_threshold") ?
    opts["rot_threshold"].as<double>() :
    PrimeSenseSlam::defaultParams().get<double>("edge_rot_threshold");
  

  cout << "Using " << opts["sseq"].as<string>() << endl;
  cout << "Using pose graph " << opts["posegraph"].as<string>() << endl;
  bool visualize = opts.count("visualize");
  StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory (opts["sseq"].as<string> ());
  PoseGraphSlam::Ptr slam(new PoseGraphSlam);
  slam->load(opts["posegraph"].as<string>());

  pcl::visualization::PCLVisualizer *vis;
  if(visualize)
  {
    vis = new pcl::visualization::PCLVisualizer();
  }
  //Build the map
  slam->solve();
  cout << "Original graph" << endl;
  if(visualize)
  {
    slam->visualize(*vis);
    vis->spin();
  }
  size_t num_pruned = slam->pruneUnsatisfiedEdges(trans_threshold, rot_threshold, 25);
  slam->solve();
  cout << "Pruned " << num_pruned << " unsatisfied edges" << endl;
  if(visualize)
  {
    slam->visualize(*vis);
    vis->spin();
  }
  num_pruned = slam->pruneAllSatisfiedEdges();
  slam->solve();
  cout << "Pruned " << num_pruned << " fully satisfied edges" << endl;
  if(visualize)
  {
    slam->visualize(*vis);
    vis->spin();
  }
  
  //Build the map again
  cout << "Rebuilding map(s)" << endl;
  PrimeSenseSlam pss;
  pss.pgs_ = slam;
  pss.sseq_ = sseq;
  pss.populateTrajAndMaps();
  for(size_t i = 0; i < pss.maps_.size(); i++)
  {
    Cloud::Ptr map = pss.maps_[i];
    // Save it
    if(opcd != "")
    {
      boost::filesystem::path dir(opcd);
      boost::filesystem::create_directory(dir);
      ostringstream oss;
      oss << opcd << "/submap_" << i << ".pcd";
      pcl::io::savePCDFileBinary(oss.str(), *map);
      cout << "Saved final map to " << oss.str() << endl;
    }
  }
  if(otraj != "")
  {
    for(size_t i = 0; i < pss.trajs_.size(); i++)
    {
      const Trajectory &traj = pss.trajs_[i];
      boost::filesystem::path dir(otraj);
      boost::filesystem::create_directory(dir);
      ostringstream oss;
      oss << otraj << "/submap_" << i << ".traj";
      traj.save(oss.str());
      cout << "Saved trajectory to " << oss.str() << endl;
    }
  }
  // Save the graph
  if(opts.count("ograph"))
    slam->save(opts["ograph"].as<string>());
  if(visualize)
  {
    delete vis;
  }
  return 0;
}
