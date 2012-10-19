#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <gperftools/profiler.h>
#include <xpl_calibration/primesense_slam.h>

using namespace std;
using namespace g2o;
using namespace rgbd;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data.")
    ("opcd", bpo::value<string>()->required(), "Output DIR for the final pointcloud.")
    ("otraj", bpo::value<string>()->required(), "Output DIR for the final trajectory.")
    ("ograph", bpo::value<string>()->required(), "Output PATH for the final pose graph.")
    ("max-loopclosures", bpo::value<int>())
    ("cam", bpo::value<string>(), "Camera file to use.")
    ("visualize", "")
    ;

  p.add("sseq", 1).add("opcd", 1).add("otraj", 1).add("ograph", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ OPCD OTRAJ OGRAPH [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Using " << opts["sseq"].as<string>() << endl;
  cout << "Saving final map at " << opts["opcd"].as<string>() << endl;
  ROS_ASSERT(!bfs::exists(opts["opcd"].as<string>()));
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(opts["sseq"].as<string>());

  PrimeSenseSlam pss;
  if(opts.count("max-loopclosures"))
    pss.max_loopclosures_ = opts["max-loopclosures"].as<int>();
  pss.sseq_ = sseq;
  FrameAlignmentVisualizer fav(sseq->model_, sseq->model_);
  if(opts.count("visualize"))
    pss.fav_ = &fav;
  ThreadPtr slamthread = pss.launch();
  if(opts.count("visualize"))
    fav.run();
  slamthread->join();

  // -- Save outputs.
  if(opts.count("opcd"))
  {
    boost::filesystem::path dir(opts["opcd"].as<string>());
    boost::filesystem::create_directory(dir);
    for(size_t i = 0; i < pss.maps_.size(); i++)
    {
      ostringstream oss;
      oss << opts["opcd"].as<string>() << "/submap_" << i << ".pcd";
      pcl::io::savePCDFileBinary(oss.str(), *pss.maps_[i]);
    }
  }
  if(opts.count("otraj"))
  {
    boost::filesystem::path dir(opts["otraj"].as<string>());
    boost::filesystem::create_directory(dir);
    for(size_t i = 0; i < pss.trajs_.size(); i++)
    {
      ostringstream oss;
      oss << opts["otraj"].as<string>() << "/submap_" << i << ".traj";
      pss.trajs_[i].save(oss.str());
    }
  }
  if(opts.count("ograph"))
  {
    pss.pgs_->save(opts["ograph"].as<string>());
  }
  
  return 0;
}
