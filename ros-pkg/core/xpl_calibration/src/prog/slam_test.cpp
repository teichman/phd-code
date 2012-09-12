#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <gperftools/profiler.h>
#include <xpl_calibration/slam_visualizer.h>

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
    ("opcd", bpo::value<string>()->required(), "Output path for the final pointcloud.")
    ("otraj", bpo::value<string>()->required(), "Output path for the final trajectory.")
    ("save-imgs", "Saves slam*.png to the current directory.  Slows things down considerably.")
    ("loop-closure", "Adds loop closure.")
    ("cam", bpo::value<string>(), "Camera file to use.")
    ;

  p.add("sseq", 1).add("opcd", 1).add("otraj", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ OPCD OTRAJ [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Using " << opts["sseq"].as<string>() << endl;
  cout << "Saving final map at " << opts["opcd"].as<string>() << endl;
  ROS_ASSERT(!bfs::exists(opts["opcd"].as<string>()));
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(opts["sseq"].as<string>());
  
  SlamVisualizer vis;
  vis.save_imgs_ = opts.count("save-imgs");
  vis.use_loop_closure_ = opts.count("loop-closure");
  if(opts.count("cam"))
    vis.setCamera(opts["cam"].as<string>());

  vis.run(sseq, opts["opcd"].as<string>(), opts["otraj"].as<string>());
  
  return 0;
}
