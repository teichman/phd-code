#include <boost/program_options.hpp>
#include <xpl_calibration/asus_vs_velo_visualizer.h>

using namespace std;
using namespace pcl;
using namespace rgbd;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data")
    ("vseq", bpo::value<string>()->required(), "VeloSequence")
    ("extrinsics", bpo::value<string>(), "Use pre-computed extrinsics")
    ("intrinsics", bpo::value<string>(), "Use pre-computed PrimeSense model")
    ("compute-extrinsics", "Automatically start extrinsics search")
    ("compute-intrinsics", "Automatically start intrinsics search")
    ("visualize-distortion", "Visualize the distortion.  Extrinsics must be provided.")
    ("skip", bpo::value<int>()->default_value(20), "For use with --visualize-distortion.  Use every kth frame for accumulating statistics.")
    ("num-pixel-plots", bpo::value<int>()->default_value(20), "For use with --visualize-distortion.  Number of random pixel plots to generate.")
    ;

  bpo::positional_options_description p;
  p.add("sseq", 1).add("vseq", 1);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: asus_vs_velo SSEQ VSEQ [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }
  
  
  cout << "Loading StreamSequence at " << opts["sseq"].as<string>() << endl;
  cout << "Loading VeloSequence at " << opts["vseq"].as<string>() << endl;

  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(opts["sseq"].as<string>());

  VeloSequence::Ptr vseq(new VeloSequence(opts["vseq"].as<string>()));
  AsusVsVeloVisualizer avv(sseq, vseq);

  if(opts.count("compute-extrinsics")) {
    ROS_ASSERT(!opts.count("extrinsics"));
    cout << "Computing extrinsics." << endl;
    avv.calibrate();
    avv.saveExtrinsics();  // "extrinsics"
    return 0;
  }    

  if(opts.count("extrinsics")) {
    ROS_ASSERT(!opts.count("compute-extrinsics"));
    avv.cal_.load(opts["extrinsics"].as<string>());
    cout << "Loaded calibration at " << opts["extrinsics"].as<string>() << "." << endl;
    cout << avv.cal_.status("  ");
  }

  if(opts.count("intrinsics")) {
    avv.model_.load(opts["intrinsics"].as<string>());
    cout << "Loaded depth distortion model at " << opts["intrinsics"].as<string>() << endl;
    cout << "Model:" << endl;
    cout << avv.model_.status("  ");
  }

  if(opts.count("compute-intrinsics")) {
    ROS_ASSERT(opts.count("extrinsics"));
    cout << "Computing depth distortion model." << endl;
    avv.fitModel();
    avv.saveIntrinsics();  // "intrinsics"
    return 0;
  }    
  
  if(opts.count("visualize-distortion")) {
    ROS_ASSERT(opts.count("extrinsics"));
    avv.skip_ = opts["skip"].as<int>();
    avv.num_pixel_plots_ = opts["num-pixel-plots"].as<int>();
    avv.fitModel();  // We don't need the resulting model.  This accumulates and caches pixel statistics, which we do need.
    avv.visualizeDistortion();
    return 0;
  }
    
  avv.run();
  
  return 0;
}
