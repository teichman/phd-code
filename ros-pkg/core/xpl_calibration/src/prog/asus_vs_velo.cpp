#include <boost/program_options.hpp>
#include <xpl_calibration/asus_vs_velo_visualizer.h>

using namespace std;
using namespace pcl;
using namespace rgbd;
namespace bpo = boost::program_options;  

int main(int argc, char** argv)
{
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>(), "StreamSequence, i.e. asus data")
    ("vseq", bpo::value<string>(), "VeloSequence")
    ("extrinsics", bpo::value<string>(), "Use pre-computed extrinsics")
    ("intrinsics", bpo::value<string>(), "Use pre-computed PrimeSense model")
    ("compute-extrinsics", "Automatically start extrinsics search")
    ("visualize-distortion", "Visualize the distortion.  Extrinsics must be provided.")
    ("skip", bpo::value<int>()->default_value(20), "For use with --visualize-distortion.  Use every kth frame for accumulating statistics.")
    ("num-pixel-plots", bpo::value<int>()->default_value(20), "For use with --visualize-distortion.  Number of random pixel plots to generate.")
    ;

  bpo::positional_options_description p;
  p.add("sseq", 1);
  p.add("vseq", 2);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);
  
  cout << "Loading StreamSequence at " << opts["sseq"].as<string>() << endl;
  cout << "Loading VeloSequence at " << opts["vseq"].as<string>() << endl;

  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(opts["sseq"].as<string>());

  VeloSequence::Ptr vseq(new VeloSequence(opts["vseq"].as<string>()));
  AsusVsVeloVisualizer avv(sseq, vseq);

  if(opts.count("compute-extrinsics")) {
    ROS_ASSERT(!opts.count("extrinsics"));
    cout << "Computing extrinsics and saving them with basename " << opts["compute-extrinsics"].as<string>() << endl;
    avv.calibrate();
    avv.saveExtrinsics(opts["compute-extrinsics"].as<string>());
    return 0;
  }    
  
  if(opts.count("extrinsics")) {
    ROS_ASSERT(!opts.count("compute-extrinsics"));
    avv.cal_.load(opts["extrinsics"].as<string>());
    cout << "Loaded calibration at " << opts["extrinsics"].as<string>() << "." << endl;
    cout << avv.cal_.offset_ << endl;
    cout << avv.cal_.velo_to_asus_.matrix() << endl;
  }

  if(opts.count("intrinsics")) {
    avv.model_.load(opts["intrinsics"].as<string>());
    cout << "Loaded depth distortion model at " << opts["intrinsics"].as<string>() << endl;
    cout << "Model:" << endl;
    cout << avv.model_.status("  ");
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
