#include <boost/program_options.hpp>
#include <gperftools/profiler.h>
#include <xpl_calibration/slam_calibrator.h>
#include <xpl_calibration/primesense_slam.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "A StreamSequence.")
    ("intrinsics", bpo::value<string>()->required(), "A discrete depth distortion model to evaluate.")
    ("max-num", bpo::value<size_t>()->default_value(50))
    ;

  p.add("sseq", 1).add("intrinsics", 1);
     
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ INTRINSICS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  StreamSequence sseq;
  sseq.load(opts["sseq"].as<string>());
  DiscreteDepthDistortionModel dddm;
  dddm.load(opts["intrinsics"].as<string>());

  // -- Load some frames.
  vector<Frame> frames;
  Frame frame;
  size_t step = 10;
  for(size_t i = 0; i < min(step * opts["max-num"].as<size_t>(), sseq.size()); i += step) {
    sseq.readFrame(i, &frame);
    frames.push_back(frame);
  }
  
  // -- Test undistortion time.
  VectorXd times(frames.size());
  //ProfilerStart("undistortion_time_experiment.prof");
  for(size_t i = 0; i < frames.size(); ++i) {
    HighResTimer hrt;
    hrt.start();
    dddm.undistort(&frames[i]);
    hrt.stop();
    times(i) = hrt.getMilliseconds();
  }
  //ProfilerStop();
  cout << "Evaluated undistortion time for " << times.rows() << " frames." << endl;
  cout << "Mean undistortion time (ms): " << times.sum() / times.rows() << endl;
  cout << "Standard deviation: " << eigen_extensions::stdev(times) << endl;
  cout << "Absolute maximum: " << times.maxCoeff() << endl;

  return 0;
}
