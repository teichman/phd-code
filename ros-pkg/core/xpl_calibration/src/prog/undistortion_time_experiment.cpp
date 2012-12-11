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

  vector<string> sseq_paths;
  size_t step;
  
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseqs", bpo::value< vector<string> >(&sseq_paths)->required()->multitoken(), "StreamSequences.")
    ("intrinsics", bpo::value<string>()->required(), "A discrete depth distortion model to evaluate.")
    ("max-num", bpo::value<size_t>()->default_value(50), "Max number of frames per sequence.")
    ("step", bpo::value<size_t>(&step)->default_value(10))
    ;

  p.add("intrinsics", 1);
     
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " INTRINSICS --sseqs SSEQ [ SSEQ ... ]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  DiscreteDepthDistortionModel dddm;
  dddm.load(opts["intrinsics"].as<string>());

  vector<double> times_std;
  for(size_t i = 0; i < sseq_paths.size(); ++i) {
    StreamSequence sseq;
    sseq.load(sseq_paths[i]);
    cout << sseq_paths[i] << endl;
    for(size_t j = 0; j < min(step * opts["max-num"].as<size_t>(), sseq.size()); j += step) {
      Frame frame;
      sseq.readFrame(i, &frame);    
      HighResTimer hrt;
      hrt.start();
      dddm.undistort(&frame);
      hrt.stop();
      times_std.push_back(hrt.getMilliseconds());
    }
  }

  VectorXd times;
  eigen_extensions::stdToEig(times_std, &times);

  cout << endl;
  cout << "Evaluated undistortion time for " << times.rows() << " frames." << endl;
  cout << "Mean undistortion time (ms): " << times.sum() / times.rows() << endl;
  cout << "Standard deviation: " << eigen_extensions::stdev(times) << endl;
  cout << "Absolute maximum: " << times.maxCoeff() << endl;

  return 0;
}
