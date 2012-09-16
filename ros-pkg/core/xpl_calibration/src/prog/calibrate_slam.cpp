#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <xpl_calibration/slam_calibration_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  vector<string> sequence_paths;
  vector<string> trajectory_paths;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value< vector<string> >(&sequence_paths)->required(), "StreamSequences, i.e. asus data.")
    ("traj", bpo::value< vector<string> >(&trajectory_paths)->required(), "Trajectories from slam.")
    ("batch", "Don't visualize.")
    ("omodel", bpo::value<string>()->default_value("model.psm"), "Output path for learned model.")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " OPTS  --sseq SSEQ --traj TRAJ [ --sseq SSEQ --traj TRAJ ... ]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ROS_ASSERT(sequence_paths.size() == trajectory_paths.size());
  SlamCalibrator::Ptr calibrator(new SlamCalibrator);
  calibrator->trajectories_.resize(trajectory_paths.size());
  cout << "Using sequence / trajectory pairs: " << endl;
  for(size_t i = 0; i < sequence_paths.size(); ++i) {
    cout << "  " << sequence_paths[i] << " ----- " << trajectory_paths[i] << endl;

    calibrator->trajectories_[i].load(trajectory_paths[i]);
    cout << "Trajectory: " << endl;
    cout << calibrator->trajectories_[i].status("  ");

    StreamSequence::Ptr sseq(new StreamSequence);
    sseq->load(sequence_paths[i]);
    calibrator->sseqs_.push_back(sseq);
  }

  if(opts.count("batch")) {
    PrimeSenseModel model = calibrator->calibrate();
    model.save(opts["omodel"].as<string>());
    cout << "Saved model to " << opts["omodel"].as<string>() << endl;
    return 0;
  }
  
  SlamCalibrationVisualizer vis(calibrator);
  vis.run();
  
  return 0;
}
