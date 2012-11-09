#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <xpl_calibration/slam_calibration_visualizer.h>
#include <xpl_calibration/primesense_slam.h>

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
    ("only-visualize", "Don't calibrate; just visualize the map you would have used to calibrate from.")
    ("imodel", bpo::value<string>(), "Use this model when projecting the frames.")
    ("omodel", bpo::value<string>()->default_value("model.psm"), "Output path for learned model.")
    ("max-range", bpo::value<double>()->default_value(MAX_RANGE_MAP), "Maximum range to use when building the map from the given trajectory.")
    ;

  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " OPTS  --sseq SSEQ --traj TRAJ [ --sseq SSEQ --traj TRAJ ... ]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Using sequence / trajectory pairs: " << endl;
  vector<Trajectory> trajectories(trajectory_paths.size());
  vector<StreamSequence::ConstPtr> sseqs;
  ROS_ASSERT(sequence_paths.size() == trajectory_paths.size());
  for(size_t i = 0; i < sequence_paths.size(); ++i) {
    cout << "  " << sequence_paths[i] << " ----- " << trajectory_paths[i] << endl;

    trajectories[i].load(trajectory_paths[i]);
    cout << "Trajectory: " << endl;
    cout << trajectories[i].status("  ");

    StreamSequence::Ptr sseq(new StreamSequence);
    sseq->load(sequence_paths[i]);
    sseqs.push_back(sseq);
  }

  SlamCalibrator::Ptr calibrator(new SlamCalibrator(sseqs[0]->model_, opts["max-range"].as<double>()));
  cout << "Using " << calibrator->max_range_ << " as max range." << endl;
  calibrator->trajectories_ = trajectories;
  calibrator->sseqs_ = sseqs;

  if(opts.count("imodel")) {
    cout << "Using custom primesense model at " << opts["imodel"].as<string>() << endl;
    calibrator->model_.load(opts["imodel"].as<string>());
    cout << calibrator->model_.status("  ");
  }
  
  if(opts.count("only-visualize")) {
    SlamCalibrationVisualizer vis(calibrator);
    vis.run();
    return 0;
  }
  else {
    PrimeSenseModel model = calibrator->calibrate();
    model.save(opts["omodel"].as<string>());
    cout << "Saved model to " << opts["omodel"].as<string>() << endl;
    return 0;
  }
    
  return 0;
}
