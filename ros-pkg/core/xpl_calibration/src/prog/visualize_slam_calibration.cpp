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

  string sequence_path;
  string trajectory_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>(&sequence_path)->required(), "StreamSequence, i.e. asus data.")
    ("traj", bpo::value<string>(&trajectory_path)->required(), "Trajectory from slam.")
    ("imodel", bpo::value<string>(), "Optional discrete distortion model.")
    ("max-range", bpo::value<double>()->default_value(MAX_RANGE_MAP), "Maximum range to use when building the map from the given trajectory.")
    // This is different from DEFAULT_VGSIZE because otherwise there are two many points to display in the PCLVisualizer.
    // OpenGL display lists come to mind...
    ("vgsize", bpo::value<double>()->default_value(0.03), "Voxel grid cell size.")  
    ;

  p.add("sseq", 1).add("traj", 1);
  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " OPTS SSEQ TRAJ" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Using sequence / trajectory pairs: " << endl;
  Trajectory traj;
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(sequence_path);
  traj.load(trajectory_path);
  vector<Trajectory> trajectories;
  trajectories.push_back(traj);
  vector<StreamSequence::ConstPtr> sseqs;
  sseqs.push_back(sseq);
  
  SlamCalibrator::Ptr calibrator(new SlamCalibrator(sseqs[0]->model_,
						    opts["max-range"].as<double>(),
						    opts["vgsize"].as<double>()));
  cout << "Using " << calibrator->max_range_ << " as max range." << endl;
  calibrator->trajectories_ = trajectories;
  calibrator->sseqs_ = sseqs;

  SlamCalibrationVisualizer vis(calibrator);
  if(opts.count("imodel")) {
    cout << "Using custom discrete distortion model at " << opts["imodel"].as<string>() << endl;
    vis.dddm_ = new DiscreteDepthDistortionModel;
    vis.dddm_->load(opts["imodel"].as<string>());
  }
  vis.run();
    
  return 0;
}
