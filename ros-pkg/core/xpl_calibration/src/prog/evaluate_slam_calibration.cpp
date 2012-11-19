#include <boost/program_options.hpp>
#include <xpl_calibration/slam_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
namespace bpo = boost::program_options;


int main(int argc, char** argv)
{

  vector<string> data_paths;
  string intrinsics_path;
  string output_path;

  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("data", bpo::value< vector<string> >(&data_paths)->required(), "Data paths in pairs of (StreamSequence, Trajectory).")
    ("intrinsics", bpo::value<string>(&intrinsics_path)->required(), "Discrete distortion model.")
    ("output", bpo::value<string>(&output_path)->required(), "Directory to put results.  Must not exist; will be created.")
    ;
    
  p.add("intrinsics", 1).add("output", 1).add("data", 100);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " INTRINSICS OUTPUT SSEQ TRAJ [ SSEQ TRAJ ... ]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << endl;
  cout << "--------------------" << endl;
  cout << "Evaluating intrinsics located at " << intrinsics_path << endl;
  ROS_ASSERT(!bfs::exists(output_path));
  cout << "Saving output to " << output_path << endl;
  cout << "--------------------" << endl;

  vector<StreamSequence::ConstPtr> sseqs;
  vector<Trajectory> trajectories;
  vector<string> names;
  ROS_ASSERT(data_paths.size() % 2 == 0);
  for(size_t i = 0; i < data_paths.size(); ++i) {
    if(i % 2 == 0) {
      cout << endl;
      string path = data_paths[i];
      cout << "Loading StreamSequence at " << path << endl;
      StreamSequence::Ptr sseq(new StreamSequence);
      sseq->load(path);
      sseqs.push_back(sseq);
      
      // Get the StreamSequence name.
      if(path[path.size() - 1] == '/')
	path = path.substr(0, path.size() - 1);
      string name = path.substr(path.find_last_of("/") + 1, string::npos);
      cout << "StreamSequence name: " << name << endl;
      names.push_back(name);
    }
    else {
      cout << "Loading Trajectory at " << data_paths[i] << endl;
      Trajectory traj;
      traj.load(data_paths[i]);
      trajectories.push_back(traj);
    }
  }
  cout << endl;
  cout << "--------------------" << endl;
  cout << endl;

  //bfs::create_directory(output_path);
  for(size_t i = 0; i < trajectories.size(); ++i) {
    ostringstream oss;
    oss << output_path << "/" << names[i];
    string eval_path = oss.str();
    cout << "Saving evaluation results to " << eval_path << endl;
    
    //DiscreteDepthDistortionModel intrinsics = calibrateLOO(sseqs, trajectories, i);
    //bfs::create_directory(eval_path);
    //evaluate(intrinsics, *sseqs[i], eval_path);
  }

  return 0;

}

