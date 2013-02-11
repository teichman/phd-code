#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <replicator/box_selection_visualizer.h>

using namespace std;
using namespace rgbd;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string sequence_path;
  string trajectory_path;
  string output_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>(&sequence_path)->required(), "StreamSequence, i.e. asus data.")
    ("traj", bpo::value<string>(&trajectory_path)->required(), "Trajectory from slam.")
    ("max-range", bpo::value<double>()->default_value(MAX_RANGE_MAP), "Maximum range to use when building the map from the given trajectory.")
    ("vgsize", bpo::value<double>()->default_value(0.002), "Voxel grid cell size.")
    ("output,o", bpo::value<string>(&output_path)->default_value("selected_box.pcd"), "Output file to save to.")
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
    cout << "  To select a center point, use shift-click.  To change the extent of the box, use x/X/y/Y/z/Z." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  
  Trajectory traj;
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(sequence_path);
  traj.load(trajectory_path);
  Cloud::Ptr map = SlamCalibrator::buildMap(*sseq, traj,
                                            opts["max-range"].as<double>(),
                                            opts["vgsize"].as<double>());

  cout << "  To select a center point, use shift-click.  To change the extent of the box, use x/X/y/Y/z/Z." << endl;
  BoxSelectionVisualizer bsv(map, output_path);
  bsv.run();
  
  return 0;
}
