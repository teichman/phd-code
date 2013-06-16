#include <boost/program_options.hpp>
#include <rgbd_sequence/primesense_model.h>
#include <xpl_calibration/slam_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
using namespace pcl::visualization;
namespace bfs = boost::filesystem;

bool g_started = false;

void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  cout << "keyboardCallback" << endl;
  if(event.getKeyCode() == 's')
    g_started = true;
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string save_dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data")
    ("traj", bpo::value<string>()->required(), "Trajectory")
    ("save-dir", bpo::value(&save_dir), "If not set, images will not be saved.")
    ;

  p.add("sseq", 1).add("traj", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] SSEQ TRAJ" << endl;  
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  if(opts.count("save-dir") && !bfs::exists(save_dir))
    bfs::create_directory(save_dir);
  
  // -- Load data.
  StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory(opts["sseq"].as<string>());
  Trajectory traj;
  traj.load(opts["traj"].as<string>());

  PCLVisualizer vis("Visualizer");
  vis.addPointCloud(Cloud::Ptr(new Cloud), "map");
  vis.setBackgroundColor(0, 0, 0);
  vis.registerKeyboardCallback(keyboardCallback);
  vis.setShowFPS(false);

  // -- Build map.
  Cloud::Ptr map = SlamCalibrator::buildMap(sseq, traj, MAX_RANGE_MAP, 0.05);

  // -- Display the map so you can set the camera angle.
  vis.updatePointCloud(map, "map");
  while(!g_started)
    vis.spinOnce(5);
  
  // -- Clear the screen and start running the visualization to be recorded.
  cout << "Started." << endl;
  vis.updatePointCloud(Cloud::Ptr(new Cloud), "map");

  // -- Display poses for the visualization.
  for(size_t i = 0; i < traj.size(); ++i) {
    if(!traj.exists(i))
      continue;
    cout << "i: " << i << endl;
    vis.addCoordinateSystem(0.3, traj.get(i).cast<float>());
    vis.spinOnce(10);
    if(opts.count("save-dir")) {
      ostringstream oss;
      oss << save_dir << "/A-" << setw(5) << setfill('0') << i << ".png";
      vis.saveScreenshot(oss.str());
    }
  }

  // -- Display map incrementally.
  map->clear();
  int num_used_frames = 0;
  for(size_t i = 0; i < traj.size(); ++i) {
    if(!traj.exists(i))
      continue;

    Frame frame;
    sseq->readFrame(i, &frame);
    
    Cloud::Ptr tmp(new Cloud);
    sseq->model_.frameToCloud(frame, tmp.get(), MAX_RANGE_MAP);
    Cloud::Ptr nonans(new Cloud);
    nonans->reserve(tmp->size());
    for(size_t j = 0; j < tmp->size(); ++j)
      if(isFinite(tmp->at(j)))
        nonans->push_back(tmp->at(j));
        
    pcl::transformPointCloud(*nonans, *nonans, traj.get(i).cast<float>());

    *map += *nonans;    
    ++num_used_frames;
    if(num_used_frames % 50 == 0)
    {
      cout << "Filtering..." << endl;
      HighResTimer hrt("filtering");
      hrt.start();
      pcl::VoxelGrid<rgbd::Point> vg;
      vg.setLeafSize(0.02, 0.02, 0.02);  // 2cm^3 voxel grid.
      Cloud::Ptr tmp(new Cloud);
      vg.setInputCloud(map);
      vg.filter(*tmp);
      *map = *tmp;
      hrt.stop();
    }

    vis.updatePointCloud(map, "map");
    vis.spinOnce(10);
    if(opts.count("save-dir")) {
      ostringstream oss;
      oss << save_dir << "/B-" << setw(5) << setfill('0') << i << ".png";
      vis.saveScreenshot(oss.str());
    }
  }

  // -- Display training example collection.
  vis.addPointCloud(Cloud::Ptr(new Cloud), "frame");
  for(size_t i = 0; i < traj.size(); ++i) {
    if(!traj.exists(i))
      continue;

    Frame frame;
    sseq->readFrame(i, &frame);

    Cloud::Ptr tmp(new Cloud);
    sseq->model_.frameToCloud(frame, tmp.get(), 10);  // 10m max range
    Cloud::Ptr nonans(new Cloud);
    nonans->reserve(tmp->size());
    for(size_t j = 0; j < tmp->size(); ++j)
      if(isFinite(tmp->at(j)))
        nonans->push_back(tmp->at(j));
        
    pcl::transformPointCloud(*nonans, *nonans, traj.get(i).cast<float>());
    for(size_t j = 0; j < nonans->size(); ++j) {
      nonans->at(j).r = 255;
      nonans->at(j).g = 0;
      nonans->at(j).b = 0;
    }

    vis.updatePointCloud(nonans, "frame");
    vis.spinOnce(10);
    if(opts.count("save-dir")) {
      ostringstream oss;
      oss << save_dir << "/C-" << setw(5) << setfill('0') << i << ".png";
      vis.saveScreenshot(oss.str());
    }
  }

  return 0;
}
