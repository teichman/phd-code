#include <performance_statistics/performance_statistics.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <online_learning/dataset.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;


void computeHoldoutFrameAVC(string path, const bpo::variables_map& opts)
{
  if(!bfs::exists(path + "/holdout_frame_predictions.eig"))
    return;
  if(!opts.count("regen-confidence") && bfs::exists(path + "/avc-holdout-frame-car.png"))
    return;
  
  cout << "Plotting holdout frame acc_vs_confidence for " << path << endl;
  MatrixXf predictions, labels;
  eigen_extensions::load(path + "/holdout_frame_predictions.eig", &predictions);
  eigen_extensions::load(path + "/holdout_frame_labels.eig", &labels);
  NameMapping cmap;
  cmap.load(path + "/cmap.txt");
  PerfStats frame_stats(cmap);
	
  for(int j = 0; j < predictions.rows(); ++j) {
    Label pred = (VectorXf)predictions.row(j);
    Label label = (VectorXf)labels.row(j);
    // TODO: id() is going to be trouble.
    int id = label.id();
    if(id > -2) {
      ROS_WARN_ONCE("holdout acc_vs_confidence plot does not include weighting.");
      frame_stats.incrementStats(id, pred);
	
      // This only applies for weights >1.
      // ROS_WARN_ONCE("Using a gross hack to reflect the hand-labeled weights in the holdout plot.");
      // double mag = max(label.maxCoeff(), fabs(label.minCoeff()));
      // for(int k = 1; k < label.rows(); ++k)
      //   ROS_ASSERT(fabs(label(k)) < 1e-5 || fabs(fabs(label(k)) - mag) < 1e-5);
	  
      // for(double k = 0; k < mag - 1e-5; ++k)
      //   frame_stats.incrementStats(id, pred);
    }
  }
  frame_stats.saveAccuracyVsConfidence(path + "/avc-holdout-frame",
				       "Holdout set frame results",
				       opts["num-bars"].as<int>(),
				       opts["max-y"].as<float>());
}

void computeHoldoutTrackAVC(string path, const bpo::variables_map& opts)
{
  if(!bfs::exists(path + "/holdout_track_predictions.eig"))
    return;
  if(!opts.count("regen-confidence") && bfs::exists(path + "/avc-holdout-track-car.png"))
    return;
  
  cout << "Plotting holdout track acc_vs_confidence for " << path << endl;
  MatrixXf predictions, labels;
  eigen_extensions::load(path + "/holdout_track_predictions.eig", &predictions);
  eigen_extensions::load(path + "/holdout_track_labels.eig", &labels);
  NameMapping cmap;
  cmap.load(path + "/cmap.txt");
  PerfStats track_stats(cmap);
	
  for(int j = 0; j < predictions.rows(); ++j) {
    Label pred = (VectorXf)predictions.row(j);
    Label label = (VectorXf)labels.row(j);
    // TODO: id() is going to be trouble.
    int id = label.id();
    if(id > -2) {
      ROS_WARN_ONCE("holdout acc_vs_confidence plot does not include weighting.");
      track_stats.incrementStats(id, pred);
	
      // This only applies for weights >1.
      // ROS_WARN_ONCE("Using a gross hack to reflect the hand-labeled weights in the holdout plot.");
      // double mag = max(label.maxCoeff(), fabs(label.minCoeff()));
      // for(int k = 1; k < label.rows(); ++k)
      //   ROS_ASSERT(fabs(label(k)) < 1e-5 || fabs(fabs(label(k)) - mag) < 1e-5);
	  
      // for(double k = 0; k < mag - 1e-5; ++k)
      //   track_stats.incrementStats(id, pred);
    }
  }
  track_stats.saveAccuracyVsConfidence(path + "/avc-holdout-track",
				       "Holdout set track results",
				       opts["num-bars"].as<int>(),
				       opts["max-y"].as<float>());
}

void computeTestAVC(string path, const bpo::variables_map& opts)
{
  if(!bfs::exists(path + "/test_track_predictions.eig"))
    return;
  if(!opts.count("regen-confidence") && bfs::exists(path + "/avc-test-track-car.png"))
    return;
  
  cout << "Plotting acc_vs_confidence for " << path << endl;
  MatrixXf predictions, annotations;
  eigen_extensions::load(path + "/test_track_predictions.eig", &predictions);
  eigen_extensions::load(path + "/test_track_annotations.eig", &annotations);
  NameMapping cmap;
  cmap.load(path + "/cmap.txt");
  PerfStats track_stats(cmap);
  
  for(int j = 0; j < predictions.rows(); ++j) {
    Label pred = (VectorXf)predictions.row(j);
    Label label = (VectorXf)annotations.row(j);
    // TODO: This is going to be trouble.
    int id = label.id();
    if(id > -2)
      track_stats.incrementStats(id, pred);
  }
  track_stats.saveAccuracyVsConfidence(path + "/avc-test-track",
				       "Test set track results",
				       opts["num-bars"].as<int>(),
				       opts["max-y"].as<float>());

  
  // -- Get the thresholds.
  int retval = system(("cat " + path + "/learner_status.txt | sed -n '/Per-class ind/,/Boosting trainer/p' | sed '$d'").c_str());
  --retval;
  
  // string tmpfile = ".plotting-aotehsuthaosetuhsnaotehu";
  // vector<float> uvec, lvec;
  // int retval = system(("cat " + path + "/learner_status.txt | grep 'Lower ind' -A1 | grep -v Lower > " + tmpfile).c_str());
  // float buf;
  // ifstream file(tmpfile.c_str());
  // while(file >> buf, !file.eof())
  //   uvec.push_back(buf);
  // file.close();
  // VectorXf lower(lvec.size());
  // for(int i = 0; i < lower.rows(); ++i)
  //   lower(i) = lvec[i];

  // retval = system(("cat " + path + "/learner_status.txt | grep 'Upper ind' -A1 | grep -v Upper > " + tmpfile).c_str()); --retval;
  // ifstream file2(tmpfile.c_str());
  // while(file2 >> buf, !file2.eof())
  //   lvec.push_back(buf);
  // file2.close();
  // VectorXf upper(uvec.size());
  // for(int i = 0; i < upper.rows(); ++i)
  //   upper(i) = uvec[i];

  // cout << "Upper: " << upper.transpose() << endl;
  // cout << "Lower: " << lower.transpose() << endl;

  // -- Compute 
  
}

int main(int argc, char** argv)
{
  // -- Parse args.
  string root = ".";
  int iter = -1;
  
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("root", bpo::value<string>(&root), "TBSSL root path")
    ("test", "plot test accuracy histograms")
    ("holdout", "plot holdout frame accuracy histograms")
    ("holdout-track", "plot holdout track accuracy histograms")
    ("regen-confidence,r", "Regenerate all accuracy vs confidence plots")
    ("num-bars", bpo::value<int>()->default_value(50), "Number of bars in acc_vs_conf plots")
    ("max-y", bpo::value<float>()->default_value(0), "Maximum number of examples on the y axis of the plot")
    ("iter", bpo::value<int>(&iter), "Specific iteration number to plot")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);

  if(!bfs::exists(root + "/learner.ol")) {
    cout << "Root path \"" << root << "\" does not appear to be a TBSSL directory." << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Get contents of root path in order.
  vector<string> iter_paths;
  bfs::recursive_directory_iterator it(root), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    if(is_directory(p))
      iter_paths.push_back(p.string());
  }
  sort(iter_paths.begin(), iter_paths.end());

  // -- Main loop through iters.
  for(size_t i = 0; i < iter_paths.size(); ++i) {
    if(iter >= 0 && i != (size_t)iter)
      continue;
    
    string path = iter_paths[i];
    if(opts.count("test"))
      computeTestAVC(path, opts);
    if(opts.count("holdout"))
      computeHoldoutFrameAVC(path, opts);
    if(opts.count("holdout-track"))
      computeHoldoutTrackAVC(path, opts);
  }
        
  return 0;
}
