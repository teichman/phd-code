#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <online_learning/grid_classifier.h>
#include <online_learning/clusterer.h>

using namespace std;
namespace bpo = boost::program_options;

void process(const TrackDataset& td, const GridClassifier& gc,
             const bpo::variables_map& opts,
             TrackDataset* std, TrackDataset* nstd)
{
  for(size_t i = 0; i < td.size(); ++i) {
    if(isStatic(td[i], gc, opts["thresh"].as<double>(), opts["slack"].as<double>()))
      std->tracks_.push_back(td.tracks_[i]);
    else
      nstd->tracks_.push_back(td.tracks_[i]);
  }
}

int main(int argc, char** argv)
{
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string classifier_path;
  string output_dir;
  vector<string> dataset_paths;
  
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("classifier,c", bpo::value<string>(&classifier_path), "")
    ("output,o", bpo::value<string>(&output_dir), "")
    ("datasets,d", bpo::value< vector<string> >(&dataset_paths)->required()->multitoken(), "")
    ("thresh,t", bpo::value<double>()->required(), "")
    ("slack,s", bpo::value<double>()->required(), "")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] -t THRESHOLD -d TD [ TD ... ] -c CLASSIFIER -o OUTPUT_DIR" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  if(!bfs::exists(output_dir))
    bfs::create_directory(output_dir);
  
  // -- Load the classifier.
  GridClassifier gc;
  cout << "Loading classifier " << classifier_path << "." << endl;
  gc.load(classifier_path);
  
  // -- Set up storage for static and nonstatic tracks.
  TrackDataset static_tracks;
  TrackDataset nonstatic_tracks;

  // -- Find static vs nonstatic tracks in all datasets.
  for(size_t i = 0; i < dataset_paths.size(); ++i) {
    TrackDataset td;
    cout << "Loading dataset " << dataset_paths[i] << "." << endl;
    td.load(dataset_paths[i]);
    td.applyNameMapping("cmap", gc.nameMapping("cmap"));
    ROS_ASSERT(td.nameMapping("dmap") == gc.nameMapping("dmap"));
    
    if(i == 0) {
      static_tracks.applyNameMappings(td);
      nonstatic_tracks.applyNameMappings(td);
    }
    
    process(td, gc, opts, &static_tracks, &nonstatic_tracks);
  }

  // -- Save static and nonstatic tracks.
  static_tracks.save(output_dir + "/static.td");
  nonstatic_tracks.save(output_dir + "/nonstatic.td");

  cout << "Done." << endl;
  return 0;
}
