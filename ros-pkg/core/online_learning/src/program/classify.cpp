#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <online_learning/grid_classifier.h>

using namespace std;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string classifier_path;
  vector<string> td_paths;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("classifier,c", bpo::value<string>(&classifier_path), "")
    ("tds,d", bpo::value< vector<string> >(&td_paths)->required()->multitoken(), "")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] --tds TD [ TD ... ] -c CLASSIFIER" << endl;
    cout << "  Classifies the given TD files in-place.  Use with care." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Load the classifier.
  GridClassifier gc;
  cout << "Loading classifier " << classifier_path << "." << endl;
  gc.load(classifier_path);

  // -- Classify each dataset and save in-place.
  for(size_t i = 0; i < td_paths.size(); ++i) {
    TrackDataset td;
    cout << "Loading dataset " << td_paths[i] << "." << endl;
    td.load(td_paths[i]);
    td.applyNameMapping("cmap", gc.nameMapping("cmap"));
    ROS_ASSERT(gc.nameMappingsAreEqual(td));
    for(size_t j = 0; j < td.size(); ++j)
      td[j].setLabel(gc.classifyTrack(td[j]));
    td.save(td_paths[i]);
  }
    
  return 0;
}
