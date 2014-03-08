#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  vector<string> paths;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("paths", bpo::value(&paths)->required()->multitoken(), "List of GridClassifiers and/or TrackDatasets")
    ;

  p.add("paths", -1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] PATH [ PATH ... ]" << endl;
    cout << "  Where PATH is a GridClassifier or TrackDataset." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  for(size_t i = 0; i < paths.size(); ++i) {
    cout << "============================================================" << endl;
    cout << "== " << paths[i] << endl;
    cout << "============================================================" << endl;
    bfs::path path(paths[i]);
    if(path.extension().string() == ".gc") {
      GridClassifier gc;  
      gc.load(paths[i]);
      cout << "GridClassifier status: " << endl;
      cout << gc.status("  ", true) << endl;
    }
    else if(path.extension().string() == ".td") {
      TrackDataset td;
      td.load(paths[i]);
      cout << "TrackDataset status: " << endl;
      cout << td.status("  ", true) << endl;
    }
    else {
      cout << "Unknown extension \"" << path.extension().string() << "\"." << endl;
      cout << "Skipping path " << paths[i] << endl;
    }
    cout << endl;
  }

  return 0;
}
