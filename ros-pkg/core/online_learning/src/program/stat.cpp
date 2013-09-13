#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  // -- Parse args.
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("gc", bpo::value<string>(), "Grid classifier")
    ("td", bpo::value<string>(), "Track dataset")
    ("classify", "Whether to classify the track dataset with the grid classifier")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);

  GridClassifier gc;  
  if(opts.count("gc")) {
    gc.load(opts["gc"].as<string>());
    cout << "GridClassifier status: " << endl;
    cout << gc.status("  ", true) << endl;
  }

  TrackDataset td;
  if(opts.count("td")) {
    td.load(opts["td"].as<string>());
    cout << "TrackDataset status: " << endl;
    cout << td.status("  ", true) << endl;
  }

  if(opts.count("classify")) {
    ROS_ASSERT(opts.count("gc") && opts.count("td"));
    for(size_t i = 0; i < td.size(); ++i) {
      cout << gc.classifyTrack(td[i]).status(gc.nameMapping("cmap")) << endl;
    }
  }

  
  return 0;
}
