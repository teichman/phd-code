#include <online_learning/cross_evaluator.h>
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

  vector<string> dataset_paths;
  string output_dir;
  vector<size_t> num_cells;
  num_cells.push_back(10);
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("tds", bpo::value(&dataset_paths)->required()->multitoken(), "Labeled data")
    ("output,o", bpo::value(&output_dir)->required(), "Output directory")
    ("num-cells", bpo::value< vector<size_t> >(&num_cells)->multitoken(), "Default is --num-cells 10.")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] TD [ TD ... ] -o OUTPUT_DIR" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Get dataset names.
  vector<string> dataset_names(dataset_paths.size());
  for(size_t i = 0; i < dataset_paths.size(); ++i) {
    bfs::path path(dataset_paths[i]);
    dataset_names[i] = path.stem().string();
    cout << "Name: " << dataset_names[i] << endl;
  }
      
  // -- Load all datasets.
  vector<TrackDataset::ConstPtr> tds;
  for(size_t i = 0; i < dataset_paths.size(); ++i) {
    cout << "Loading " << dataset_paths[i] << endl;
    TrackDataset::Ptr td(new TrackDataset);
    td->load(dataset_paths[i]);
    if(i > 0)
      ROS_ASSERT(td->nameMappingsAreEqual(*tds[i-1]));
    tds.push_back(td);
  }

  CrossEvaluator ce;
  for(size_t i = 0; i < tds.size(); ++i)
    ce.addTrackDataset(tds[i], dataset_names[i]);

  ce.evaluate(output_dir, num_cells);

  return 0;
}
