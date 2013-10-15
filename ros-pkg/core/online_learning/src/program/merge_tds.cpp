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

  vector<string> td_paths;
  string output_path;
  vector<string> class_names;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("tds", bpo::value< vector<string> >(&td_paths)->required()->multitoken())
    ("output,o", bpo::value<string>(&output_path)->required())
    ("class-names", bpo::value(&class_names)->multitoken(), "Apply a new name map, if desired.")
    ;
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " --tds TD [TD ...] -o OUTPUT" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  NameMapping cmap;
  if(opts.count("class-names")) {
    cmap.addNames(class_names);
    cout << "Using cmap: " << endl;
    cout << cmap.status("  ") << endl;
  }
  TrackDataset::Ptr td = loadDatasets(td_paths, cmap, true);  // If cmap is empty, this will assume that all TDs have the same cmap.
  td->save(output_path);
  
  return 0;
}
