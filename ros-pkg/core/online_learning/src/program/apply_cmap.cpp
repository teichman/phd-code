#include <boost/program_options.hpp>
#include <online_learning/dataset.h>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  vector<string> paths;
  vector<string> names;
  opts_desc.add_options()
    ("help,h", "")
    ("datasets,d", bpo::value< vector<string> >(&paths)->required()->multitoken(), ".td files")
    ("names,n", bpo::value< vector<string> >(&names)->multitoken(), "Class names")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] -d TDS -n NAME [ NAME ... ]" << endl;
    cout << "  Applies a new class map composed of NAMES to all TDS." << endl;

    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  NameMapping cmap;
  for(size_t i = 0; i < names.size(); ++i)
    cmap.addName(names[i]);
  
  for(size_t i = 0; i < paths.size(); ++i) {
    cout << paths[i] << endl;
    TrackDataset td((IfstreamWrapper(paths[i])));
    cout << "  Before: " << endl;
    cout << td.status("    ") << endl;

    td.applyNameMapping("cmap", cmap);

    cout << "  After: " << endl;
    cout << td.status("    ") << endl;
    string tmpfile = paths[i] + ".labeling.tmp";
    td.save(tmpfile);
    int retval = system(("mv " + tmpfile + " " + paths[i]).c_str());
    ROS_ASSERT(retval == 0);
  }
  
  return 0;
}
