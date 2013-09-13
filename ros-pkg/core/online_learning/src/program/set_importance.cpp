#include <boost/program_options.hpp>
#include <online_learning/dataset.h>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  double importance;
  vector<string> paths;
  
  opts_desc.add_options()
    ("importance", bpo::value<double>(&importance)->required())
    ("datasets", bpo::value< vector<string> >(&paths)->required()->multitoken())
    ;

  p.add("importance", 1).add("datasets", -1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " IMPORTANCE PATHS" << endl; 
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  for(size_t i = 0; i < paths.size(); ++i) {
    cout << paths[i] << endl;
    TrackDataset td((IfstreamWrapper(paths[i])));
    cout << "  Before: " << endl;
    cout << td.status("    ") << endl;
    td.setImportance(importance);
    cout << "  After: " << endl;
    cout << td.status("    ") << endl;
    string tmpfile = ".tmp-" + paths[i];
    td.save(tmpfile);
    int retval = system(("mv " + tmpfile + " " + paths[i]).c_str());
    ROS_ASSERT(retval == 0);
  }
  
  return 0;
}
