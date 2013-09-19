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
  opts_desc.add_options()
    ("datasets", bpo::value< vector<string> >(&paths)->required()->multitoken())
    ;

  p.add("datasets", -1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " PATHS" << endl; 
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ROS_WARN("This program will strip your .td of custom data.");
  
  for(size_t i = 0; i < paths.size(); ++i) {
    cout << paths[i] << endl;
    TrackDataset td((IfstreamWrapper(paths[i])));
    cout << "  Before: " << endl;
    cout << td.status("    ") << endl;
    for(size_t j = 0; j < td.size(); ++j)
      for(size_t k = 0; k < td[j].size(); ++k)
        td[j][k].label_.setZero();
    cout << "  After: " << endl;
    cout << td.status("    ") << endl;
    string tmpfile = paths[i] + ".unlabeling.tmp";
    td.save(tmpfile);
    int retval = system(("mv " + tmpfile + " " + paths[i]).c_str());
    ROS_ASSERT(retval == 0);
  }
  
  return 0;
}
