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
  vector<string> pos_labels;
  vector<string> neg_labels;
  opts_desc.add_options()
    ("help,h", "")
    ("datasets,d", bpo::value< vector<string> >(&paths)->required()->multitoken(), ".td files")
    ("pos", bpo::value< vector<string> >(&pos_labels)->multitoken(), "positive classes")
    ("neg", bpo::value< vector<string> >(&neg_labels)->multitoken(), "negative classes")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] -d TDS" << endl;
    cout << "  The {-1, 0, +1} label for all class problems will be set to zero except for those that are" << endl;
    cout << "  set via --pos or --neg.  Custom data is preserved." << endl;

    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }
  
  for(size_t i = 0; i < paths.size(); ++i) {
    cout << paths[i] << endl;
    TrackDataset td((IfstreamWrapper(paths[i])));
    cout << "  Before: " << endl;
    cout << td.status("    ") << endl;

    Label annotation = VectorXf::Zero(td.nameMapping("cmap").size());
    for(size_t j = 0; j < pos_labels.size(); ++j)
      annotation(td.nameMapping("cmap").toId(pos_labels[j])) = +1;
    for(size_t j = 0; j < neg_labels.size(); ++j)
      annotation(td.nameMapping("cmap").toId(neg_labels[j])) = -1;
    cout << "Applying annotation " << annotation.transpose() << endl;
    cout << annotation.status(td.nameMapping("cmap"), "  ") << endl;

    for(size_t j = 0; j < td.size(); ++j) 
      td[j].setLabel(annotation);
    
    cout << "  After: " << endl;
    cout << td.status("    ") << endl;
    string tmpfile = paths[i] + ".labeling.tmp";
    td.save(tmpfile);
    int retval = system(("mv " + tmpfile + " " + paths[i]).c_str());
    ROS_ASSERT(retval == 0);
  }
  
  return 0;
}
