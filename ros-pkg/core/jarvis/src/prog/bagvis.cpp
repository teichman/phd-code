#include <jarvis/bagvis.h>
#include <boost/program_options.hpp>
#include <gperftools/profiler.h>

using namespace std;
using namespace sentinel;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string path;
  int max_num_bg;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("bag", bpo::value(&path)->required(), "location of bagfile")
    ("max-num-bg", bpo::value(&max_num_bg), "")
    ;

  p.add("bag", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] BAG" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  BagVis bv(path, 3000);
  if(opts.count("max-num-bg")) {
    bv.max_num_bg_ = max_num_bg;
    cout << "Processing " << bv.max_num_bg_ << " background patches max." << endl;
  }
  ProfilerStart("bagvis.prof");
  bv.run();
  ProfilerStop();
  
  return 0;
}
