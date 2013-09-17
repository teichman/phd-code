#include <jarvis/bagvis.h>
#include <boost/program_options.hpp>

using namespace std;
using namespace sentinel;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("bag", bpo::value(&path)->required(), "location of bagfile")
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
  bv.run();
  
  return 0;
}
