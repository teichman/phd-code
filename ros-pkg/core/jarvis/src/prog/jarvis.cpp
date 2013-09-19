#include <jarvis/jarvis.h>
#include <boost/program_options.hpp>

using namespace std;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  int vis_level;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("vis-level,v", bpo::value(&vis_level)->default_value(0), "")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }
  
  ros::init(argc, argv, "Jarvis");
  Jarvis jarvis(vis_level);
  ros::spin();
  
  return 0;
}
