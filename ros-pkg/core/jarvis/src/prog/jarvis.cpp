#include <jarvis/jarvis.h>
#include <boost/program_options.hpp>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Jarvis");
  
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  int vis_level;
  int rotation;
  string output_directory;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("vis-level,v", bpo::value(&vis_level)->default_value(0), "")
    ("rotation,r", bpo::value(&rotation)->default_value(0), "")
    ("output-directory,o", bpo::value(&output_directory)->default_value("jarvis_tds"), "Where to save TD files")
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

  cout << "Using vis_level " << vis_level << " and rotation " << rotation << endl;
  cout << "Saving TD files to \"" << output_directory << "\"" << endl;
  
  Jarvis jarvis(vis_level, rotation, output_directory);
  ros::spin();

  // -- Save any remaining tracks.
  jarvis.flush();
  
  return 0;
}
