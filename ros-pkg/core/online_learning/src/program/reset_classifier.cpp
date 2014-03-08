#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string input_path;
  vector<string> classes;
  string output_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("input,i", bpo::value(&input_path)->required(), "Input grid classifier")
    ("reset,r", bpo::value(&classes)->required()->multitoken(), "Classes to reset")
    ("output,o", bpo::value(&output_path)->required(), "Location to save output classifier")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " -i PATH -r CLASS [ CLASS ... ] -o PATH" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  GridClassifier gc;
  cout << "Loading " << input_path << "." << endl;
  gc.load(input_path);
  cout << "GridClassifier status: " << endl;
  cout << gc.status("  ", true) << endl;

  for(size_t i = 0; i < classes.size(); ++i) {
    cout << "Resetting " << classes[i] << " classifier." << endl;
    gc.setZero(gc.nameMapping("cmap").toId(classes[i]));
  }

  cout << "Saving to " << output_path << "." << endl;
  gc.save(output_path);
  
  return 0;
}
