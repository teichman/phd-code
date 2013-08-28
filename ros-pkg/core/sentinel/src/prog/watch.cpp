#include <sentinel/sentinel.h>
#include <boost/program_options.hpp>

using namespace std;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;
  
  string name;
  string resolution;
  double threshold;
  double save_interval;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("name", bpo::value(&name)->required(), "Data will be saved to .sentinel-name")
    ("visualize", "Whether to show the rgb stream")
    ("resolution,r", bpo::value(&resolution), "")
    ("threshold,t", bpo::value(&threshold)->default_value(0.03), "")
    ("save-interval,s", bpo::value(&save_interval)->default_value(1), "How often to save, in seconds")
    ("update-interval,u", bpo::value(&update_interval)->default_value(1), "How often to update, in seconds")
    ;

  p.add("name", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] NAME" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  OpenNI2Interface::Resolution res = OpenNI2Interface::VGA;
  if(opts.count("resolution")) {
    if(resolution == "QVGA" || resolution == "qvga")
      res = OpenNI2Interface::QVGA;
    else {
      cout << "Unrecognized resolution \"" << resolution << "\"." << endl;
      return 1;
    }
  }
  
  int max_training_imgs = 1000;
  DiskStreamingSentinel sen("sentinal-" + name, save_interval,
                            update_interval, max_training_imgs,
                            threshold, opts.count("visualize"), res);
  sen.run();

  return 0;
}
