#include <sentinel/sentinel.h>
#include <boost/program_options.hpp>

using namespace std;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;
  
  string name;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("name", bpo::value(&name)->required(), "Data will be saved to .sentinel-name")
    ("visualize", "Whether to show the rgb stream")
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

  double update_interval = 1;
  double save_interval = 1;
  double threshold = 0.00001;
  int max_training_imgs = 600;
  Sentinel sen(name, update_interval,
               save_interval, max_training_imgs,
               threshold, opts.count("visualize"));
  sen.run();

  return 0;
}
