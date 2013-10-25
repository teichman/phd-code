#include <jarvis/cannon_reactor.h>
#include <boost/program_options.hpp>

using namespace std;
namespace bpo = boost::program_options;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cannon_reactor");
  
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  double delay;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("delay,d", bpo::value(&delay)->default_value(3), "")
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

  // usleep(delay * 1e6);
  // CannonReactor cr;
  // cr.run();

  PythonCannonDriver cd;
  cd.launch();
  cd.fire();

  while(true)
    usleep(1e6);
  
  return 0;
}
