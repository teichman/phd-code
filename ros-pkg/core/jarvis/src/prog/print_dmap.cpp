#include <jarvis/descriptor_pipeline.h>
#include <boost/program_options.hpp>

using namespace std;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string config_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("config", bpo::value(&config_path)->required(), "Pipeline YML")
    ;

  p.add("config", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] CONFIG" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(config_path);
  DescriptorPipeline dp;
  dp.initialize(config["Pipeline"]);
  cout << dp.dmap() << endl;

  return 0;
}
