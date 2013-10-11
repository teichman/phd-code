#include <pipeline/pipeline.h>
#include <boost/program_options.hpp>

using namespace std;
using namespace pl;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string config_path;
  string output_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("config", bpo::value(&config_path)->required(), "YAML that contains a Pipeline node at the root level.")
    ("output,o", bpo::value(&output_path)->required(), "Where to save the graphviz.")
    ;

  p.add("config", 1);
  p.add("output", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] CONFIG OUTPUT" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Loading config at " << config_path << endl;
  YAML::Node config = YAML::LoadFile(config_path);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);
  pl.writeGraphviz(output_path);
  cout << "Saved graphviz to " << output_path << endl;

  return 0;
}
