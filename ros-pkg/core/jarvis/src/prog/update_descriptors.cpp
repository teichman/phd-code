#include <boost/program_options.hpp>
#include <jarvis/descriptor_pipeline.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;
using namespace pl;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string config_path;
  vector<string> td_paths;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("config", bpo::value<string>(&config_path)->required(), "")
    ("tds", bpo::value< vector<string> >(&td_paths)->required()->multitoken(), "")
    ;

  p.add("config", 1);
  p.add("tds", -1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] CONFIG TDS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(config_path);
  cout << "Using " << NUM_THREADS << " threads to update descriptors." << endl;
  cout << "Using pipeline defined at " << config_path << "." << endl;

  for(size_t i = 0; i < td_paths.size(); ++i) {
    cout << "Working on " << td_paths[i] << endl;
    TrackDataset td;
    td.load(td_paths[i]);
    updateDescriptors(config["Pipeline"], NUM_THREADS, &td);

    // Serializable writes to a temporary file and then does a move,
    // so if you control-c you are guaranteed that at least one of
    // the files will still exist and be correct.
    td.save(td_paths[i]);
  }

  return 0;
}