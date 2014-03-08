#include <boost/program_options.hpp>
#include <jarvis/descriptor_pipeline.h>
#include <online_learning/dataset.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;
using namespace pl;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string config_path;
  vector<string> td_paths;
  string out_descriptoronly_td;
  int num_threads;
  string up_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("config", bpo::value<string>(&config_path)->default_value(DescriptorPipeline::defaultSpecificationPath()), "")
    ("tds", bpo::value< vector<string> >(&td_paths)->required()->multitoken(), "")
    ("debug", "")
    ("out-descriptoronly-td",
     bpo::value<string>(&out_descriptoronly_td)->default_value(""))
    ("num-threads,j", bpo::value(&num_threads)->default_value(1))
    ("up,u", bpo::value(&up_path), "")
    ("force,f", "")
    ("strip", "Removes descriptors from TDs.")
    ("randomize", "")
    ;

  p.add("tds", -1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] TDS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  if(opts.count("randomize")) {
    cout << "Setting random seed to something random." << endl;
    srand(time(NULL));
  }

  // -- If stripping descriptors, just do this and return.
  if(opts.count("strip")) {
    cout << "Stripping descriptors..." << endl;
    for(size_t i = 0; i < td_paths.size(); ++i) {
      cout << "Working on " << td_paths[i] << endl;
      TrackDataset td;
      td.load(td_paths[i]);
      
      // Apply an empty dmap to drop all descriptors.
      td.applyNameMapping("dmap", NameMapping());
      td.save(td_paths[i]);
    }
    return 0;
  }


  // -- Otherwise proceed with update.
  YAML::Node config = YAML::LoadFile(config_path);
  cout << "Using " << num_threads << " threads to update descriptors." << endl;
  cout << "Using pipeline defined at " << config_path << "." << endl;

  // -- Get the up vector.
  // If using less_gravity.yml, you need to put something here, but
  // it won't have any effect.  TODO: Add accelerometer, get rid of the annoying
  // cruft involving manually setting the up vector.
  VectorXf up = VectorXf::Ones(3);
  if(opts.count("up")) {
    cout << "Setting up vector to that found at " << up_path << endl;
    eigen_extensions::loadASCII(up_path, &up);
  }

  if (!out_descriptoronly_td.empty()) {
    CustomSerializer::Ptr original_serializer = Instance::custom_serializer_;
    CustomSerializer::Ptr reference_serializer = CustomSerializer::Ptr(
        new ReferenceSavingCustomSerializer());

    TrackDataset td;
    for(size_t i = 0; i < td_paths.size(); ++i) {
      cout << "Adding " << td_paths[i] << endl;
      Instance::custom_serializer_ = original_serializer;
      td.load(td_paths[i]);
      td.clearRaw();
    }
    // To force descriptor re-computation, apply an empty dmap,
    // which will drop all descriptors.
    if(opts.count("force")) {
      cout << "Forcing descriptor re-computation." << endl;
      td.applyNameMapping("dmap", NameMapping());
    }

    updateDescriptors(config["Pipeline"], num_threads, &td, up,
                      opts.count("debug"));
    Instance::custom_serializer_ = reference_serializer;
    td.save(out_descriptoronly_td);
  }
  else
  {
    for(size_t i = 0; i < td_paths.size(); ++i) {
      cout << "Working on " << td_paths[i] << endl;
      TrackDataset td;
      td.load(td_paths[i]);

      // To force descriptor re-computation, apply an empty dmap,
      // which will drop all descriptors.
      if(opts.count("force")) {
        cout << "Forcing descriptor re-computation." << endl;
        td.applyNameMapping("dmap", NameMapping());
      }

      double ms_per_obj = updateDescriptors(config["Pipeline"], num_threads, &td, up, opts.count("debug"));
      // Only save if something changed.
      if(ms_per_obj != 0) {
        // Serializable writes to a temporary file and then does a move,
        // so if you control-c you are guaranteed that at least one of
        // the files will still exist and be correct.
        td.save(td_paths[i]);
      }
    }
  }

  return 0;
}
