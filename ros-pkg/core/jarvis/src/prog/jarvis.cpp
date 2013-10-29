#include <jarvis/jarvis.h>
#include <boost/program_options.hpp>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Jarvis");
  
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  int vis_level;
  int rotation;
  string output_directory;
  string config_path;
  string gc_path;
  string up_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("vis-level,v", bpo::value(&vis_level)->default_value(0), "")
    ("rotation,r", bpo::value(&rotation)->default_value(0), "")
    ("record", bpo::value(&output_directory)->default_value(""), "Where to save TD files")
    ("config", bpo::value(&config_path)->default_value(""), "")
    ("classifier,c", bpo::value(&gc_path)->default_value(""), "")
    ("up,u", bpo::value(&up_path), "")
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
 
  Jarvis jarvis(vis_level, rotation, output_directory);
  if(output_directory != "")
    cout << "Saving TD files to \"" << output_directory << "\"" << endl;

  if(opts.count("up")) {
    VectorXf up;
    cout << "Setting up vector to that found at " << up_path << endl;
    eigen_extensions::loadASCII(up_path, &up);
    jarvis.dp_->setUpVector(up);
  }
  
  
  // -- If we're going to classify things...
  if(gc_path != "") {
    // Load the DescriptorPipeline.
    jarvis.dp_ = DescriptorPipeline::Ptr(new DescriptorPipeline);
    if(config_path == "") {
      cout << "Using default descriptor pipeline." << endl;
      jarvis.dp_->initializeWithDefault();
    }
    else {
      cout << "Loading descriptor pipeline specification at \"" << config_path << "\"." << endl;
      YAML::Node config = YAML::LoadFile(config_path);
      ROS_ASSERT(config["Pipeline"]);
      jarvis.dp_->initialize(config["Pipeline"]);
    }
    // Load the classifier.
    cout << "Loading classifier at \"" << gc_path << "\"." << endl;
    jarvis.gc_ = GridClassifier::Ptr(new GridClassifier);
    jarvis.gc_->load(gc_path);
  }

  // -- Run.
  ros::spin();

  // -- Save any remaining tracks.
  jarvis.flush();
  
  return 0;
}
