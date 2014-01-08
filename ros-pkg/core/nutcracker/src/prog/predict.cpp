#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <bag_of_tricks/glob.h>
#include <nutcracker/descriptor_pipeline.h>
#include <ros/assert.h>
#include <ros/package.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string video_path;
  string config_path;
  string weights_path;
  string output_dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("video,v", bpo::value(&video_path)->required(), "")
    ("weights,w", bpo::value(&weights_path)->required(), "")
    ("output,o",  bpo::value(&output_dir)->default_value(""), "")
    ("config,c", bpo::value(&config_path)->default_value(""), "")
    ("dump-images,d", "")
    ;

  p.add("video", 1);
  p.add("weights", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] VIDEO WEIGHTS" << endl; 
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Parse args.
  if(output_dir == "")
    output_dir = video_path + "-output";
  ROS_ASSERT(!bfs::exists(output_dir));
  //bfs::create_directory(output_dir);
  cout << "Using output directory of " << output_dir << endl;

  if(config_path == "")
    config_path = ros::package::getPath("nutcracker") + "/config/config.yml";
  cout << "Using config at " << config_path << endl;
  YAML::Node config = YAML::LoadFile(config_path);

  cout << "Using video at " << video_path << endl;
  cv::VideoCapture cap(video_path);
  if(!cap.isOpened()) {
    cout << "Failed to open " << video_path << endl;
    return 1;
  }

  cout << "Using weights at " << weights_path << endl;
  VectorXd weights;
  eigen_extensions::loadASCII(weights_path, &weights);
  cout << "  " << weights.transpose() << endl;
  
  // -- Main loop.
  DescriptorPipeline dp(config);
  cv::Mat3b img;
  for(int i = 0; i < 500; ++i) cap >> img;
  while(true) {
    cap >> img;
    VectorXd descriptors = dp.computeDescriptors(img);
    cout << descriptors.transpose() << endl;
    if(descriptors.rows() == 0)
      continue;
    cout << "Estimate: " << weights.dot(descriptors) << endl;
    cv::imshow("video", dp.cropped());
    cv::waitKey(0);
  }
  
  return 0;
}
