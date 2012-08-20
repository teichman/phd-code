#include <boost/program_options.hpp>
#include <rgbd_sequence/stream_sequence.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace rgbd;
namespace bpo = boost::program_options;

int main(int argc, char** argv)
{
  string dir;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("distortion-model", bpo::value<string>(), "Use pre-computed distortion model")
    ("seq", bpo::value<string>(&dir), "Sequence directory")
    ("only-stats", "Only print stats and exit")
    ;

  bpo::positional_options_description p;
  p.add("seq", 1);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);
  
  cout << "Looking at dir: " << dir << endl;
  StreamSequence seq;
  seq.load(dir);
  cout << "Loaded successfully" << endl;

  double mean_dt = 0;
  double max_dt = -std::numeric_limits<double>::max();
  for(size_t i = 1; i < seq.size(); i++) {
    double dt = seq.timestamps_[i] - seq.timestamps_[i-1];
    mean_dt += dt;
    if(dt > max_dt)
      max_dt = dt;
  }
  mean_dt /= (double)seq.size();
  cout << "--------------------" << endl;
  cout << "Mean fps: " << (double)seq.size() / (seq.timestamps_.back() - seq.timestamps_.front()) << endl;
  cout << "Mean dt: " << mean_dt << endl;
  cout << "Max dt: " << max_dt << endl;
  cout << "--------------------" << endl;

  if(opts.count("only-stats"))
    return 0;
  
  pcl::visualization::CloudViewer cloud_viewer("cloud");
  cv::namedWindow("image");
  for(size_t i = 0; i < seq.size(); i++){
    cout << "Viewing cloud: " << i << endl;
    Cloud::Ptr cloud = seq.getCloud(i);
    cloud_viewer.showCloud(cloud);
    cv::imshow("image", seq.getImage(i));
    if(i < seq.size() ){
      double dt = seq.timestamps_[i+1]-seq.timestamps_[i];
      cout << "dt: " << dt << endl;
      cv::waitKey(dt * 1e3);
    }
  }
  
  return 0;
}


