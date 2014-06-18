#include <jarvis/thermal_grabber.h>
#include <jarvis/tracker.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;

namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string td_path, therm_path, tform_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("td", bpo::value(&td_path)->required(), "TrackDataset path.")
    ("therm", bpo::value(&therm_path)->required(), "Thermal path.")
    ("transform", bpo::value(&tform_path)->required(), "Transform path.");

  p.add("td", 1);
  p.add("therm", 1);
  p.add("transform", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] TD THERM TRANSFORM" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Loading TrackDataset..." << endl;
  TrackDataset td;
  td.load(td_path);
  
  cout << "Initializing grabber..." << endl;
  const Dataset& lt = td[td.size()-1];
  ThermalGrabber grabber(therm_path, false,
			 boost::any_cast<Blob::ConstPtr>(td[0][0].raw())->wall_timestamp_.toNSec(),
			 boost::any_cast<Blob::ConstPtr>(lt[lt.size()-1].raw())->wall_timestamp_.toNSec());
  
  cout << "Loading transform..." << endl;
  Eigen::ArrayXd v;
  Eigen::MatrixXd m;
  eigen_extensions::loadASCII(tform_path, &m);
  v = Eigen::ArrayXd(m.rows());
  v.matrix() = m;
  Eigen::Affine3f transform;
  pcl::getTransformation(v(0), v(1), v(2), v(3), v(4), v(5), transform);
  
  int i = 0, j = 0;
  bool show_track_thermal = true;
  while(1){
    const Dataset& track = td[i];
    cout << "Track: " << i << " Frame: " << j << endl;;
    // Get RGBD data and time.
    const Blob& blob = *boost::any_cast<Blob::ConstPtr>(track[j].raw());
    cv::Mat3b img = blob.image();
    cv::Mat3b track_thermal(img.size(), cv::Vec3b(127, 127, 127));
    cv::Mat1f therm_img(img.size(), 0.f);
    blob.project();
    Cloud::ConstPtr cloud = blob.cloud_;
    uint64_t time = blob.wall_timestamp_.toNSec();
    // Get thermal data and time.
    int idx = grabber.nearestIndexAtTime(time);// 3*1e7);
    if(idx >= 0){ // if specify a max_diff to nearestIndexAtTime
      // Get thermal data and create the thermal image.
      cv::Mat1f therm_data = *grabber[idx];
      therm_img = (therm_data-min_vis_temp)/(max_vis_temp-min_vis_temp);
      // Get point temperatures and use to create the thermal track image.
      Cloud::Ptr trans_cloud(new Cloud);
      pcl::transformPointCloud(*cloud, *trans_cloud, transform);
      Cloud::ConstPtr const_trans_cloud = trans_cloud;
      vector<float> point_temps;
      getPointTemperatures(const_trans_cloud, therm_data, point_temps);
      thermalTrackImage(cloud, blob.indices_, point_temps, track_thermal);
      // Print timing info.
      uint64_t therm_time = grabber.time(idx);
      cout << "   RGB time: " << fixed << setprecision(16) << setw(16) << setfill('0') << time << endl;
      cout << " therm time: " << fixed << setprecision(16) << setw(16) << setfill('0') << therm_time << endl;
      cout << "       diff: " << (int)(time>therm_time ? 1e-6*(time-therm_time) : -1e-6*(therm_time-time)) 
	   << "ms" <<endl;
    }
    // Visualize.
    cv::resize(therm_img, therm_img, img.size());
    cv::imshow("Image", therm_img);
    if(show_track_thermal){
      cv::imshow("Track", track_thermal);
    }
    else{
      cv::imshow("Track", img);
    }
    char c = cv::waitKey(0);
    if(c == 'q'){
      return 0;
    }
    else if(c == 'c'){
      show_track_thermal = !show_track_thermal;
    }
    else if(c == 'j' || c == 'J'){
      j = std::max(0, j - ((c == 'j') ? 1 : 100));
    }
    else if(c == 'k' || c == 'K'){
      j = std::min((int)td[i].size()-1, j + ((c == 'k') ? 1 : 100));
    }
    else if(c == 'd' || c == '['){
      i = std::max(0, i-1); j = 0;
    }
    else if(c == 'f' || c == ']'){
      i = std::min((int)td.size()-1, i+1); j = 0;
    }
  }

  return 0;
}
