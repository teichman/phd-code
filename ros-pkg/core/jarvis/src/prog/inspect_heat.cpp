#include <jarvis/tracker.h>
#include <eigen_extensions/eigen_extensions.h>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace Eigen;

namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;

namespace cv{
  typedef boost::shared_ptr<Mat1f> Mat1fPtr;
  typedef boost::shared_ptr<const Mat1f> Mat1fConstPtr;
}

typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> Mat1b;

const float min_temp = 50.f, max_temp = 90.f;

void thermalDataToTemperature(const Mat1b& in, cv::Mat1f& out)
{
  const float min_temp = 0.f, max_temp = 128.f;
  out = cv::Mat1f(in.rows(), in.cols());
  for(int y = 0; y < out.rows; y++){
    for(int x = 0; x < out.cols; x++){
      out(y, x) = in(y, x)/255.f * (max_temp-min_temp) + min_temp;
    }
  }
}


bool filepathToTime(const std::string &filepath, uint64_t &timestamp)
{
  // For now, we assume the file is of the form frame_[22-char POSIX timestamp]_*         
  char timestamp_str[256];
  int result = std::sscanf(boost::filesystem::basename (filepath).c_str (),
			   "frame_%22s_%*s", timestamp_str);
  if(result > 0){
    // Convert to uint64_t, microseconds since 1970-01-01.
    bpt::ptime cur_date = bpt::from_iso_string (timestamp_str);
    bpt::ptime zero_date(boost::gregorian::date(1970, boost::gregorian::Jan, 1));
    timestamp = (cur_date - zero_date).total_microseconds ();
    return true;
  }
  return false;
}

class ThermalGrabber
{
public:
  ThermalGrabber(){}
  ThermalGrabber(const std::string& input, bool in_memory=false,
		 uint64_t min_time=0,
		 uint64_t max_time=std::numeric_limits<uint64_t>::max());
  size_t size() const{ return times_.size(); }
  cv::Mat1fConstPtr operator[](int i) const;
  uint64_t time(int i) const{ return times_[i]; }
  int nearestIndexAtTime(uint64_t time) const;
  void loadIntoMemory(int i);
  void removeFromMemory(int i);
protected:
  std::vector<std::string> file_names_;
  std::vector<uint64_t> times_;
  std::vector<bool> in_memory_;
  std::vector<cv::Mat1fPtr> imgs_;
};

ThermalGrabber::ThermalGrabber(const std::string& input, bool in_memory,
			       uint64_t min_time, uint64_t max_time)
{
  bpt::ptime epoch(boost::gregorian::date(1970,1,1));
  // Load and sort filenames / times.
  bfs::directory_iterator date_it(input), date_eod;
  BOOST_FOREACH(bfs::path const & date_p, make_pair(date_it, date_eod)){
    bfs::directory_iterator hour_it(date_p.string()), hour_eod;
    BOOST_FOREACH(bfs::path const & hour_p, make_pair(hour_it, hour_eod)){
      std::string yyyymodd = date_p.string().substr(date_p.string().size()-8, 8);
      std::string hh = hour_p.string().substr(hour_p.string().size()-2, 2);
      ostringstream start_oss, end_oss;
      start_oss << yyyymodd << "T" << setw(2) << setfill('0') << atoi(hh.c_str())-1 << "5959";
      end_oss   << yyyymodd << "T" << setw(2) << setfill('0') << atoi(hh.c_str())+1 << "0000";
      uint64_t start_time = (bpt::from_iso_string(start_oss.str()) - epoch).total_nanoseconds();
      uint64_t end_time = (bpt::from_iso_string(end_oss.str()) - epoch).total_nanoseconds();
      if(min_time <= end_time && max_time >= start_time){
	bfs::directory_iterator it(hour_p.string()), eod;
	BOOST_FOREACH(bfs::path const & p, make_pair(it, eod)){
	  uint64_t timestamp;
	  if(filepathToTime(p.string(), timestamp)){
	    file_names_.push_back(p.string());
	    times_.push_back(timestamp*1000ull); // convert to nanoseconds
	  }
	}
      }
    }
  }
  std::sort(file_names_.begin(), file_names_.end());
  std::sort(times_.begin(), times_.end());
  // Initialize / load (if in_memory) data.
  in_memory_.resize(times_.size(), in_memory);
  imgs_.resize(times_.size());
  if(in_memory){
    for(size_t i = 0; i < times_.size(); i++){
      loadIntoMemory(i);
    }
  }
}

cv::Mat1fConstPtr ThermalGrabber::operator[](int i) const
{
  if(in_memory_[i]){
    return imgs_[i];
  }
  else{
    Mat1b m;
    eigen_extensions::load(file_names_[i], &m);
    cv::Mat1fPtr img(new cv::Mat1f);
    thermalDataToTemperature(m, *img);
    return img;
  }
}

int ThermalGrabber::nearestIndexAtTime(uint64_t time) const
{
  std::vector<uint64_t>::const_iterator low;
  low = std::lower_bound(times_.begin(), times_.end(), time);
  int idx = std::max(0, (int)(low-times_.begin())-1);
  if(idx == (int)times_.size()-1){
    return idx;
  }
  else{
    double diff_lw = time-times_[idx];
    double diff_up = times_[idx+1]-time;
    return diff_lw<diff_up ? idx : idx+1;
  }
}

void ThermalGrabber::loadIntoMemory(int i)
{
  imgs_[i] = cv::Mat1fPtr(new cv::Mat1f);
  Mat1b m;
  eigen_extensions::load(file_names_[i], &m);
  thermalDataToTemperature(m, *imgs_[i]);
  in_memory_[i] = true;
}

void ThermalGrabber::removeFromMemory(int i)
{
  imgs_[i].reset();
  in_memory_[i] = false;
}

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB_t;
typedef boost::shared_ptr<CloudXYZRGB_t>  CloudXYZRGBPtr_t;
typedef boost::shared_ptr<const CloudXYZRGB_t>  CloudXYZRGBConstPtr_t;

const double THERMAL_PITCH = 25e-6; // 25 microns                                           
const double THERMAL_FOCAL_LENGTH = 9.66e-3; // 9.66 millimeters                            
const double THERMAL_SCALED_FOCAL_LENGTH = THERMAL_FOCAL_LENGTH / THERMAL_PITCH;
const int THERMAL_WIDTH = 320, THERMAL_HEIGHT = 240;
const double THERMAL_CENTER_X = THERMAL_WIDTH/2.0, THERMAL_CENTER_Y = THERMAL_HEIGHT/2.0;

bool reprojectPoint(const Eigen::Vector3f &pt, int &u, int &v,
                    int width, int height, double scaled_focal_length,
                    double center_x, double center_y)
{
  u = scaled_focal_length * pt(0) / pt(2) + center_x;
  v = scaled_focal_length * pt(1) / pt(2) + center_y;
  return (pt(2) > 0 && u >= 0 && u < width && v >= 0 && v < height);
}

bool reprojectPointThermal(const Eigen::Vector3f &pt, int &u, int &v)
{
  return reprojectPoint(pt, u, v, THERMAL_WIDTH, THERMAL_HEIGHT,
                        THERMAL_SCALED_FOCAL_LENGTH, THERMAL_CENTER_X, THERMAL_CENTER_Y);
}

template <typename T>
void reprojectCloudThermal(boost::shared_ptr<const pcl::PointCloud<T> > cloud,
                           boost::shared_ptr<pcl::PointCloud<T> > organized_cloud)
{
  reprojectCloud(cloud, organized_cloud, THERMAL_WIDTH, THERMAL_HEIGHT,
                 THERMAL_SCALED_FOCAL_LENGTH, THERMAL_CENTER_X, THERMAL_CENTER_Y);
}

template <typename T>
void reprojectCloud(boost::shared_ptr<const pcl::PointCloud<T> > cloud,
                    boost::shared_ptr<pcl::PointCloud<T> > organized_cloud,
                    int width, int height, double scaled_focal_length,
                    double center_x, double center_y)
{
  organized_cloud->clear();
  T nan_pt; nan_pt.x = nan_pt.y = nan_pt.z = std::numeric_limits<float>::quiet_NaN();
  organized_cloud->resize(height*width);
  for(size_t i = 0; i < organized_cloud->size(); i++){
    organized_cloud->at(i) = nan_pt;
  }
  organized_cloud->width  = width;
  organized_cloud->height = height;
  int u, v;
  for(size_t i = 0; i < cloud->size(); i++){
    const T& pt = cloud->at(i);
    if(reprojectPoint(pt.getVector3fMap(), u, v, width, height, scaled_focal_length,
                      center_x, center_y)){
      T& opt = (*organized_cloud)(u, v);
      if(::isnan(opt.z) || pt.z < opt.z){
        opt = pt;
      }
    }
  }
}

void getThermalColoredFg(CloudXYZRGBConstPtr_t cloud,
			 const std::vector<uint32_t>& indices,
			 const cv::Mat1f& thermal,
			 const Eigen::Affine3f& transform,
			 cv::Mat3b& output,
			 float alpha = 0.8)
{
  CloudXYZRGBPtr_t trans_cloud(new CloudXYZRGB_t), org_trans_cloud(new CloudXYZRGB_t);
  pcl::transformPointCloud(*cloud, *trans_cloud, transform);
  CloudXYZRGBConstPtr_t trans_const_cloud = trans_cloud;
  reprojectCloudThermal(trans_const_cloud, org_trans_cloud);
  for(size_t i = 0; i < trans_cloud->size(); i++){
    int y = indices[i] / output.cols, x = indices[i]%output.cols;
    const pcl::PointXYZRGB& pt = trans_cloud->at(i);
    int u, v;
    if( reprojectPointThermal(pt.getVector3fMap(), u, v)
	&& fabs(pt.z-(*org_trans_cloud)(u, v).z < 0.1)){
      int nv = v * 1.f * thermal.rows / THERMAL_HEIGHT,
	nu = u * 1.f * thermal.cols / THERMAL_WIDTH;
      float f = (thermal(nv, nu)-min_temp)/(max_temp-min_temp);
      output(y, x) = (1-alpha)*output(y, x) + alpha*f * cv::Vec3b(255, 255, 255);
    }
  }
}

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
    cout << "Usage: " << argv[0] << " [OPTS] TD" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Loading TrackDataset..." << endl;
  TrackDataset td;
  td.load(td_path);
  cout << "Done." << endl;
  
  cout << "Initializing grabber..." << endl;
  const Dataset& lt = td[td.size()-1];
  ThermalGrabber grabber(therm_path, false,
			 boost::any_cast<Blob::ConstPtr>(td[0][0].raw())->wall_timestamp_.toNSec(),
			 boost::any_cast<Blob::ConstPtr>(lt[lt.size()-1].raw())->wall_timestamp_.toNSec());
  cout << "Done." << endl;
  
  Eigen::ArrayXd v;
  Eigen::MatrixXd m;
  eigen_extensions::loadASCII(tform_path, &m);
  v = Eigen::ArrayXd(m.rows());
  v.matrix() = m;
  Eigen::Affine3f transform;
  pcl::getTransformation(v(0), v(1), v(2), v(3), v(4), v(5), transform);
  
  int i = 0, j = 0;
  float alpha = 0.f;
  while(1){
    const Dataset& track = td[i];
    cout << "Track: " << i << " Frame: " << j << endl;;
    // Get RGBD data and time.
    const Blob& blob = *boost::any_cast<Blob::ConstPtr>(track[j].raw());
    cv::Mat3b img = blob.image();
    blob.project();
    CloudXYZRGBConstPtr_t cloud = blob.cloud_, therm_cloud(new CloudXYZRGB_t);    
    uint64_t time = blob.wall_timestamp_.toNSec();
    // Get thermal data and time.
    int idx = grabber.nearestIndexAtTime(time);
    uint64_t therm_time = grabber.time(idx);
    cv::Mat1f therm_data = *grabber[idx];
    cv::Mat1f therm_img = (therm_data-min_temp)/(max_temp-min_temp);
    // Get thermal colored foreground.
    cv::Mat3b therm_colored_fg = 1*img;
    getThermalColoredFg(cloud, blob.indices_, therm_data, transform, therm_colored_fg, alpha);
    // Print info and visualize.
    cout << "   RGB time: " << fixed << setprecision(16) << setw(16) << setfill('0') << time << endl;
    cout << " therm time: " << fixed << setprecision(16) << setw(16) << setfill('0') << therm_time << endl;
    cout << "       diff: " << (int)(time>therm_time ? 1e-6*(time-therm_time) : -1e-6*(therm_time-time)) 
	 << "ms" <<endl;
    cv::imshow("RgbImage", img);
    cv::imshow("ThermalImage", therm_img);
    cv::imshow("ThermalFg", therm_colored_fg);
    char c = cv::waitKey(0);
    if(c == 'q'){
      return 0;
    }
    else if(c == 'a'){
      alpha = alpha > 0.5 ? 0.f : 1.f;
    }
    else if(c == 'j' || c == 'J'){
      j = std::max(0, j - ((c == 'j') ? 1 : 100));
    }
    else if(c == 'k' || c == 'K'){
      j = std::min((int)td[i].size()-1, j + ((c == 'k') ? 1 : 100));
    }
    else if(c == 'd'){
      i = std::max(0, i-1); j = 0;
    }
    else if(c == 'f'){
      i = std::min((int)td.size()-1, i+1); j = 0;
    }
  }

  return 0;
}
