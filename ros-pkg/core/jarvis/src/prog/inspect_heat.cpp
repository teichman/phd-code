#include <jarvis/tracker.h>
#include <eigen_extensions/eigen_extensions.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/common/transforms.h>

using namespace std;

namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;

namespace cv{
  typedef boost::shared_ptr<Mat1f> Mat1fPtr;
  typedef boost::shared_ptr<const Mat1f> Mat1fConstPtr;
}

namespace Eigen{
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;
}

const float min_vis_temp = 50.f, max_vis_temp = 90.f;

static void thermalDataToTemperature(const Eigen::MatrixXb& in, cv::Mat1f& out)
{
  const float min_temp = 0.f, max_temp = 128.f;
  out = cv::Mat1f(in.rows(), in.cols());
  for(int y = 0; y < out.rows; y++){
    for(int x = 0; x < out.cols; x++){
      out(y, x) = in(y, x)/255.f * (max_temp-min_temp) + min_temp;
    }
  }
}

static bool filepathToTime(const std::string &filepath, uint64_t &timestamp)
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

//! \brief Grabber for thermal data. Data is assumed to be stored in nested
//! directory (day then hour).
class ThermalGrabber
{
public:
  //! \brief Default constructor.
  ThermalGrabber(){}
  //! \brief Constructor that takes input directory, whether or not to store
  //! data in memory, and minimum and maximum times for grabber (in case
  //! only care about a small slice of a day).
  ThermalGrabber(const std::string& input_dir, bool in_memory=false,
		 uint64_t min_time=0,
		 uint64_t max_time=std::numeric_limits<uint64_t>::max());
  //! \brief Size() returns the number of samples recorded (images, times).
  size_t size() const{ return times_.size(); }
  //! \brief Returns a shared_ptr to the ith thermal image (absolute temps).
  cv::Mat1fConstPtr operator[](int i) const;
  //! \brief Returns the recording time of the ith thermal image.
  uint64_t time(int i) const{ return times_[i]; }
  //! \brief Returns the nearest thermal image index to the given time (in nanoseconds)
  //! for which the time difference is <= max_diff (-1 otherwise).
  //! max_diff=-1.0 means return no matter what.
  int nearestIndexAtTime(uint64_t time, 
			 double max_diff=-1.0) const;
  //! \brief Loads the ith thermal image into memory.
  void loadIntoMemory(int i);
  //! \brief Removes the ith thermal image from memory..
  void removeFromMemory(int i);
protected:
  //! \brief File names.
  std::vector<std::string> file_names_;
  //! \brief Thermal images.
  std::vector<cv::Mat1fPtr> imgs_;
  //! \brief Recording times for the thermal images.
  std::vector<uint64_t> times_;
  //! \brief Whether individual images are stored in memory (true) or read from disk (false).
  std::vector<bool> in_memory_;
};

ThermalGrabber::ThermalGrabber(const std::string& input_dir, bool in_memory,
			       uint64_t min_time, uint64_t max_time)
{
  bpt::ptime epoch(boost::gregorian::date(1970,1,1));
  // Load and sort filenames / times.
  bfs::directory_iterator date_it(input_dir), date_eod;
  BOOST_FOREACH(bfs::path const & date_p, make_pair(date_it, date_eod)){ // day
    bfs::directory_iterator hour_it(date_p.string()), hour_eod;
    BOOST_FOREACH(bfs::path const & hour_p, make_pair(hour_it, hour_eod)){ // hour
      // Get start and end time for this day & hour.
      string yyyymodd = date_p.string().substr(date_p.string().size()-8, 8);
      string hh = hour_p.string().substr(hour_p.string().size()-2, 2);
      ostringstream start_oss, end_oss;
      start_oss << yyyymodd << "T" << setw(2) << setfill('0') << atoi(hh.c_str())-1 << "5959";
      end_oss   << yyyymodd << "T" << setw(2) << setfill('0') << atoi(hh.c_str())+1 << "0000";
      uint64_t start_time = (bpt::from_iso_string(start_oss.str()) - epoch).total_nanoseconds();
      uint64_t end_time = (bpt::from_iso_string(end_oss.str()) - epoch).total_nanoseconds();
      // Store file name and time if the day overlaps with [min_time, max_time].
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
  // Allocte and initialize data members (load if in_memory=true).
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
  if(in_memory_[i]){ // return shared_ptr to stored image
    return imgs_[i];
  }
  else{ // load image and return shared_ptr.
    Eigen::MatrixXb m;
    eigen_extensions::load(file_names_[i], &m);
    cv::Mat1fPtr img(new cv::Mat1f);
    thermalDataToTemperature(m, *img);
    return img;
  }
}

int ThermalGrabber::nearestIndexAtTime(uint64_t time,
				       double max_diff) const
{
  if(!times_.size()){ // if no times, return -1 (no overlap with region, for example)
    return -1;
  }
  // Get index of last element less than time.
  vector<uint64_t>::const_iterator low;
  low = std::lower_bound(times_.begin(), times_.end(), time);
  int idx = std::max(0, (int)(low-times_.begin())-1);
  if(idx == (int)times_.size()-1){ // if last element, return if < max_diff.
    double diff = time-times_[idx];
    return (max_diff < 0 || diff <= max_diff) ? idx : -1;
  }
  else{ // otherwise return either idx or idx+1 (whichever is nearer) if < max_diff
    double diff_lw = time-times_[idx];
    double diff_up = times_[idx+1]-time;
    if(max_diff < 0 || diff_lw <= max_diff || diff_up <= max_diff){
      return diff_lw<diff_up ? idx : idx+1;
    }
    else{
      return -1;
    }
  }
}

void ThermalGrabber::loadIntoMemory(int i)
{
  if(!in_memory_[i]){
    imgs_[i] = cv::Mat1fPtr(new cv::Mat1f);
    Eigen::MatrixXb m;
    eigen_extensions::load(file_names_[i], &m);
    thermalDataToTemperature(m, *imgs_[i]);
    in_memory_[i] = true;
  }
}

void ThermalGrabber::removeFromMemory(int i)
{
  if(in_memory_[i]){
    imgs_[i].reset();
    in_memory_[i] = false;
  }
}

// Camera parameters
const double THERMAL_PITCH = 25e-6; // 25 microns                                           
const double THERMAL_FOCAL_LENGTH = 9.66e-3; // 9.66 millimeters                            
const double THERMAL_SCALED_FOCAL_LENGTH = THERMAL_FOCAL_LENGTH / THERMAL_PITCH;
const int THERMAL_WIDTH = 320, THERMAL_HEIGHT = 240;
const double THERMAL_CENTER_X = THERMAL_WIDTH/2.0, THERMAL_CENTER_Y = THERMAL_HEIGHT/2.0;

static bool reprojectPointThermal(const Eigen::Vector3f &pt, int &u, int &v)
{
  u = THERMAL_SCALED_FOCAL_LENGTH * pt(0) / pt(2) + THERMAL_CENTER_X;
  v = THERMAL_SCALED_FOCAL_LENGTH * pt(1) / pt(2) + THERMAL_CENTER_Y;
  if (pt(2) > 0 && u >= 0 && u < THERMAL_WIDTH && v >= 0 && v < THERMAL_HEIGHT){
    return true; // reprojected into thermal image.
  }
  else{ // reproject outside of image, so set coord = (-1, -1) and return false.
    u = -1; v = -1;
    return false;
  }
}

template <typename T>
static void reprojectCloudThermal(boost::shared_ptr<const pcl::PointCloud<T> > cloud,
				  const cv::Mat1f& thermal,
				  std::vector<cv::Point>& img_pts)
{
  img_pts.resize(cloud->size());
  for(size_t i = 0; i < cloud->size(); i++){
    if(reprojectPointThermal(cloud->at(i).getVector3fMap(), img_pts[i].x, img_pts[i].y)){
      // Scale reprojected points according to thermal image scale.
      img_pts[i].x *= 1.f*thermal.cols/THERMAL_WIDTH;
      img_pts[i].y *= 1.f*thermal.rows/THERMAL_HEIGHT;
    }
  }
}

template <typename T>
static void getPointTemperatures(boost::shared_ptr<const pcl::PointCloud<T> > cloud,
				 const cv::Mat1f& thermal,
				 std::vector<float>& point_temps)
{
  vector<cv::Point> img_pts;
  reprojectCloudThermal(cloud, thermal, img_pts);
  point_temps.resize(cloud->size(), std::numeric_limits<float>::quiet_NaN());
  for(size_t i = 0; i < cloud->size(); i++){
    if(img_pts[i].x >= 0){ // if reprojected into image
      point_temps[i] = thermal(img_pts[i]);
    }
  }  
}

template <typename T>
static void thermalTrackImage(boost::shared_ptr<const pcl::PointCloud<T> > cloud,
			      const std::vector<uint32_t>& indices,
			      const std::vector<float>& point_temps,
			      cv::Mat3b& output, cv::Vec3b missing_color=cv::Vec3b(0,0,255))
{
  for(size_t i = 0; i < cloud->size(); i++){
    int y = indices[i] / output.cols, x = indices[i]%output.cols;
    if(!::isnan(point_temps[i])){ // if valid, color based on temp
      float f = std::min(1.f, std::max(0.f, (point_temps[i]-min_vis_temp)/(max_vis_temp-min_vis_temp)));
      output(y, x) = f*cv::Vec3b(255, 255, 255);
    }
    else{// otherwise, colore with missing color
      output(y, x) = 1*missing_color;
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
    else if(c == 'd'){
      i = std::max(0, i-1); j = 0;
    }
    else if(c == 'f'){
      i = std::min((int)td.size()-1, i+1); j = 0;
    }
  }

  return 0;
}
