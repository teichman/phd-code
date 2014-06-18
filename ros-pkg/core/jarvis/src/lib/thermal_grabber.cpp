#include <jarvis/thermal_grabber.h>
#include <eigen_extensions/eigen_extensions.h>
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;
namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;

bool reprojectPointThermal(const Eigen::Vector3f &pt, int &u, int &v)
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

ThermalGrabber::ThermalGrabber(const std::string& input_dir, bool in_memory,
			       uint64_t min_time, uint64_t max_time)
{
  bpt::ptime epoch(boost::gregorian::date(1970,1,1));
  // Load and sort filenames / times.
  bfs::directory_iterator date_it(input_dir), date_eod;
  BOOST_FOREACH(bfs::path const & date_p, make_pair(date_it, date_eod)){ // day
    if(date_p.leaf().string() == "transform.eig.txt")
      continue;
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


