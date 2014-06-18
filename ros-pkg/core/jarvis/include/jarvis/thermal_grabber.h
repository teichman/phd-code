#ifndef THERMAL_GRABBER_H
#define THERMAL_GRABBER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <pcl/common/transforms.h>
#include <vector>

namespace cv {
  typedef boost::shared_ptr<Mat1f> Mat1fPtr;
  typedef boost::shared_ptr<const Mat1f> Mat1fConstPtr;
}

namespace Eigen {
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;
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

const float min_vis_temp = 50.f, max_vis_temp = 90.f;
// Camera parameters
const double THERMAL_PITCH = 25e-6; // 25 microns                                           
const double THERMAL_FOCAL_LENGTH = 9.66e-3; // 9.66 millimeters                            
const double THERMAL_SCALED_FOCAL_LENGTH = THERMAL_FOCAL_LENGTH / THERMAL_PITCH;
const int THERMAL_WIDTH = 320, THERMAL_HEIGHT = 240;
const double THERMAL_CENTER_X = THERMAL_WIDTH/2.0, THERMAL_CENTER_Y = THERMAL_HEIGHT/2.0;

bool reprojectPointThermal(const Eigen::Vector3f &pt, int &u, int &v);

template <typename T>
void reprojectCloudThermal(boost::shared_ptr<const pcl::PointCloud<T> > cloud,
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
  std::vector<cv::Point> img_pts;
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


#endif // THERMAL_GRABBER_H
