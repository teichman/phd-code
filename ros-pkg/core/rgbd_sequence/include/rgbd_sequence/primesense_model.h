#ifndef PRIMESENSE_MODEL_H
#define PRIMESENSE_MODEL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>

namespace rgbd
{

  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> Cloud;
  // PCL and OpenCV are row-major, but lots of work is required to make this change.
  // eigen_extensions serialization and re-collection of data, probably.
  //typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DepthMat;
  typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> DepthMat;  
  typedef boost::shared_ptr<DepthMat> DepthMatPtr;
  typedef boost::shared_ptr<const DepthMat> DepthMatConstPtr;
  
  //! "Projective" point comes from the OpenNI terminology, and refers to (u, v, z), i.e.
  //! pixel id and depth value.  Here I've added color, too, so that this represents everything
  //! that is known about a pixel in an RBGD camera.
  class ProjectivePoint : public Serializable
  {
  public:
    int u_;
    int v_;
    //! In millimeters, same as the raw depth image from the primesense device.
    unsigned short z_;
    unsigned char r_;
    unsigned char g_;
    unsigned char b_;

    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  };
  
  class Frame
  {
  public:
    DepthMatPtr depth_;
    cv::Mat3b img_;
    double timestamp_;

    cv::Mat3b depthImage() const;

  protected:
    cv::Vec3b colorize(double depth, double min_range, double max_range) const;
  };
  
  class PrimeSenseModel : public Serializable
  {
  public:
    //! "xpl" or "kinect"
    std::string type_;
    //! Device identifier.  Would use serial number but Asus didn't bother to provide one.
    int id_;
    int width_;
    int height_;
    double cx_;
    double cy_;
    //! Depth distortion model.  weights_[0] is a constant offset, weights_[1] is the raw depth measurement.
    Eigen::VectorXd weights_;
    double fx_;
    double fy_;

    // void setFocalLength(double f) { fx_ = f; fy_ = f; fx_inv_ = 1 / f; fy_inv_ = 1 / f; }
    // double fx() const { return fx_; }
    // double fy() const { return fy_; }
        
    //! Initializes with a bogus model: all params set to -1.
    PrimeSenseModel();
    //! max_range in meters
    //! Does *not* apply the depth distortion model.
    //! The only way to apply the depth distortion model is to call undistort.
    void frameToCloud(const Frame& frame, Cloud* pcd,
		      double max_range = std::numeric_limits<double>::max()) const;
    void cloudToFrame(const Cloud& pcd, Frame* frame) const;
    //! Applies depth distortion model to the depth data in frame.
    void undistort(Frame* frame) const;
    
    void project(const ProjectivePoint& ppt, Point* pt) const;
    void project(const Point& pt, ProjectivePoint* ppt) const;

    bool initialized() const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    std::string status(const std::string& prefix = "") const;
    bool hasDepthDistortionModel() const;
    void resetDepthDistortionModel();
    //! Returns type_ and id_ formatted nicely together.
    std::string name() const;

    //! f[1] is measured depth in decameters.
    Eigen::VectorXd computeFeatures(const ProjectivePoint& ppt) const;
    int numFeatures() const;

  protected:
    // double fx_inv_;
    // double fy_inv_;

    Eigen::VectorXd computeFeaturesMU(const ProjectivePoint& ppt) const;
    Eigen::VectorXd computeFeaturesMUV(const ProjectivePoint& ppt) const;
  };
  

}

#endif // PRIMESENSE_MODEL_H
