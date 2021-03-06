#ifndef PRIMESENSE_MODEL_H
#define PRIMESENSE_MODEL_H

#include <ros/assert.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>
#include <ros/console.h>
#include <ros/assert.h>

// Depth multipliers, used in PrimeSenseModel::estimateMapDepth and
// all over the xpl_calibration package.
#define MAX_MULT 1.2
#define MIN_MULT 0.8

// Voxel grid size.
#define DEFAULT_VGSIZE 0.01

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
  // For storing z values in meters.  This is not Euclidean distance.
  typedef std::vector< std::vector< std::vector<double> > > RangeIndex;
  typedef Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic> IndexMap;
  
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
  
  class Frame : public Serializable
  {
  public:
    DepthMatPtr depth_;
    cv::Mat3b img_;
    double timestamp_;

    cv::Mat3b depthImage() const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    
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
    void cloudToFrame(const Cloud& pcd, Frame* frame, IndexMap* indexmap = NULL) const;
    void cloudToRangeIndex(const Cloud& pcd, RangeIndex* rindex) const;
    //! transform is applied to the map, then projected into a depth index.
    //! The best depth estimate from the map corresponding to the measurement depth frame
    //! will be returned.
    void estimateMapDepth(const Cloud& map, const Eigen::Affine3f& transform,
                          const Frame& measurement,
                          DepthMat* estimate) const;
                          
    //! Applies depth distortion model to the depth data in frame.
    void undistort(Frame* frame) const;
    
    void project(const ProjectivePoint& ppt, Point* pt) const;
    void project(const Point& pt, ProjectivePoint* ppt) const;

    bool initialized() const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    std::string status(const std::string& prefix = "") const;
    bool hasDepthDistortionModel() const;
    bool hasOldDefaultDepthDistortionModel() const;
    void resetDepthDistortionModel();
    //! Returns type_ and id_ formatted nicely together.
    std::string name() const;

    //! f[1] is measured depth in decameters.
    void computeFeatures(const ProjectivePoint& ppt, Eigen::VectorXd* features) const;
    int numFeatures() const;

  protected:
    // double fx_inv_;
    // double fy_inv_;

    //Eigen::VectorXd computeFeaturesMU(const ProjectivePoint& ppt) const;
    void computeFeaturesMUV(const ProjectivePoint& ppt, Eigen::VectorXd* features) const;
    bool coneFit(const DepthMat& naive_mapdepth, const RangeIndex& rindex,
                 int uc, int vc, double radius, double measurement_depth,
                 double* mean, double* stdev) const;
  };
    
}

#endif // PRIMESENSE_MODEL_H
