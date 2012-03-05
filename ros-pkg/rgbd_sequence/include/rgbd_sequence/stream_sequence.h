#ifndef STREAM_SEQUENCE_H
#define STREAM_SEQUENCE_H

#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <serializable/serializable.h>
#include <Eigen/Eigen>

namespace rgbd
{

  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> Cloud;
  typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> DepthMat;
  using cv::Mat3b;

  class StreamSequence : public Serializable
  {
  public:
    typedef boost::shared_ptr<StreamSequence> Ptr;
    typedef boost::shared_ptr<const StreamSequence> ConstPtr;
  
    std::vector<std::string> img_names_;
    std::vector<std::string> dpt_names_;
    std::vector<std::string> clk_names_;
    std::vector<double> timestamps_; //Keep these in memory
    std::string save_dir_;

    StreamSequence();
    StreamSequence(const std::string& save_dir);
    //! Deep-copies.
    //TODO StreamSequence(const StreamSequence& seq);
    //! Deep-copies.
    // TODO StreamSequence& operator=(const StreamSequence& seq);
    // ! This changes the save_dir and saves everything to it
    void save(const std::string& filename) const;
    void load(const std::string& filename);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    size_t size() const { ROS_ASSERT(img_names_.size() == dpt_names_.size()); return img_names_.size(); }
    Cloud::Ptr getCloud(size_t frame) const;
    Mat3b getImage(size_t frame) const;
    Cloud::Ptr getCloud(double timestamp, double* dt) const;
    Mat3b getImage(double timestamp, double* dt) const;
    void addFrame( const Mat3b &img, const DepthMat &depth, 
        double focal_length, double timestamp);

  protected:
    void saveFrame(const std::string& dir, size_t frame, const Mat3b &img, 
        const DepthMat &depth, double focal_length, double timestamp);
    void loadImage(const std::string& dir, size_t frame, Mat3b &img) const;
    void loadDepth(const std::string& dir, size_t frame, 
        DepthMat &depth, double &focal_length, double &timestamp) const;
    size_t seek(double timestamp, double* dt) const;
  };


}

#endif // STREAM_SEQUENCE_H


