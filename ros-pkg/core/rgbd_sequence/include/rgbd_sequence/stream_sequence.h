#ifndef STREAM_SEQUENCE_H
#define STREAM_SEQUENCE_H

#include <Eigen/Eigen>
#include <rgbd_sequence/rgbd_sequence.h>

namespace rgbd
{

  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> Cloud;
  typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> DepthMat;
  typedef boost::shared_ptr<DepthMat> DepthMatPtr;
  typedef boost::shared_ptr<const DepthMat> DepthMatConstPtr;
  using cv::Mat3b;
  using cv::Mat1w;

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

    double fx_,fy_,cx_,cy_; //Calibration parameters
    bool initialized_calibration_;

    StreamSequence();
    StreamSequence(const std::string& save_dir);
    //! Deep-copies.
    //TODO StreamSequence(const StreamSequence& seq);
    //! Deep-copies.
    // TODO StreamSequence& operator=(const StreamSequence& seq);
    // ! This changes the save_dir and saves everything to it
    void save(const std::string& filename);
    void load(const std::string& filename);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    size_t size() const;
    Cloud::Ptr getCloud(size_t frame, double f = 0) const;
    Mat3b getImage(size_t frame) const;
    //! dt is signed.
    Cloud::Ptr getCloud(double timestamp, double* dt) const;
    //! dt is signed.
    Mat3b getImage(double timestamp, double* dt) const;
    Mat1w getDepthRaw(size_t frame) const;
    Mat1w getDepthRaw(size_t frame, double &fx, double &fy, double &cx, double &cy) const;
    void getIntrinsics(size_t frame, double &fx, double &fy, double &cx, double &cy) const;
    void addFrame( const Mat3b &img, const DepthMat &depth, 
        double fx, double fy, double cx, double cy, double timestamp);
    //! Adds dt to all timestamps.
    void applyTimeOffset(double dt);

  protected:
    void saveFrame(const std::string& dir, size_t frame, const Mat3b &img, 
        const DepthMat &depth, double fx, double fy, double cx, double cy, 
        double timestamp);
    void loadImage(const std::string& dir, size_t frame, Mat3b &img) const;
    void loadDepth(const std::string& dir, size_t frame, 
        DepthMat &depth, double &fx, double &fy, double &cx, double &cy, 
        double &timestamp) const;
    //! dt is signed.
    size_t seek(double timestamp, double* dt) const;
  };


  inline bool isFinite(const Point& pt)
  {
    return (pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z));
  }

}

#endif // STREAM_SEQUENCE_H


