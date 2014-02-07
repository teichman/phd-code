#include <sentinel/sentinel.h>
#include <jarvis/compression_helpers.h>

struct H264Serializer
{
  EncodingOptions opts_;

  H264Serializer()
  {
    // opts_.fps_ = 30;
    // opts_.crf_ = 13;
  }
  
  void operator()(std::vector<cv::Mat3b> images, std::string path)
  {
    std::vector<uint8_t> blob;
    encodeH264Shm(opts_, images, &blob);
    writeBlob(blob, path);
  }
};

class VideoStreamingSentinel : public Sentinel
{
public:
  VideoStreamingSentinel(double update_interval,
                         double occupancy_threshold,
                         int raytracing_threshold,
                         double detection_threshold,
                         bool visualize,
                         OpenNI2Interface::Resolution color_res,
                         OpenNI2Interface::Resolution depth_res);

  ~VideoStreamingSentinel();
  
protected:
  ThreadedSerializer<std::vector<cv::Mat3b>, H264Serializer> serializer_;
  std::vector<cv::Mat3b> images_;
  int timeout_;
  
  void handleDetection(openni::VideoFrameRef color,
                       openni::VideoFrameRef depth,
                       const std::vector<uint32_t>& indices,
                       const std::vector<uint32_t>& fg_markers,
                       const std::vector<uint32_t>& bg_fringe_markers,
                       double sensor_timestamp,
                       double wall_timestamp,
                       size_t frame_id);
  
  void handleNonDetection(openni::VideoFrameRef color,
                          openni::VideoFrameRef depth,
                          double sensor_timestamp,
                          double wall_timestamp,
                          size_t frame_id);
  
  //! Used for processing recordings.
  void processHook(openni::VideoFrameRef color, openni::VideoFrameRef depth, double wall_timestamp);
};

