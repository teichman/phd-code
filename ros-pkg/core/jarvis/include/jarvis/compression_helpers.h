#include <stdint.h>
#include <stddef.h>
#include <vector>
#include <opencv2/core/core.hpp>

// Nasty shit needed to make libav happy.  TODO: Make this not gross.
// http://ethioprogrammer.blogspot.com/2012/09/uint64c-was-not-declared-in-this-scope.html
#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif

extern "C" {
  #include <libavcodec/avcodec.h>
}

class H264Encoder
{
public:
  H264Encoder(int fps, int bitrate);
  void initialize(int width, int height);
  void reserve(size_t num_bytes) { blob_.reserve(num_bytes); }
  bool initialized() const { return initialized_; }
  void addFrame(cv::Mat3b bgr);
  //! Flush delayed frames to blob_.
  //! Add final bytes to produce an actual video file.
  void finalize();
  //! Must be finalized.
  void save(const std::string& path) const;
  const std::vector<uint8_t>& blob() const { return blob_; }
  
protected:
  std::vector<uint8_t> blob_;
  int fps_;
  int bitrate_;

  AVCodec* codec_;
  AVCodecContext* ctx_;
  AVFrame* frame_;
  std::vector<uint8_t> frame_buffer_;
  std::vector<uint8_t> output_buffer_;
  bool finalized_;
  bool initialized_;
  int num_frames_;
  cv::Mat3b yuv_;
};

struct EncodingOptions
{
  int fps_;
  int crf_;

  EncodingOptions() :
    fps_(30),
    crf_(13)
  {
  }
};

void readBlob(const std::string& path, std::vector<uint8_t>* blob);
//! Warning: If the program terminates in the middle of this call, it will leave a turd in /dev/shm/encodeH264Shm-XXXXXX.
void encodeH264Shm(const EncodingOptions& opts, const std::vector<cv::Mat3b>& images, std::vector<uint8_t>* blob);
void decodeH264Shm(const std::vector<uint8_t>& blob, std::vector<cv::Mat3b>* images);

void writeToVec(size_t num, std::vector<uint8_t>* data);
size_t readFromVec(const std::vector<uint8_t>& data, size_t idx, size_t* num);
void writeToVec(const std::vector<uint8_t>& chunk, std::vector<uint8_t>* data);
size_t readFromVec(const std::vector<uint8_t>& data, size_t idx, std::vector<uint8_t>* chunk);
