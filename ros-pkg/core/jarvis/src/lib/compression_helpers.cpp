#include <jarvis/compression_helpers.h>
#include <string.h>  // memcpy
#include <ros/assert.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <bag_of_tricks/glob.h>

using namespace std;
namespace bfs = boost::filesystem;

H264Encoder::H264Encoder(int fps, int bitrate) :
  fps_(fps),
  bitrate_(bitrate),
  finalized_(false),
  initialized_(false),
  num_frames_(0)
{
}

void H264Encoder::initialize(int width, int height)
{
  avcodec_init(); 
  avcodec_register_all();
  av_log_set_level(-1);

  codec_ = avcodec_find_encoder(CODEC_ID_H264);
  ROS_ASSERT(codec_);

  ctx_ = avcodec_alloc_context3(codec_);
  ROS_ASSERT(ctx_);

  ctx_->codec_id = (CodecID)CODEC_ID_H264;
  ctx_->width = width;
  ctx_->height = height;
  ctx_->time_base = (AVRational){1, fps_};
  ctx_->pix_fmt = PIX_FMT_YUV420P;
  ctx_->bit_rate = bitrate_;
  
  // See http://stackoverflow.com/questions/3553003/encoding-h-264-with-libavcodec-x264
  // I have no idea what most of these are.  They could probably be improved.
  // libx264-medium.ffpreset preset
  // ctx->coder_type = 1;  // coder = 1
  // ctx->flags|=CODEC_FLAG_LOOP_FILTER;   // flags=+loop
  // ctx->me_cmp|= 1;  // cmp=+chroma, where CHROMA = 1
  // ctx->partitions|=X264_PART_I8X8+X264_PART_I4X4+X264_PART_P8X8+X264_PART_B8X8; // partitions=+parti8x8+parti4x4+partp8x8+partb8x8
  // ctx->me_method=ME_HEX;    // me_method=hex
  // ctx->me_subpel_quality = 7;   // subq=7
  // ctx->me_range = 16;   // me_range=16
  // ctx->gop_size = 250;  // g=250
  // ctx->keyint_min = 25; // keyint_min=25
  // ctx->scenechange_threshold = 40;  // sc_threshold=40
  // ctx->i_quant_factor = 0.71; // i_qfactor=0.71
  // ctx->b_frame_strategy = 1;  // b_strategy=1
  // ctx->qcompress = 0.6; // qcomp=0.6
  // ctx->qmin = 10;   // qmin=10
  // ctx->qmax = 51;   // qmax=51
  // ctx->max_qdiff = 4;   // qdiff=4
  // ctx->max_b_frames = 3;    // bf=3
  // ctx->refs = 3;    // refs=3
  // ctx->directpred = 1;  // directpred=1
  // ctx->trellis = 1; // trellis=1
  // ctx->flags2|=CODEC_FLAG2_BPYRAMID+CODEC_FLAG2_MIXED_REFS+CODEC_FLAG2_WPRED+CODEC_FLAG2_8X8DCT+CODEC_FLAG2_FASTPSKIP;  // flags2=+bpyramid+mixed_refs+wpred+dct8x8+fastpskip
  // ctx->weighted_p_pred = 2; // wpredp=2
  // // libx264-main.ffpreset preset
  // ctx->flags2|=CODEC_FLAG2_8X8DCT;c->flags2^=CODEC_FLAG2_8X8DCT;    // flags2=-dct8x8
  
  ctx_->bit_rate_tolerance = 0;
  ctx_->rc_max_rate = 0;
  ctx_->rc_buffer_size = 0;
  ctx_->gop_size = 40;
  ctx_->max_b_frames = 3;
  ctx_->b_frame_strategy = 1;
  ctx_->coder_type = 1;
  ctx_->me_cmp = 1;
  ctx_->me_range = 16;
  ctx_->qmin = 10;
  ctx_->qmax = 51;
  ctx_->scenechange_threshold = 40;
  ctx_->flags |= CODEC_FLAG_LOOP_FILTER;
  ctx_->me_method = ME_HEX;
  ctx_->me_subpel_quality = 5;
  ctx_->i_quant_factor = 0.71;
  ctx_->qcompress = 0.6;
  ctx_->max_qdiff = 4;
  ctx_->directpred = 1;
  ctx_->flags2 |= CODEC_FLAG2_FASTPSKIP;

  int err = avcodec_open2(ctx_, codec_, NULL);
  if(err < 0) {
    cout << "Failed to open codec." << endl;
    cout << "Error: " << err << endl;
    ROS_ASSERT(0);
  }

  size_t num_pixels = width * height;
  frame_buffer_.resize(3 * num_pixels / 2);
  frame_ = avcodec_alloc_frame();
  frame_->data[0] = frame_buffer_.data();
  frame_->data[1] = frame_->data[0] + num_pixels;
  frame_->data[2] = frame_->data[1] + num_pixels / 4;
  frame_->linesize[0] = ctx_->width;
  frame_->linesize[1] = ctx_->width / 2;
  frame_->linesize[2] = ctx_->width / 2;
  //cout << "Line sizes: " << frame_->linesize[0] << " " << frame_->linesize[1] << " " << frame_->linesize[2] << endl;

  reserve(1e6);
  initialized_ = true;
}

void H264Encoder::addFrame(cv::Mat3b bgr)
{
  ROS_ASSERT(initialized_);
  size_t output_buffer_max_size = 1e6;
  
  // -- Encode each frame one at a time.
  //    TODO: It is probably better to use libav's conversion functions.
  cv::cvtColor(bgr, yuv_, cv::COLOR_BGR2YCrCb);
  for(int i = 0; i < bgr.rows * bgr.cols; ++i)
    frame_->data[0][i] = yuv_(i)[0];
  for(int y = 0; y < ctx_->height / 2; ++y) {
    for(int x = 0; x < ctx_->width / 2; ++x) { 
      frame_->data[1][y * frame_->linesize[1] + x] = yuv_(y*2, x*2)[2];
      frame_->data[2][y * frame_->linesize[2] + x] = yuv_(y*2, x*2)[1];
    }
  }

  // http://stackoverflow.com/questions/6603979/ffmpegavcodec-encode-video-setting-pts-h264
  // http://thompsonng.blogspot.com.au/2011/09/ffmpeg-avinterleavedwriteframe-return.html
  // But this doesn't seem to work...
  frame_->pts = (1.0 / fps_) * 90 * num_frames_;

  output_buffer_.resize(output_buffer_max_size);
  size_t out_size = avcodec_encode_video(ctx_, output_buffer_.data(), output_buffer_.size(), frame_);
  ROS_ASSERT(out_size < output_buffer_.size());
  output_buffer_.resize(out_size);
  blob_.insert(blob_.end(), output_buffer_.begin(), output_buffer_.end());

  ++num_frames_;
}

void H264Encoder::finalize()
{
  ROS_ASSERT(!finalized_);
  
  // -- Flush out the buffer.
  while(true) {
    size_t output_buffer_max_size = 1e6;
    output_buffer_.resize(output_buffer_max_size);
    size_t out_size = avcodec_encode_video(ctx_, output_buffer_.data(), output_buffer_.size(), NULL);
    ROS_ASSERT(out_size < output_buffer_.size());
    output_buffer_.resize(out_size);
    blob_.insert(blob_.end(), output_buffer_.begin(), output_buffer_.end());
    if(out_size == 0)
      break;
  }

  // -- Add sequence at end to produce a real mpeg file.
  blob_.resize(blob_.size() + 4);
  blob_[blob_.size() - 4] = 0x00;
  blob_[blob_.size() - 3] = 0x00;
  blob_[blob_.size() - 2] = 0x01;
  blob_[blob_.size() - 1] = 0xb7;
  
  // -- Clean up.
  avcodec_close(ctx_);
  av_free(ctx_);
  av_free(frame_);

  finalized_ = true;
}

void H264Encoder::save(const std::string& path) const
{
  ROS_ASSERT(finalized_);
  FILE* file = fopen(path.c_str(), "wb");
  ROS_ASSERT(file);
  fwrite(blob_.data(), 1, blob_.size(), file);
  fclose(file);
}


void writeToVec(size_t num, std::vector<uint8_t>* data)
{
  size_t idx = data->size();
  data->resize(data->size() + sizeof(size_t));
  memcpy(data->data() + idx, &num, sizeof(size_t));
}

size_t readFromVec(const std::vector<uint8_t>& data, size_t idx, size_t* num)
{
  memcpy(num, data.data() + idx, sizeof(size_t));
  return idx + sizeof(size_t);
}

void writeToVec(const std::vector<uint8_t>& chunk, std::vector<uint8_t>* data)
{
  // Assume that data is sufficiently large that we are not re-allocating constantly.
  size_t idx = data->size();
  data->resize(data->size() + chunk.size() + sizeof(size_t));
  size_t buf = chunk.size();
  memcpy(data->data() + idx, &buf, sizeof(size_t));
  memcpy(data->data() + idx + sizeof(size_t), chunk.data(), chunk.size());
}

size_t readFromVec(const std::vector<uint8_t>& data, size_t idx, std::vector<uint8_t>* chunk)
{
  size_t buf;
  memcpy(&buf, data.data() + idx, sizeof(size_t));
  chunk->resize(buf);
  memcpy(chunk->data(), data.data() + idx + sizeof(size_t), buf);
  return idx + buf + sizeof(size_t);
}

int64_t filesize(const std::string& path)
{
  ifstream ifs(path, ios::binary);
  ifs.seekg(0, ios::end);
  ROS_ASSERT(ifs.tellg() > 0);
  return ifs.tellg();
}

void readBlob(const std::string& path, std::vector<uint8_t>* blob)
{
  int64_t length = filesize(path);
  blob->clear();
  blob->resize(length);
  ifstream ifs(path, ios::binary);
  ifs.read((char*)blob->data(), length);
}

void writeBlob(const std::vector<uint8_t>& blob, const std::string& path)
{
  ofstream file(path.c_str());
  file.write((char*)blob.data(), blob.size());
  file.close();
}

void encodeH264Shm(const EncodingOptions& opts, const std::vector<cv::Mat3b>& images, std::vector<uint8_t>* blob)
{
  // -- Create a directory to work in.
  string dir = "/dev/shm/encodeH264Shm-" + bfs::unique_path().string();  // Technically a bad idea but fine for prototyping.
  ROS_ASSERT(!bfs::exists(dir));
  bfs::create_directory(dir);

  // -- Dump out all the images.
    for(size_t i = 0; i < images.size(); ++i) {
    ostringstream oss;
    oss << dir << "/img" << setw(6) << setfill('0') << i << ".ppm";
    cv::imwrite(oss.str(), images[i]);
  }
    
  // -- Encode the video.
  ostringstream oss;
  oss << "avconv -i " << dir << "/img%06d.ppm -c:v libx264 -crf " << opts.crf_ << " -r " << opts.fps_ << " " << dir << "/video.avi > /dev/null 2>&1";
  int retval = system(oss.str().c_str());
  ROS_ASSERT(retval == 0);

  readBlob(dir + "/video.avi", blob);
  bfs::remove_all(dir);
}

void decodeH264Shm(const std::vector<uint8_t>& blob, std::vector<cv::Mat3b>* images)
{
  // -- Create a directory to work in.
  string dir = "/dev/shm/decodeH264Shm-" + bfs::unique_path().string();  // Technically a bad idea but fine for prototyping.
  ROS_ASSERT(!bfs::exists(dir));
  bfs::create_directory(dir);

  // -- Put the blob into a file.
  writeBlob(blob, dir + "/video.avi");
  
  // -- Decode individual frames.
  bfs::create_directory(dir + "/imgs");
  int retval = system(string("avconv -i " + dir + "/video.avi " + dir + "/imgs/img%06d.ppm > /dev/null 2>&1").c_str());
  ROS_ASSERT(retval == 0);

  // -- Read the frames.
  vector<string> paths = glob(dir + "/imgs/*");
  images->clear();
  for(size_t i = 0; i < paths.size(); ++i)
    images->push_back(cv::imread(paths[i]));

  // -- Clean up.
  bfs::remove_all(dir);
}
