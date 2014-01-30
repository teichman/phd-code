#include <timer/timer.h>
#include <jarvis/tracker.h>
#include <online_learning/dataset.h>
#include <boost/program_options.hpp>
#include <pcl/io/lzf.h>
#include <boost/filesystem.hpp>
#include <bag_of_tricks/glob.h>
#include <jarvis/compression_helpers.h>
extern "C" {
#include <libavcodec/avcodec.h>
}

using namespace std;
namespace bfs = boost::filesystem;

class CompressionTest
{
public:
  CompressionTest(std::string name) : name_(name) {}
  //! Compress the chunk and append to data.
  virtual void compressChunk(const std::vector<cv::Mat3b>& color,
                             std::vector<uint8_t>* data) = 0;
  //! Decompress the chunk that starts at data[idx].
  //! Return the index that the next track starts at.
  //! If none, return zero.
  virtual size_t decompressChunk(const std::vector<uint8_t>& data,
                                 size_t idx, std::vector<cv::Mat3b>* color) = 0;

  void run(const std::vector<cv::Mat3b>& color);

protected:
  std::string name_;

  double meanPixelDifference(cv::Mat3b img0, cv::Mat3b img1) const;
};

class SingleFrameCompressionTest : public CompressionTest
{
public:
  
  SingleFrameCompressionTest(std::string name) :
    CompressionTest(name)
  {
  }
  
  //! Compress the chunk and append to data.
  void compressChunk(const std::vector<cv::Mat3b>& color,
                     std::vector<uint8_t>* data);
  //! Decompress the chunk that starts at data[idx].
  //! Return the index that the next track starts at.
  //! If none, return zero.
  size_t decompressChunk(const std::vector<uint8_t>& data,
                         size_t idx, std::vector<cv::Mat3b>* color);

protected:
  //! Temporary storage.  Avoids excessive reallocation.
  vector<uint8_t> buffer_;

  virtual size_t decompressImage(const std::vector<uint8_t>& data,
                                 size_t idx, cv::Mat3b* img) = 0;
  virtual void compressImage(cv::Mat3b img, std::vector<uint8_t>* data) = 0;
};

class JPGCompressionTest : public SingleFrameCompressionTest
{
public:
  JPGCompressionTest(std::string name, int level) :
    SingleFrameCompressionTest(name),
    level_(level)
  {
  }
  
  size_t decompressImage(const std::vector<uint8_t>& data,
                         size_t idx, cv::Mat3b* img);
  void compressImage(cv::Mat3b img, std::vector<uint8_t>* data);

protected:
  int level_;
};

class PNGCompressionTest : public SingleFrameCompressionTest
{
public:
  PNGCompressionTest(std::string name, int level) :
    SingleFrameCompressionTest(name),
    level_(level)
  {
  }
  
  size_t decompressImage(const std::vector<uint8_t>& data,
                         size_t idx, cv::Mat3b* img);
  void compressImage(cv::Mat3b img, std::vector<uint8_t>* data);

protected:
  int level_;
};

class VideoCompressionTest : public CompressionTest
{
public:
  
  VideoCompressionTest(std::string name) :
    CompressionTest(name)
  {
  }
  
  //! Compress the chunk and append to data.
  void compressChunk(const std::vector<cv::Mat3b>& color,
                     std::vector<uint8_t>* data);
  //! Decompress the chunk that starts at data[idx].
  //! Return the index that the next track starts at.
  //! If none, return zero.
  size_t decompressChunk(const std::vector<uint8_t>& data,
                         size_t idx, std::vector<cv::Mat3b>* color);

protected:
  //! Temporary storage.  Avoids excessive reallocation.
  vector<uint8_t> buffer_;
};

class OpenCVVideoCompressionTest : public CompressionTest
{
public:
  
  OpenCVVideoCompressionTest(std::string name) :
    CompressionTest(name)
  {
  }
  
  //! Compress the chunk and append to data.
  void compressChunk(const std::vector<cv::Mat3b>& color,
                     std::vector<uint8_t>* data);
  //! Decompress the chunk that starts at data[idx].
  //! Return the index that the next track starts at.
  //! If none, return zero.
  size_t decompressChunk(const std::vector<uint8_t>& data,
                         size_t idx, std::vector<cv::Mat3b>* color);

protected:
  //! Temporary storage.  Avoids excessive reallocation.
  vector<uint8_t> buffer_;
};

double CompressionTest::meanPixelDifference(cv::Mat3b img0, cv::Mat3b img1) const
{
  ROS_ASSERT(img0.size() == img1.size());

  double diff = 0;
  for(int y = 0; y < img0.rows; ++y) {
    for(int x = 0; x < img0.cols; ++x) {
      diff += fabs(img0(y, x)[0] - img1(y, x)[0]);
      diff += fabs(img0(y, x)[1] - img1(y, x)[1]);
      diff += fabs(img0(y, x)[2] - img1(y, x)[2]);
    }
  }
  diff /= (3 * img0.rows * img0.cols);
  return diff;
}

void CompressionTest::run(const std::vector<cv::Mat3b>& color)
{
  for(size_t i = 1; i < color.size(); ++i)
    ROS_ASSERT(color[i].size() == color[i-1].size());

  size_t num_bytes_raw = color[0].rows * color[0].cols * 3 * color.size();
  HighResTimer hrt; 
  
  vector<uint8_t> compressed;
  compressed.reserve(num_bytes_raw);
  hrt.reset(); hrt.start();
  compressChunk(color, &compressed);
  hrt.stop();
  cout << "Uncompressed size (bytes): " << num_bytes_raw << endl;
  cout << name_ << " compressed size (bytes): " << compressed.size() << endl;
  cout << name_ << " compression ratio vs raw: " << (double)num_bytes_raw / compressed.size() << endl;
  cout << name_ << " compression time (ms / frame): " << hrt.getMilliseconds() / color.size() << endl;

  vector<cv::Mat3b> color2;
  color2.reserve(color.size());
  hrt.reset(); hrt.start();
  decompressChunk(compressed, 0, &color2);
  hrt.stop();
  cout << name_ << " decompression time (ms / frame): " << hrt.getMilliseconds() / color.size() << endl;

  if(color.size() != color2.size()) {
    cout << "Before: " << color.size() << " frames." << endl;
    cout << "After: " << color2.size() << " frames." << endl;
    ROS_ASSERT(0);
  }

  double diff = 0;
  for(size_t i = 0; i < color.size(); ++i) {
    cv::imshow("Decompressed", color2[i]);
    cv::waitKey(2);
    diff += meanPixelDifference(color[i], color2[i]);
  }
  diff /= color.size();
  cout << name_ << " mean pixel difference (RGB values): " << diff << endl;
}

void SingleFrameCompressionTest::compressChunk(const std::vector<cv::Mat3b>& color,
                                               std::vector<uint8_t>* data)
{
  writeToVec(color.size(), data);
  for(size_t i = 0; i < color.size(); ++i)
    compressImage(color[i], data);
}

size_t SingleFrameCompressionTest::decompressChunk(const std::vector<uint8_t>& data,
                                                   size_t idx, std::vector<cv::Mat3b>* color)
{
  size_t num_frames;
  idx = readFromVec(data, idx, &num_frames);
  color->clear();
  color->reserve(num_frames);

  for(size_t i = 0; i < num_frames; ++i) {
    cv::Mat3b img;
    idx = decompressImage(data, idx, &img);
    color->push_back(img);
  }

  return idx;
}

void PNGCompressionTest::compressImage(cv::Mat3b img, std::vector<uint8_t>* data) 
{
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(level_);  

  buffer_.clear();
  buffer_.reserve(100000);
  cv::imencode(".png", img, buffer_, compression_params);
  writeToVec(buffer_, data);
}

size_t PNGCompressionTest::decompressImage(const std::vector<uint8_t>& data,
                                           size_t idx, cv::Mat3b* img) 
{
  buffer_.clear();
  idx = readFromVec(data, idx, &buffer_);
  *img = cv::imdecode(buffer_, -1);  // Load as-is.
  return idx;
}

void JPGCompressionTest::compressImage(cv::Mat3b img, std::vector<uint8_t>* data) 
{
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(level_);  

  buffer_.clear();
  buffer_.reserve(100000);
  cv::imencode(".jpg", img, buffer_, compression_params);
  writeToVec(buffer_, data);
}

size_t JPGCompressionTest::decompressImage(const std::vector<uint8_t>& data,
                                           size_t idx, cv::Mat3b* img) 
{
  buffer_.clear();
  idx = readFromVec(data, idx, &buffer_);
  *img = cv::imdecode(buffer_, -1);  // Load as-is.
  return idx;
}
  

void VideoCompressionTest::compressChunk(const std::vector<cv::Mat3b>& color,
                                         std::vector<uint8_t>* data)
{
  avcodec_register_all();
  
  // -- Set up AVCodec.
  //    See video_encode_example in libav.
  //AVCodec* codec = avcodec_find_encoder(CODEC_ID_FFV1);
  //AVCodec* codec = avcodec_find_encoder(CODEC_ID_H264);
  AVCodec* codec = avcodec_find_encoder(CODEC_ID_MPEG1VIDEO);
  AVCodecContext* ctx = avcodec_alloc_context3(codec);
  ctx->bit_rate = 400000;
  ctx->width = color[0].cols;
  ctx->height = color[0].rows;
  ctx->time_base = (AVRational){1, 25};
  ctx->gop_size = 10;
  ctx->max_b_frames = 1;
  //ctx->pix_fmt = PIX_FMT_RGB32;
  ctx->pix_fmt = PIX_FMT_YUV420P;

  // // Shamelessly copied from http://stackoverflow.com/questions/3553003/encoding-h-264-with-libavcodec-x264
  // ctx->bit_rate = 500*1000;
  // ctx->bit_rate_tolerance = 0;
  // ctx->rc_max_rate = 0;
  // ctx->rc_buffer_size = 0;
  // ctx->gop_size = 40;
  // ctx->max_b_frames = 3;
  // ctx->b_frame_strategy = 1;
  // ctx->coder_type = 1;
  // ctx->me_cmp = 1;
  // ctx->me_range = 16;
  // ctx->qmin = 10;
  // ctx->qmax = 51;
  // ctx->scenechange_threshold = 40;
  // ctx->flags |= CODEC_FLAG_LOOP_FILTER;
  // ctx->me_method = ME_HEX;
  // ctx->me_subpel_quality = 5;
  // ctx->i_quant_factor = 0.71;
  // ctx->qcompress = 0.6;
  // ctx->max_qdiff = 4;
  // ctx->directpred = 1;
  // ctx->flags2 |= CODEC_FLAG2_FASTPSKIP;

  int err = avcodec_open2(ctx, codec, NULL);
  if(err < 0) {
    cout << "Failed to open codec." << endl;
    cout << "Error: " << err << endl;
    ROS_ASSERT(0);
  }

  // -- Set up the AVFrame.
  size_t num_pixels = color[0].rows * color[0].cols;
  vector<uint8_t> frame_buffer(3 * num_pixels / 2);
  AVFrame* frame = avcodec_alloc_frame();
  frame->data[0] = frame_buffer.data();
  frame->data[1] = frame->data[0] + num_pixels;
  frame->data[2] = frame->data[1] + num_pixels / 4;
  frame->linesize[0] = ctx->width;
  frame->linesize[1] = ctx->width / 2;
  frame->linesize[2] = ctx->width / 2;

  // -- Encode each frame one at a time.
  cv::Mat3b yuv;
  for(size_t i = 0; i < color.size(); ++i) {
    cv::cvtColor(color[i], yuv, cv::COLOR_BGR2YCrCb);
    for(size_t j = 0; j < num_pixels; ++j)
      frame->data[0][j] = yuv(j)[0];
    size_t idx = 0;
    for(int y = 0; y < yuv.rows; y += 2) {
      for(int x = 0; x < yuv.rows; x += 2, ++idx) { 
        frame->data[1][idx] = yuv(y, x)[1];
        frame->data[2][idx] = yuv(y, x)[2];
      }
    }
    vector<uint8_t> output_buffer(100000);  
    size_t out_size = avcodec_encode_video(ctx, output_buffer.data(), output_buffer.size(), frame);
    ROS_ASSERT(out_size < output_buffer.size());
    output_buffer.resize(out_size);
    writeToVec(output_buffer, data);
  }
  
  // -- Clean up.
  avcodec_close(ctx);
  av_free(ctx);
  av_free(frame);
}

static void pgm_save(unsigned char *buf, int wrap, int xsize, int ysize,
                     char *filename)
{
    FILE *f;
    int i;

    f=fopen(filename,"w");
    fprintf(f,"P5\n%d %d\n%d\n",xsize,ysize,255);
    for(i=0;i<ysize;i++)
        fwrite(buf + i * wrap,1,xsize,f);
    fclose(f);
}

size_t VideoCompressionTest::decompressChunk(const std::vector<uint8_t>& data,
                                             size_t idx, std::vector<cv::Mat3b>* color)
{
  // See video_decode_example().
  AVCodec* codec = avcodec_find_decoder(CODEC_ID_MPEG1VIDEO);
  ROS_ASSERT(codec);
  AVCodecContext* ctx = avcodec_alloc_context3(codec);
  AVFrame* frame = avcodec_alloc_frame();

  // Dunno what this is.
  if(codec->capabilities & CODEC_CAP_TRUNCATED)
    ctx->flags |= CODEC_FLAG_TRUNCATED; /* we do not send complete frames */

  int err = avcodec_open2(ctx, codec, NULL);
  if(err < 0) {
    cout << "Failed to open codec." << endl;
    cout << "Error: " << err << endl;
    ROS_ASSERT(0);
  }

  AVPacket avpkt;
  av_init_packet(&avpkt);
  size_t inbuf_size = 4096;
  size_t ff_input_buffer_padding_size = 8;
  /* set end of buffer to 0 (this ensures that no overreading happens for damaged mpeg streams) */
  vector<uint8_t> inbuf(inbuf_size + ff_input_buffer_padding_size, 0);

  vector<uint8_t> chunk;
  chunk.reserve(100000);
  int num = 0;
  while(idx < data.size()) {
    // Get the next chunk.
    chunk.clear();
    idx = readFromVec(data, idx, &chunk);
    if(chunk.empty())
      continue;
    
    avpkt.size = chunk.size();
    avpkt.data = chunk.data();

    while(avpkt.size > 0) {
      int got_frame;
      int len = avcodec_decode_video2(ctx, frame, &got_frame, &avpkt);
      if(len < 0) {
        fprintf(stderr, "Error while decoding frame %d\n", num);
        exit(1);
      }
      if(got_frame) {
        // printf("saving frame %3d\n", num);
        // fflush(stdout);
        /* the picture is allocated by the decoder. no need to
           free it */
        // char buf[1024];
        // snprintf(buf, sizeof(buf), "test%04d.pgm", num);
        // pgm_save(frame->data[0], frame->linesize[0],
        //          ctx->width, ctx->height, buf);

        cv::Mat3b yuv(cv::Size(ctx->width, ctx->height));
        for(int y = 0; y < yuv.rows; ++y) {
          for(int x = 0; x < yuv.cols; ++x) {
            yuv(y, x)[0] = frame->data[0][y * frame->linesize[0] + x];
          }
        }

        cv::Mat3b bgr;
        cv::cvtColor(yuv, bgr, cv::COLOR_YCrCb2BGR);
        cv::imshow("decompressed", bgr);
        cv::waitKey(2);
        color->push_back(bgr);
        
        num++;
      }
      avpkt.size -= len;
      avpkt.data += len;
    }
  }
  return idx;
}

void OpenCVVideoCompressionTest::compressChunk(const std::vector<cv::Mat3b>& color,
                                               std::vector<uint8_t>* data)
{
  cv::VideoWriter writer;

  // This consistently spits out "Unsupported format or combination of formats"
  // I am compiling with FFMPEG=ON.
  writer.open("tmpvideo", CV_FOURCC('H','2','6','4'), 30, color[0].size());
  //writer.open("tmpvideo", CV_FOURCC('P','I','M','1'), 30, color[0].size());
  //writer.open("tmpvideo", CV_FOURCC('M','J','P','G'), 30, color[0].size());
  
  for(size_t i = 0; i < color.size(); ++i)
    writer.write(color[i]);
}
size_t OpenCVVideoCompressionTest::decompressChunk(const std::vector<uint8_t>& data,
                                                   size_t idx, std::vector<cv::Mat3b>* color)
{
  ROS_ASSERT(0);
  return 0;
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string rgb_dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("rgb-dir", bpo::value(&rgb_dir)->required(), "Directory of RGB images")
    ;

  p.add("rgb-dir", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] RGB_DIR" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Load the data.
  vector<string> paths = glob(rgb_dir + "/*.png");
  vector<cv::Mat3b> color;
  color.reserve(paths.size());
  for(size_t i = 0; i < paths.size(); ++i) {
    color.push_back(cv::imread(paths[i]));
    cv::imshow("Color", color.back());
    cv::waitKey(2);
  }
  cout << "Loaded " << color.size() << " test images." << endl;
  
  vector<CompressionTest*> cts;
  //cts.push_back(new OpenCVVideoCompressionTest("OpenCVVideoCompressionTest"));
  cts.push_back(new VideoCompressionTest("VideoCompressionTest"));
  cts.push_back(new JPGCompressionTest("JPGCompressionTest-0", 0));
  cts.push_back(new JPGCompressionTest("JPGCompressionTest-50", 50));
  cts.push_back(new JPGCompressionTest("JPGCompressionTest-90", 90));
  cts.push_back(new JPGCompressionTest("JPGCompressionTest-99", 99));
  cts.push_back(new PNGCompressionTest("PNGCompressionTest-0", 0));
  cts.push_back(new PNGCompressionTest("PNGCompressionTest-5", 5));
  cts.push_back(new PNGCompressionTest("PNGCompressionTest-9", 9));

  for(size_t i = 0; i < cts.size(); ++i) {
    cout << "================================================================================" << endl;
    CompressionTest& ct = *cts[i];
    ct.run(color);
  }

  return 0;
}
