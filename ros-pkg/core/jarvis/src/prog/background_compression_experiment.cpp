#include <timer/timer.h>
#include <jarvis/tracker.h>
#include <online_learning/dataset.h>
#include <boost/program_options.hpp>
#include <pcl/io/lzf.h>
#include <boost/filesystem.hpp>
#include <bag_of_tricks/glob.h>
#include <jarvis/compression_helpers.h>

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

  ROS_ASSERT(color.size() == color2.size());
  double diff = 0;
  for(size_t i = 0; i < color.size(); ++i) {
    cv::imshow("Decompressed", color2[i]);
    cv::waitKey(2);
    diff += meanPixelDifference(color[i], color2[i]);
  }
  diff /= color.size();
  cout << name_ << " mean pixel difference (RGB values): " << diff << endl;
}

class PNGCompressionTest : public CompressionTest
{
public:
  
  PNGCompressionTest(std::string name, int level) :
    CompressionTest(name),
    level_(level)
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
  int level_;
  //! Temporary storage.  Avoids excessive reallocation.
  vector<uint8_t> buffer_;

  size_t decompressImage(const std::vector<uint8_t>& data,
                         size_t idx, cv::Mat3b* img);
  void compressImage(cv::Mat3b img, std::vector<uint8_t>* data);
};


void PNGCompressionTest::compressChunk(const std::vector<cv::Mat3b>& color,
                                               std::vector<uint8_t>* data)
{
  writeToVec(color.size(), data);
  for(size_t i = 0; i < color.size(); ++i)
    compressImage(color[i], data);
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

size_t PNGCompressionTest::decompressChunk(const std::vector<uint8_t>& data,
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

size_t PNGCompressionTest::decompressImage(const std::vector<uint8_t>& data,
                                           size_t idx, cv::Mat3b* img) 
{
  buffer_.clear();
  idx = readFromVec(data, idx, &buffer_);
  *img = cv::imdecode(buffer_, -1);  // Load as-is.
  return idx;
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
