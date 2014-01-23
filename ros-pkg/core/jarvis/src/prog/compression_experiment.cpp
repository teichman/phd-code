#include <timer/timer.h>
#include <jarvis/tracker.h>
#include <online_learning/dataset.h>
#include <boost/program_options.hpp>

using namespace std;

class CompressionType
{
public:
  CompressionType(std::string name);
  virtual void compress(const TrackDataset& td, std::vector<uint8_t>* data) = 0;
  virtual TrackDataset decompress(const std::vector<uint8_t>& data) = 0;

  void run(const TrackDataset& td);
  
protected:
  std::string name_;

  // The following consider only the raw_ data.
  bool equal(const TrackDataset& td0, const TrackDataset& td1) const;
  bool equal(const Blob& blob0, const Blob& blob1) const;
  size_t numBytes(const Blob& blob) const;
  size_t numBytes(const TrackDataset& td) const;
};

CompressionType::CompressionType(std::string name) :
  name_(name)
{
}

size_t CompressionType::numBytes(const Blob& blob) const
{
  size_t num_bytes = 0;
  if(!blob.indices_.empty()) {
    num_bytes += blob.indices_.size() * sizeof(blob.indices_[0]);
    num_bytes += blob.color_.size() * sizeof(blob.color_[0]);
    num_bytes += blob.depth_.size() * sizeof(blob.depth_[0]);
  }
  return num_bytes;
}

size_t CompressionType::numBytes(const TrackDataset& td) const
{
  size_t num_bytes = 0;
  for(size_t i = 0; i < td.size(); ++i) {
    for(size_t j = 0; j < td[i].size(); ++j) {
      const Blob& blob = *boost::any_cast<Blob::Ptr>(td[i][j].raw_);
      num_bytes += numBytes(blob);
    }
  }
  return num_bytes;
}
  
bool CompressionType::equal(const Blob& blob0, const Blob& blob1) const
{
  if(blob0.indices_.size() != blob1.indices_.size())
    return false;
  if(blob0.color_.size() != blob1.color_.size())
    return false;
  if(blob0.depth_.size() != blob1.depth_.size())
    return false;

  for(size_t i = 0; i < blob0.indices_.size(); ++i)
    if(blob0.indices_[i] != blob1.indices_[i])
      return false;
  for(size_t i = 0; i < blob0.color_.size(); ++i)
    if(blob0.color_[i] != blob1.color_[i])
      return false;
  for(size_t i = 0; i < blob0.depth_.size(); ++i)
    if(blob0.depth_[i] != blob1.depth_[i])
      return false;

  return true;
}

bool CompressionType::equal(const TrackDataset& td0, const TrackDataset& td1) const
{
  if(td0.size() != td1.size())
    return false;
  
  for(size_t i = 0; i < td0.size(); ++i) {
    if(td0[i].size() != td1[i].size())
      return false;
    
    for(size_t j = 0; j < td0[i].size(); ++j) {
      const Blob& blob0 = *boost::any_cast<Blob::Ptr>(td0[i][j].raw_);
      const Blob& blob1 = *boost::any_cast<Blob::Ptr>(td1[i][j].raw_);
      if(!equal(blob0, blob1))
        return false;
    }
  }

  return true;
}

void CompressionType::run(const TrackDataset& td) 
{
  HighResTimer hrt;

  hrt.reset(); hrt.start();
  vector<uint8_t> compressed;
  // Reserve enough space for the whole thing uncompressed.
  // Re-allocation will not be an issue.
  compressed.reserve(numBytes(td));  
  compress(td, &compressed);
  hrt.stop();
  cout << name_ << " compression ratio: " << (double)numBytes(td) / compressed.size() << endl;
  cout << name_ << " compression time (ms per instance): "
       << hrt.getMilliseconds() / td.totalInstances() << endl;


  hrt.reset(); hrt.start();
  TrackDataset td2 = decompress(compressed);
  hrt.stop();
  cout << name_ << " decompression time (ms per instance): "
       << hrt.getMilliseconds() / td.totalInstances() << endl;

  ROS_ASSERT(equal(td, td2));  // Ensure it is lossless compression.
}

class SingleFrameCompression : public CompressionType
{
public:
  SingleFrameCompression();
  void compress(const TrackDataset& td, std::vector<uint8_t>* data);
  TrackDataset decompress(const std::vector<uint8_t>& data);

protected:
  cv::Mat3b color_;
  cv::Mat1f depth_;
  
  void compress(const Blob& blob, std::vector<uint8_t>* data);
  size_t decompress(const std::vector<uint8_t>& data, size_t idx, Blob* blob);
};

SingleFrameCompression::SingleFrameCompression() :
    CompressionType("SingleFrameCompression")
{
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
  return idx + buf;
}

void SingleFrameCompression::compress(const Blob& blob, std::vector<uint8_t>* data) 
{
  // // -- Make rectangular images that can contain the blob.
  // int min_u = blob.width_;
  // int max_u = 0;
  // int min_v = blob.height_;
  // int max_v = 0;
  // for(size_t i = 0; i < blob.indices_.size(); ++i) {
  //   int u, v;
  //   blob.coords(i, &u, &v);
  //   min_u = min(min_u, u);
  //   max_u = max(max_u, u);
  //   min_v = min(min_v, v);
  //   max_v = max(max_v, v);
  // }
  // cv::Size size(max_u - min_u, max_v - min_v);

  if(color_.rows != blob.height_) {
    cv::Size size(blob.width_, blob.height_);
    color_ = cv::Mat3b(size);
    depth_ = cv::Mat1f(size);
  }
  color_ = cv::Vec3b(127, 127, 127);
  depth_ = 0;
  
  // -- Make the color and depth images.
  for(size_t i = 0; i < blob.indices_.size(); ++i) {
    int u, v;
    blob.coords(i, &u, &v);
    color_(v, u)[2] = blob.color_[i*3+0];
    color_(v, u)[1] = blob.color_[i*3+1];
    color_(v, u)[0] = blob.color_[i*3+2];
    depth_(v, u) = blob.depth_[i];
  }

  cv::imshow("color", color_);
  cv::imshow("depth", depth_);
  cv::waitKey();
  
  // -- Encode using PNG.

  // 1-9.  Higher is better but slower.
  // http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#bool%20imwrite%28const%20string&%20filename,%20InputArray%20img,%20const%20vector%3Cint%3E&%20params%29
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(3);

  vector<uint8_t> color_data;
  vector<uint8_t> depth_data;
  cv::imencode(".png", color_, color_data, compression_params);
  cv::imencode(".png", depth_, depth_data, compression_params);
  writeToVec(color_data, data);
  writeToVec(depth_data, data);
}

void SingleFrameCompression::compress(const TrackDataset& td, std::vector<uint8_t>* data)
{
  for(size_t i = 0; i < td.size(); ++i) {
    for(size_t j = 0; j < td[i].size(); ++j) {
      const Blob& blob = *boost::any_cast<Blob::Ptr>(td[i][j].raw_);
      compress(blob, data);
    }
  }
}

size_t SingleFrameCompression::decompress(const std::vector<uint8_t>& data,
                                          size_t idx, Blob* blob) 
{
  
}

TrackDataset SingleFrameCompression::decompress(const std::vector<uint8_t>& data) 
{
  TrackDataset td;
  td.tracks_.reserve(10000);
  
  size_t idx = 0;
  while(idx < data.size()) {
    Blob blob;
    idx = decompress(data, idx, &blob);
    
  }
}


int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  vector<string> td_paths;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("tds,d", bpo::value(&td_paths)->required()->multitoken(), "TD paths")
    ;

  p.add("tds", -1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] TD [TD ...]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  vector<CompressionType*> cts;
  cts.push_back(new SingleFrameCompression);

  TrackDataset td = *loadDatasets(td_paths);
  for(size_t i = 0; i < cts.size(); ++i) {
    CompressionType& ct = *cts[i];
    ct.run(td);
  }

  return 0;
}
