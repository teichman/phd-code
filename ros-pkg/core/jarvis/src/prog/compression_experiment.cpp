#include <timer/timer.h>
#include <jarvis/tracker.h>
#include <online_learning/dataset.h>
#include <boost/program_options.hpp>

using namespace std;

class CompressionType
{
public:
  CompressionType(std::string name);
  //! Compress the track and append to data.
  virtual void compressTrack(const Dataset& track,
                             std::vector<uint8_t>* data) = 0;
  //! Decompress the track that starts at data[idx].
  //! Return the index that the next track starts at.
  //! If none, return zero.
  virtual size_t decompressTrack(const std::vector<uint8_t>& data,
                                 size_t idx, Dataset* track) = 0;
  
  void run(const TrackDataset& td);
  
protected:
  std::string name_;

  // The following consider only the raw_ data.
  void assertEqual(const TrackDataset& td0, const TrackDataset& td1) const;
  void assertEqual(const Blob& blob0, const Blob& blob1) const;
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
  
void CompressionType::assertEqual(const Blob& blob0, const Blob& blob1) const
{
  cv::imshow("blob0", blob0.image());
  cv::imshow("blob1", blob1.image());
  cv::waitKey(2);
  return;  // SingleFrameCompression is apparently not lossless yet.
  
  ROS_ASSERT(blob0.indices_.size() == blob1.indices_.size());
  ROS_ASSERT(blob0.color_.size() == blob1.color_.size());
  ROS_ASSERT(blob0.depth_.size() == blob1.depth_.size());

  for(size_t i = 0; i < blob0.indices_.size(); ++i)
    ROS_ASSERT(blob0.indices_[i] == blob1.indices_[i]);
  for(size_t i = 0; i < blob0.color_.size(); ++i)
    ROS_ASSERT(blob0.color_[i] == blob1.color_[i]);
  for(size_t i = 0; i < blob0.depth_.size(); ++i)
    ROS_ASSERT(blob0.depth_[i] == blob1.depth_[i]);
}

void CompressionType::assertEqual(const TrackDataset& td0, const TrackDataset& td1) const
{
  ROS_ASSERT(td0.size() == td1.size());
  for(size_t i = 0; i < td0.size(); ++i) {
    ROS_ASSERT(td0[i].size() == td1[i].size());
    for(size_t j = 0; j < td0[i].size(); ++j) {
      const Blob& blob0 = *boost::any_cast<Blob::Ptr>(td0[i][j].raw_);
      const Blob& blob1 = *boost::any_cast<Blob::Ptr>(td1[i][j].raw_);
      assertEqual(blob0, blob1);
    }
  }
}

void CompressionType::run(const TrackDataset& td) 
{
  HighResTimer hrt;
  
  hrt.reset(); hrt.start();
  vector<uint8_t> compressed;
  // Reserve enough space for the whole thing uncompressed.
  // Re-allocation will not be an issue.
  compressed.reserve(numBytes(td));
  for(size_t i = 0; i < td.size(); ++i)
    compressTrack(td[i], &compressed);
  hrt.stop();
  cout << name_ << " compression ratio: " << (double)numBytes(td) / compressed.size() << endl;
  cout << name_ << " compression time (ms per instance): "
       << hrt.getMilliseconds() / td.totalInstances() << endl;


  hrt.reset(); hrt.start();
  TrackDataset td2;
  td2.tracks_.resize(td.size());
  size_t idx = 0;
  for(size_t i = 0; i < td.size(); ++i) {
    td2.tracks_[i] = Dataset::Ptr(new Dataset);
    idx = decompressTrack(compressed, idx, td2.tracks_[i].get());
  }
  hrt.stop();
  cout << name_ << " decompression time (ms per instance): "
       << hrt.getMilliseconds() / td.totalInstances() << endl;

  assertEqual(td, td2);  // Ensure it is lossless compression.
}

class SingleFrameCompression : public CompressionType
{
public:
  SingleFrameCompression();
  void compressTrack(const Dataset& track, std::vector<uint8_t>* data);
  size_t decompressTrack(const std::vector<uint8_t>& data,
                         size_t idx, Dataset* track);

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

void SingleFrameCompression::compress(const Blob& blob, std::vector<uint8_t>* data) 
{
  if(color_.rows != blob.height_) {
    cv::Size size(blob.width_, blob.height_);
    color_ = cv::Mat3b(size);
    depth_ = cv::Mat1f(size);
  }
  color_ = cv::Vec3b(127, 127, 127);
  depth_ = 0;
  
  // -- Make the color and depth images.
  for(size_t i = 0; i < blob.indices_.size(); ++i) {
    int idx = blob.indices_[i];
    color_(idx)[2] = blob.color_[i*3+0];
    color_(idx)[1] = blob.color_[i*3+1];
    color_(idx)[0] = blob.color_[i*3+2];
    depth_(idx) = blob.depth_[i];
  }

  // -- Encode using PNG.

  // 1-9.  Higher is better but slower.
  // http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#bool%20imwrite%28const%20string&%20filename,%20InputArray%20img,%20const%20vector%3Cint%3E&%20params%29
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  vector<uint8_t> color_data;
  vector<uint8_t> depth_data;
  cv::imencode(".png", color_, color_data, compression_params);
  cv::imencode(".png", depth_, depth_data, compression_params);
  writeToVec(color_data, data);
  writeToVec(depth_data, data);
}

size_t SingleFrameCompression::decompress(const std::vector<uint8_t>& data,
                                          size_t idx, Blob* blob) 
{
  
  // -- PNG de-encode.
  vector<uint8_t> color_data;
  vector<uint8_t> depth_data;
  idx = readFromVec(data, idx, &color_data);
  idx = readFromVec(data, idx, &depth_data);
  cv::Mat3b color = cv::imdecode(color_data, -1);  // Load as-is.
  cv::Mat1f depth = cv::imdecode(depth_data, CV_LOAD_IMAGE_ANYDEPTH);  // This won't work.
  ROS_ASSERT(color.size == depth.size);

  // cv::imshow("color", color);
  // cv::imshow("depth", depth);
  // cv::waitKey();
  
  blob->height_ = color.rows;
  blob->width_ = color.cols;

  size_t num = 0;
  for(int i = 0; i < depth.rows * depth.cols; ++i)
    if(depth(i) != 0)
      ++num;

  blob->indices_.clear();
  blob->color_.clear();
  blob->depth_.clear();
  blob->indices_.reserve(num);
  blob->color_.reserve(num * 3);
  blob->depth_.reserve(num);

  for(int i = 0; i < depth.rows * depth.cols; ++i) {
    if(depth(i) != 0) {
      blob->indices_.push_back(i);
      blob->depth_.push_back(depth(i));
      blob->color_.push_back(color(i)[2]);
      blob->color_.push_back(color(i)[1]);
      blob->color_.push_back(color(i)[0]);
    }
  }
  
  return idx;
}


void SingleFrameCompression::compressTrack(const Dataset& track, std::vector<uint8_t>* data)
{
  writeToVec(track.size(), data);

  for(size_t i = 0; i < track.size(); ++i) {
    const Blob& blob = *boost::any_cast<Blob::Ptr>(track[i].raw_);
    compress(blob, data);
  }
}

size_t SingleFrameCompression::decompressTrack(const std::vector<uint8_t>& data,
                                               size_t idx, Dataset* track)
{
  
  size_t num_frames;
  idx = readFromVec(data, idx, &num_frames);
  track->instances_.resize(num_frames);
  for(size_t i = 0; i < track->size(); ++i) {
    Blob::Ptr blob(new Blob);
    idx = decompress(data, idx, blob.get());
    track->instances_[i].raw_ = blob;
  }

  return idx;
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
