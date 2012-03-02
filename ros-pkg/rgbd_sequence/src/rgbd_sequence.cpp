#include <rgbd_sequence/rgbd_sequence.h>

using namespace std;
namespace bfs = boost::filesystem;

RGBDSequence::RGBDSequence() :
  Serializable()
{
}

void RGBDSequence::serialize(std::ostream& out) const
{
  ROS_FATAL_STREAM("Use save() instead." << flush);
  abort();
}

void RGBDSequence::deserialize(std::istream& in)
{
  ROS_FATAL_STREAM("Use load() instead." << flush);
  abort();
}

void RGBDSequence::save(const std::string& dir) const
{
  ROS_ASSERT(!bfs::exists(dir));
  bfs::create_directory(dir);

  for(size_t i = 0; i < imgs_.size(); ++i) {
    ostringstream oss;
    oss << "img" << setw(4) << setfill('0') << i << ".png";
    cv::imwrite(dir + "/" + oss.str(), imgs_[i]);
  }

  for(size_t i = 0; i < pcds_.size(); ++i) {
    ostringstream oss;
    oss << "pcd" << setw(4) << setfill('0') << i << ".pcd";
    pcl::io::savePCDFileBinary(dir + "/" + oss.str(), *pcds_[i]);
  }
}

void RGBDSequence::load(const std::string& dir)
{
  // -- Get filenames.
  vector<string> img_names;
  vector<string> pcd_names;
    
  bfs::recursive_directory_iterator it(dir), eod;
  BOOST_FOREACH(bfs::path const & p, make_pair(it, eod)) {
    ROS_ASSERT(is_regular_file(p));
    if(p.leaf().string().substr(0, 3).compare("img") == 0 && bfs::extension(p).compare(".png") == 0)
      img_names.push_back(p.string());
    else if(bfs::extension(p).compare(".pcd") == 0)
      pcd_names.push_back(p.string());
  }
  ROS_ASSERT(img_names.size() == pcd_names.size());

  // -- Sort all filenames.
  sort(img_names.begin(), img_names.end());
  sort(pcd_names.begin(), pcd_names.end());

  // -- Load images and pointclouds.
  imgs_.resize(img_names.size());
  pcds_.resize(pcd_names.size());
  for(size_t i = 0; i < img_names.size(); ++i) {
    imgs_[i] = cv::imread(img_names[i], 1);
    pcds_[i] = RGBDCloud::Ptr(new RGBDCloud());
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_names[i], *pcds_[i]);
  }
}
  
RGBDSequence::RGBDSequence(const RGBDSequence& seq)
{
  ROS_ASSERT(seq.imgs_.size() == seq.pcds_.size());
  imgs_.resize(seq.imgs_.size());
  pcds_.resize(seq.imgs_.size());

  for(size_t i = 0; i < imgs_.size(); ++i) {
    imgs_[i] = seq.imgs_[i].clone();
    pcds_[i] = seq.pcds_[i]->makeShared();
  }
}
  
RGBDSequence& RGBDSequence::operator=(const RGBDSequence& seq)
{
  if(&seq == this)
    return *this;

  ROS_ASSERT(seq.imgs_.size() == seq.pcds_.size());
  imgs_.resize(seq.imgs_.size());
  pcds_.resize(seq.imgs_.size());
    
  for(size_t i = 0; i < imgs_.size(); ++i) {
    imgs_[i] = seq.imgs_[i].clone();
    pcds_[i] = seq.pcds_[i]->makeShared();
  }

  return *this;
}  

void loadSequences(const std::string& path,
		   std::vector<RGBDSequence::Ptr>* sequences)
{
  size_t prev_num_seq = sequences->size();
    
  bfs::directory_iterator end_itr; // default construction yields past-the-end
  for(bfs::directory_iterator itr(path); itr != end_itr; ++itr) {
    string p = itr->path().string();
    if(!bfs::is_directory(p))
      continue;

    RGBDSequence::Ptr seq(new RGBDSequence());
    seq->load(p);
    ROS_DEBUG_STREAM("Loaded sequence " << p << " with  " << seq->imgs_.size() << " frames.");
    sequences->push_back(seq);
  }

  // -- If it was just a single sequence directory, load that.
  if(sequences->size() == prev_num_seq) { 
    RGBDSequence::Ptr seq(new RGBDSequence());
    seq->load(path);
    ROS_DEBUG_STREAM("Loaded sequence " << path << " with  " << seq->imgs_.size() << " frames.");
    sequences->push_back(seq);
  }
}
  
