#include <rgbd_sequence/rgbd_sequence.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  Sequence::Sequence() :
    Serializable()
  {
  }

  void Sequence::serialize(std::ostream& out) const
  {
    ROS_FATAL_STREAM("Use save() instead." << flush);
    abort();
  }

  void Sequence::deserialize(std::istream& in)
  {
    ROS_FATAL_STREAM("Use load() instead." << flush);
    abort();
  }

  void Sequence::save(const std::string& dir) const
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
    for(size_t i = 0; i < pcds_.size(); ++i) {
      ostringstream oss;
      oss << "clk" << setw(4) << setfill('0') << i << ".clk";
      ofstream fs( (dir+"/" +oss.str()).c_str());
      fs.precision(10);
      fs.setf(ios::fixed,ios::floatfield);
      fs << pcds_[i]->header.stamp * 1e-9  << endl;
      fs.close();
    }
  }

  void Sequence::load(const std::string& dir)
  {
    // -- Get filenames.
    vector<string> img_names;
    vector<string> pcd_names;
    vector<string> clk_names;
    
    bfs::recursive_directory_iterator it(dir), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      ROS_ASSERT(is_regular_file(p));
      if(p.leaf().substr(0, 3).compare("img") == 0 && bfs::extension(p).compare(".png") == 0)
        img_names.push_back(p.string());
      else if(bfs::extension(p).compare(".pcd") == 0)
        pcd_names.push_back(p.string());
      else if(bfs::extension(p).compare(".clk") == 0)
        clk_names.push_back(p.string());
    }
    ROS_ASSERT(img_names.size() == pcd_names.size());
    ROS_ASSERT(img_names.size() == clk_names.size());

    // -- Sort all filenames.
    sort(img_names.begin(), img_names.end());
    sort(pcd_names.begin(), pcd_names.end());
    sort(clk_names.begin(), clk_names.end());

    // -- Load images and pointclouds.
    imgs_.resize(img_names.size());
    pcds_.resize(pcd_names.size());
    for(size_t i = 0; i < img_names.size(); ++i) {
      imgs_[i] = cv::imread(img_names[i], 1);
      pcds_[i] = Cloud::Ptr(new Cloud());
      pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_names[i], *pcds_[i]);
      ifstream fs(clk_names[i].c_str());
      double stamp;
      fs >> stamp;
      fs.close();
      pcds_[i]->header.stamp = ( stamp ) * 1e9;
    }
  }
  
  Sequence::Sequence(const Sequence& seq)
  {
    ROS_ASSERT(seq.imgs_.size() == seq.pcds_.size());
    imgs_.resize(seq.imgs_.size());
    pcds_.resize(seq.imgs_.size());

    for(size_t i = 0; i < imgs_.size(); ++i) {
      imgs_[i] = seq.imgs_[i].clone();
      pcds_[i] = seq.pcds_[i]->makeShared();
    }
  }
  
  Sequence& Sequence::operator=(const Sequence& seq)
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
                     std::vector<Sequence::Ptr>* sequences)
  {
    size_t prev_num_seq = sequences->size();
    
    bfs::directory_iterator end_itr; // default construction yields past-the-end
    for(bfs::directory_iterator itr(path); itr != end_itr; ++itr) {
      string p = itr->path().string();
      if(!bfs::is_directory(p))
        continue;

      Sequence::Ptr seq(new Sequence());
      seq->load(p);
      ROS_DEBUG_STREAM("Loaded sequence " << p << " with  " << seq->imgs_.size() << " frames.");
      sequences->push_back(seq);
    }

    // -- If it was just a single sequence directory, load that.
    if(sequences->size() == prev_num_seq) { 
      Sequence::Ptr seq(new Sequence());
      seq->load(path);
      ROS_DEBUG_STREAM("Loaded sequence " << path << " with  " << seq->imgs_.size() << " frames.");
      sequences->push_back(seq);
    }
  }

  void zthresh(Cloud::Ptr pcd, double max_z)
  {
    for(size_t i = 0; i < pcd->size(); ++i) {
      if(pcd->at(i).z > max_z) {
        pcd->at(i).x = std::numeric_limits<float>::quiet_NaN();
        pcd->at(i).y = std::numeric_limits<float>::quiet_NaN();
        pcd->at(i).z = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }

  
} // namespace rgbd
