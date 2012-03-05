#include <rgbd_sequence/compact_sequence.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  CompactSequence::CompactSequence() :
    Serializable()
  {
  }

  void CompactSequence::serialize(std::ostream& out) const
  {
    ROS_FATAL_STREAM("Use save() instead." << flush);
    abort();
  }

  void CompactSequence::deserialize(std::istream& in)
  {
    ROS_FATAL_STREAM("Use load() instead." << flush);
    abort();
  }

  void CompactSequence::save(const std::string& dir) const
  {
    ROS_ASSERT(!bfs::exists(dir));
    bfs::create_directory(dir);
    //TODO write meta
    for(size_t i = 0; i < imgs_.size(); ++i) {
      ostringstream oss;
      oss << "img" << setw(4) << setfill('0') << i << ".png";
      cv::imwrite(dir + "/" + oss.str(), imgs_[i]);
    }

    for(size_t i = 0; i < depths_.size(); ++i) {
      ostringstream oss;
      oss << "dpt" << setw(4) << setfill('0') << i << ".dpt";
      ofstream outfile;
      outfile.open((dir+"/"+oss.str()).c_str());
      eigen_extensions::serialize(depths_[i], outfile);
      outfile.write((char*)&focal_length_, sizeof(double));
      outfile.close();
    }
    for(size_t i = 0; i < timestamps_.size(); ++i) {
      ostringstream oss;
      oss << "clk" << setw(4) << setfill('0') << i << ".clk";
      ofstream fs( (dir+"/" +oss.str()).c_str());
      fs.precision(10);
      fs.setf(ios::fixed,ios::floatfield);
      fs << timestamps_[i] << endl;
      fs.close();
    }
  }

  void CompactSequence::load(const std::string& dir)
  {
    // -- Get filenames.
    vector<string> img_names;
    vector<string> dpt_names;
    vector<string> clk_names;
    
    bfs::recursive_directory_iterator it(dir), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      ROS_ASSERT(is_regular_file(p));
      if(p.leaf().substr(0, 3).compare("img") == 0 && bfs::extension(p).compare(".png") == 0)
	img_names.push_back(p.string());
      else if(bfs::extension(p).compare(".dpt") == 0)
	dpt_names.push_back(p.string());
      else if(bfs::extension(p).compare(".clk") == 0)
	clk_names.push_back(p.string());
    }
    ROS_ASSERT(img_names.size() == dpt_names.size());
    ROS_ASSERT(img_names.size() == clk_names.size());

    // -- Sort all filenames.
    sort(img_names.begin(), img_names.end());
    sort(dpt_names.begin(), dpt_names.end());
    sort(clk_names.begin(), clk_names.end());

    // -- Load images and pointclouds.
    imgs_.resize(img_names.size());
    depths_.resize(dpt_names.size());
    timestamps_.resize(clk_names.size());
    for(size_t i = 0; i < img_names.size(); ++i) {
      imgs_[i] = cv::imread(img_names[i], 1);
      ifstream infile;
      infile.open(dpt_names[i].c_str());
      eigen_extensions::deserialize(infile, &depths_[i]);
      infile.read((char*)&focal_length_, sizeof(double));
      infile.close();
      ifstream fs(clk_names[i].c_str());
      fs >> timestamps_[i];
      fs.close();
    }
  }
  
  CompactSequence::CompactSequence(const CompactSequence& seq)
  {
    ROS_ASSERT(seq.imgs_.size() == seq.depths_.size());
    ROS_ASSERT(seq.imgs_.size() == seq.timestamps_.size());
    imgs_.resize(seq.imgs_.size());
    depths_.resize(seq.imgs_.size());
    timestamps_.resize(seq.imgs_.size());

    for(size_t i = 0; i < imgs_.size(); ++i) {
      imgs_[i] = seq.imgs_[i].clone();
      depths_[i] = seq.depths_[i];
      timestamps_[i] = seq.timestamps_[i];
    }
  }
  
  CompactSequence& CompactSequence::operator=(const CompactSequence& seq)
  {
    if(&seq == this)
      return *this;

    ROS_ASSERT(seq.imgs_.size() == seq.depths_.size());
    ROS_ASSERT(seq.imgs_.size() == seq.timestamps_.size());
    imgs_.resize(seq.imgs_.size());
    depths_.resize(seq.imgs_.size());
    timestamps_.resize(seq.imgs_.size());
    
    for(size_t i = 0; i < imgs_.size(); ++i) {
      imgs_[i] = seq.imgs_[i].clone();
      depths_[i] = seq.depths_[i];
      timestamps_[i] = seq.timestamps_[i];
    }

    return *this;
  }  

  Cloud::Ptr CompactSequence::getCloud(size_t frame) const
  {
    ROS_ASSERT(frame < depths_.size());
    const DepthMat& depth_mat = depths_[frame];
    const cv::Mat3b& img = imgs_[frame];
    Cloud::Ptr cloud (new Cloud);
    cloud->height = depth_mat.rows();
    cloud->width = depth_mat.cols();
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    float constant = 1.0f / focal_length_;
    register int centerX = (cloud->width >> 1 );
    int centerY = (cloud->height >> 1);
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    int pt_idx = 0;
    for(int y = 0; y < depth_mat.rows(); ++y){
      for(register int x = 0; x < depth_mat.cols(); ++x, ++pt_idx){
        Point& pt = cloud->points[pt_idx];
        unsigned short z = depth_mat(y,x);
        //Check for invalid measurements
        if (z == 0 ){ //TODO be sure all invalid are 0
          pt.x = pt.y = pt.z = bad_point;
        }
        else{
          pt.z = z * 0.001f;
          pt.x = (x-centerX) * pt.z * constant;
          pt.y = (y-centerY) * pt.z * constant;
          pt.b = img(y,x)[0];
          pt.g = img(y,x)[1];
          pt.r = img(y,x)[2];
        }
      }
    }
    //Finally, get timestamp
    cloud->header.stamp.fromSec(timestamps_[frame]);
    return cloud;
  }
} // namespace rgbd

