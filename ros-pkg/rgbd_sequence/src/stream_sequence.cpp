#include <rgbd_sequence/stream_sequence.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  StreamSequence::StreamSequence() :
    Serializable(),
    save_dir_(".")
  {
  }
  
  StreamSequence::StreamSequence(const string& save_dir) :
    Serializable(),
    save_dir_(save_dir)
  {
    ROS_ASSERT(!bfs::exists(save_dir_));
    bfs::create_directory(save_dir_);
  }

  void StreamSequence::serialize(std::ostream& out) const
  {
    ROS_FATAL_STREAM("Use save() instead." << flush);
    abort();
  }

  void StreamSequence::deserialize(std::istream& in)
  {
    ROS_FATAL_STREAM("Use load() instead." << flush);
    abort();
  }

  void StreamSequence::save(const std::string& dir)
  {
    if( dir == save_dir_ ){
      return; //No need to do anything at this point
    }
    ROS_ASSERT(!bfs::exists(dir));
    //TODO copy to new directory!
    bfs::create_directory(dir);
    for(size_t i = 0; i < timestamps_.size(); i++){
      Mat3b img;
      DepthMat depth;
      double focal_length, timestamp;
      loadImage(save_dir_, i, img);
      loadDepth(save_dir_, i, depth, focal_length, timestamp);
      ROS_ASSERT(timestamp == timestamps_[i]);
      saveFrame(dir, i, img, depth, focal_length, timestamp );
    }
    //TODO modify save_dir_?
  }

  void StreamSequence::saveFrame(const string &dir, size_t frame, 
      const Mat3b &img, const DepthMat &depth, 
      double focal_length, double timestamp){
    //Write image  
    ostringstream oss;
    oss << "img" << setw(4) << setfill('0') << frame << ".png";
    cv::imwrite(dir + "/" + oss.str(), img);
    if(img_names_.size() <= frame)
      img_names_.resize(frame+1);
    img_names_[frame] = oss.str();
    //Write depth
    oss.str("");
    oss << "dpt" << setw(4) << setfill('0') << frame << ".dpt";
    ofstream outfile;
    outfile.open((dir+"/"+oss.str()).c_str());
    eigen_extensions::serialize(depth, outfile);
    outfile.write((char*)&focal_length, sizeof(double));
    outfile.close();
    if(dpt_names_.size() <= frame)
      dpt_names_.resize(frame+1);
    dpt_names_[frame] = oss.str();
    //Write timestamp
    oss.str("");
    oss << "clk" << setw(4) << setfill('0') << frame << ".clk";
    ofstream fs( (dir+"/" +oss.str()).c_str());
    fs.precision(10);
    fs.setf(ios::fixed,ios::floatfield);
    fs << timestamp << endl;
    fs.close();
    if(timestamps_.size() <= frame)
      timestamps_.resize(frame+1);
    timestamps_[frame] = timestamp;
    if(clk_names_.size() <= frame)
      clk_names_.resize(frame+1);
    clk_names_[frame] = oss.str();
  }

  void StreamSequence::load(const std::string& dir)
  {
    // -- Get filenames.
    img_names_.clear();
    dpt_names_.clear();
    clk_names_.clear();
    bfs::recursive_directory_iterator it(dir), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      ROS_ASSERT(is_regular_file(p));
      if(p.leaf().substr(0, 3).compare("img") == 0 && bfs::extension(p).compare(".png") == 0)
	img_names_.push_back(p.leaf());
      else if(bfs::extension(p).compare(".dpt") == 0)
	dpt_names_.push_back(p.leaf());
      else if(bfs::extension(p).compare(".clk") == 0)
	clk_names_.push_back(p.leaf());
    }
    ROS_ASSERT(img_names_.size() == dpt_names_.size());
    ROS_ASSERT(img_names_.size() == clk_names_.size());

    // -- Sort all filenames.
    sort(img_names_.begin(), img_names_.end());
    sort(dpt_names_.begin(), dpt_names_.end());
    sort(clk_names_.begin(), clk_names_.end());

    // -- Load timestamps.
    timestamps_.resize(clk_names_.size());
    for(size_t i = 0; i < clk_names_.size(); ++i) {
      ifstream fs((dir + "/" + clk_names_[i]).c_str());
      ROS_ASSERT(fs.is_open());
      fs >> timestamps_[i];
      fs.close();
    }
    //Update save_dir_
    save_dir_ = dir;
  }

  void StreamSequence::loadImage(const string &dir, size_t frame, Mat3b &img) const
  {
    img = cv::imread(dir+"/"+img_names_[frame], 1);
  }
  
  void StreamSequence::loadDepth(const string &dir, size_t frame,
                 DepthMat &depth, double &focal_length, double &timestamp ) const
  {
      ifstream infile;
      infile.open((dir+"/"+dpt_names_[frame]).c_str());
      eigen_extensions::deserialize(infile, &depth);
      infile.read((char*)&focal_length, sizeof(double));
      infile.close();
      timestamp = timestamps_[frame]; //In memory already
  }

  size_t StreamSequence::seek(double timestamp, double* dt) const
  {
    ROS_ASSERT(!timestamps_.empty());
    
    // TODO: This could be much faster than linear search.
    size_t nearest = 0;
    *dt = numeric_limits<double>::max();
    for(size_t i = 0; i < timestamps_.size(); ++i) {
      double d = fabs(timestamp - timestamps_[i]);
      if(d < *dt) {
	*dt = d;
	nearest = i;
      }
    }

    return nearest;
  }
  
  Cloud::Ptr StreamSequence::getCloud(double timestamp, double* dt) const
  {
    return getCloud(seek(timestamp, dt));
  }
  
  Mat3b StreamSequence::getImage(double timestamp, double* dt) const
  {
    return getImage(seek(timestamp, dt));
  }
  
  Cloud::Ptr StreamSequence::getCloud(size_t frame) const
  {
    ROS_ASSERT(frame < dpt_names_.size());
    DepthMat depth_mat;
    cv::Mat3b img;
    double focal_length, timestamp;
    loadImage(save_dir_, frame, img);
    loadDepth(save_dir_, frame, depth_mat, focal_length, timestamp);
    Cloud::Ptr cloud (new Cloud);
    cloud->height = depth_mat.rows();
    cloud->width = depth_mat.cols();
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    float constant = 1.0f / focal_length;
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
    cloud->header.stamp.fromSec(timestamp);
    return cloud;
  }

  Mat3b StreamSequence::getImage(size_t frame) const
  {
    ROS_ASSERT(frame < img_names_.size());
    Mat3b img;
    loadImage(save_dir_, frame, img);
    return img;
  }


  void StreamSequence::addFrame( const Mat3b &img, const DepthMat &depth, 
      double focal_length, double timestamp)
  {
    saveFrame(save_dir_, timestamps_.size(), img, depth, focal_length, timestamp );
  }

  size_t StreamSequence::size() const
  {
    ROS_ASSERT(img_names_.size() == dpt_names_.size());
    ROS_ASSERT(img_names_.size() == clk_names_.size());
    ROS_ASSERT(img_names_.size() == timestamps_.size());
    return img_names_.size();
  }

  void StreamSequence::applyTimeOffset(double dt)
  {
    for(size_t i = 0; i < timestamps_.size(); ++i)
      timestamps_[i] += dt;
  }
  
} // namespace rgbd


