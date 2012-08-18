#include <rgbd_sequence/stream_sequence.h>
#include <eigen_extensions/eigen_extensions.h>
#define USE_DEFAULT_CALIBRATION (getenv("USE_DEFAULT_CALIBRATION") ? atoi(getenv("USE_DEFAULT_CALIBRATION")) : 0)
#define LOAD_LEGACY (getenv("LOAD_LEGACY") ? atoi(getenv("LOAD_LEGACY")) : 0)
using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  StreamSequence::StreamSequence() :
    Serializable(),
    save_dir_("."),
    initialized_calibration_(false)
  {
    if(USE_DEFAULT_CALIBRATION){
      cout << "Using default calibration" << endl;
    } else{
      cout << "Using saved calibration" << endl;
    }
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
      double fx, fy, cx, cy, timestamp;
      loadImage(save_dir_, i, img);
      loadDepth(save_dir_, i, depth, fx, fy, cx, cy, timestamp);
      ROS_ASSERT(timestamp == timestamps_[i]);
      saveFrame(dir, i, img, depth, fx, fy, cx, cy, timestamp );
    }
    //TODO modify save_dir_?
  }

  void StreamSequence::saveFrame(const string &dir, size_t frame, 
      const Mat3b &img, const DepthMat &depth, 
      double fx, double fy, double cx, double cy, 
      double timestamp){
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
    outfile.write((char*)&fx, sizeof(double));
    outfile.write((char*)&fy, sizeof(double));
    outfile.write((char*)&cx, sizeof(double));
    outfile.write((char*)&cy, sizeof(double));
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
    // Load one cloud, for calibration parameters
    cout << "Initializing calibration parameters" << endl;
    DepthMat depth;
    double timestamp;
    loadDepth(dir, 0, depth, fx_, fy_, cx_, cy_, timestamp);
    initialized_calibration_ = true;
    cout << "Done" << endl;
    //Update save_dir_
    save_dir_ = dir;
  }

  void StreamSequence::loadImage(const string &dir, size_t frame, Mat3b &img) const
  {
    img = cv::imread(dir+"/"+img_names_[frame], 1);
  }
  
  void StreamSequence::loadDepth(const string &dir, size_t frame,
                 DepthMat &depth, double &fx, double &fy, double &cx, double &cy, double &timestamp ) const
  {
      ifstream infile;
      infile.open((dir+"/"+dpt_names_[frame]).c_str());
      eigen_extensions::deserialize(infile, &depth);
      if(!LOAD_LEGACY){
        infile.read((char*)&fx, sizeof(double));
        infile.read((char*)&fy, sizeof(double));
        infile.read((char*)&cx, sizeof(double));
        infile.read((char*)&cy, sizeof(double));
        infile.close();
      } else{
        string calibration_file = dir+"/calib.yaml";
        if (bfs::exists(calibration_file)){
          cv::FileStorage fs( calibration_file, cv::FileStorage::READ );
          cv::Mat1f camera_matrix;
          fs["camera_matrix"] >> camera_matrix;
          fx = camera_matrix(0,0);
          cx = camera_matrix(0,2);
          fy = camera_matrix(1,1);
          cy = camera_matrix(1,2);
        }
        else{
          infile.read((char*)&fx, sizeof(double));
          fy = fx;
          cx = 320;
          cy = 240;
        }
      }
      timestamp = timestamps_[frame]; //In memory already
  }

  size_t StreamSequence::seek(double timestamp, double* dt) const
  {
    ROS_ASSERT(!timestamps_.empty());
    
    // TODO: This could be much faster than linear search.
    size_t nearest = 0;
    *dt = numeric_limits<double>::max();
    for(size_t i = 0; i < timestamps_.size(); ++i) {
      double d = timestamp - timestamps_[i];
      if(fabs(d) < *dt) {
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
    
  Mat1w StreamSequence::getDepthRaw(size_t frame) const
  {
    double fx, fy, cx, cy;
    return getDepthRaw(frame, fx, fy, cx, cy);
  }

  Mat1w StreamSequence::getDepthRaw(size_t frame, double &fx, double &fy, double &cx, double &cy) const
  {
    DepthMat depth_mat;
    double timestamp;
    loadDepth(save_dir_, frame, depth_mat, fx, fy, cx, cy, timestamp);
    Mat1w cv_mat(depth_mat.rows(), depth_mat.cols());
#pragma omp parallel for
    for(int i = 0; i < depth_mat.rows(); i++)
    {
#pragma omp parallel for
      for(int j = 0; j < depth_mat.cols(); j++)
      {
        cv_mat(i,j) = depth_mat(i,j);
      }
    }
    return cv_mat;
  }
    
  void StreamSequence::getIntrinsics(size_t frame, double &fx, double &fy, double &cx, double &cy) const
  {
    DepthMat depth_mat;
    double timestamp;
    loadDepth(save_dir_, frame, depth_mat, fx, fy, cx, cy, timestamp);
  }


  Cloud::Ptr StreamSequence::getCloud(size_t frame, double f) const
  {
    ROS_ASSERT(frame < dpt_names_.size());
    DepthMat depth_mat;
    cv::Mat3b img;
    double fx, fy, cx, cy , timestamp;
    loadImage(save_dir_, frame, img);
    loadDepth(save_dir_, frame, depth_mat, fx, fy, cx, cy, timestamp);
    if(f != 0)
      fx = fy = f;
    
    if(USE_DEFAULT_CALIBRATION){
      fx = fy = 525;
      cx = depth_mat.cols()/2;
      cy = depth_mat.rows()/2;
    }
    Cloud::Ptr cloud (new Cloud);
    cloud->height = depth_mat.rows();
    cloud->width = depth_mat.cols();
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
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
          pt.x = (x-cx) * pt.z / fx;
          pt.y = (y-cy) * pt.z / fy;
          pt.b = img(y,x)[0];
          pt.g = img(y,x)[1];
          pt.r = img(y,x)[2];
        }
      }
    }
    //Finally, get timestamp
    cloud->header.stamp.fromSec(timestamp);

    if(getenv("MAX_Z"))
      zthresh(cloud, atof(getenv("MAX_Z")));

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
      double fx, double fy, double cx, double cy, double timestamp)
  {
    saveFrame(save_dir_, timestamps_.size(), img, depth, fx, fy, cx, cy, timestamp );
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


