#include <rgbd_sequence/stream_sequence.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  StreamSequence::StreamSequence()
  {
  }
  
  void StreamSequence::save() const
  {
    ROS_ASSERT(model_.initialized());
    model_.save(root_path_ + "/primesense_model");

    ROS_ASSERT(timestamps_.size() == clk_names_.size());
    for(size_t i = 0; i < timestamps_.size(); ++i) { 
      ofstream fs((root_path_ + "/" + clk_names_[i]).c_str());
      fs.precision(10);
      fs.setf(ios::fixed, ios::floatfield);
      fs << timestamps_[i] << endl;
      fs.close();
    }
  }

  void StreamSequence::init(const std::string& root_path)
  {
    ROS_ASSERT(!bfs::exists(root_path));
    root_path_ = root_path;
    bfs::create_directory(root_path_);
    load(root_path_);
  }

  void StreamSequence::load(const std::string& dir)
  {
    root_path_ = dir;
    model_.load(root_path_ + "/primesense_model");
    ROS_ASSERT(model_.initialized());
    
    // -- Build filename index.
    img_names_.clear();
    dpt_names_.clear();
    clk_names_.clear();
    bfs::recursive_directory_iterator it(root_path_), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      ROS_ASSERT(is_regular_file(p));
      if(p.leaf().substr(0, 3).compare("img") == 0 && bfs::extension(p).compare(".ppm") == 0)
	img_names_.push_back(p.leaf());
      else if(bfs::extension(p).compare(".eig") == 0)
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
      ifstream fs((root_path_ + "/" + clk_names_[i]).c_str());
      ROS_ASSERT(fs.is_open());
      fs >> timestamps_[i];
      fs.close();
    }
  }
  
  void StreamSequence::writeFrame(const Frame& frame)
  {
    ostringstream oss;    
    size_t idx = size();
    
    // -- Write image
    vector<int> params;
    oss << "img" << setw(5) << setfill('0') << idx << ".ppm";
    cv::imwrite(root_path_ + "/" + oss.str(), frame.img_, params);
    img_names_.push_back(oss.str());

    // -- Write depth
    oss.str("");
    oss << "dpt" << setw(5) << setfill('0') << idx << ".eig";
    eigen_extensions::save(*frame.depth_, root_path_ + "/" + oss.str());
    dpt_names_.push_back(oss.str());

    // -- Write timestamp and add to index
    oss.str("");
    oss << "clk" << setw(5) << setfill('0') << idx << ".clk";
    ofstream fs((root_path_ + "/" + oss.str()).c_str());
    fs.precision(10);
    fs.setf(ios::fixed, ios::floatfield);
    fs << frame.timestamp_ << endl;
    fs.close();
    clk_names_.push_back(oss.str());
    
    timestamps_.push_back(frame.timestamp_);
  }

  void StreamSequence::readFrame(size_t idx, Frame* frame) const
  {
    frame->img_ = cv::imread(root_path_ + "/" + img_names_[idx], 1);
    frame->depth_ = DepthMatPtr(new DepthMat);
    eigen_extensions::load(root_path_ + "/" + dpt_names_[idx], frame->depth_.get());
    frame->timestamp_ = timestamps_[idx];
  }

  void StreamSequence::readFrame(double timestamp, double* dt, Frame* frame) const
  {
    size_t idx = seek(timestamp, dt);
    readFrame(idx, frame);    
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


