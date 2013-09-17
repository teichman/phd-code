#include <jarvis/bagvis.h>
#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/date_time.hpp>

using namespace std;
using namespace rosbag;
using namespace sentinel;
namespace bpt = boost::posix_time;
namespace blt = boost::local_time;  // yum

BufferingBagViewer::BufferingBagViewer(std::string path,
                                       vector<std::string> topics,
                                       size_t max_buffer_size) :
  max_buffer_size_(max_buffer_size),
  idx_(0)
{
  bag_ = new Bag;
  bag_->open(path, bagmode::Read);
  view_ = new View(*bag_, TopicQuery(topics));
  it_ = view_->begin();
  
  read(1);
}

void BufferingBagViewer::increment(int num)
{
  idx_ += num;
  idx_ = max(0, idx_);
  if(idx_ >= (int)buffer_.size()) { 
    read(idx_ - buffer_.size() + 1);
  }
}

void BufferingBagViewer::read(int num)
{
  for(int i = 0; i < num; ++i) {
    ++it_;
    buffer_.push_back(*it_);
    // cout << "Read message on topic " << buffer_.back().getTopic()
    //      << " with time: " << buffer_.back().getTime() << endl;
    if(buffer_.size() > max_buffer_size_)
      buffer_.pop_front();
  }
  
  idx_ = buffer_.size() - 1;
}


BagVis::BagVis(std::string path, size_t max_buffer_size) :
  tracker_(100),
  terminating_(false),
  paused_(false),
  idx_(false),
  max_buffer_size_(max_buffer_size)
{
  tracker_.visualize_ = true;
  
  std::vector<string> topics;
  topics.push_back("/foreground");
  topics.push_back("/background");

  bag_ = new Bag;
  bag_->open(path, bagmode::Read);
  view_ = new View(*bag_, TopicQuery(topics));
  it_ = view_->begin();
}

void BagVis::handleForegroundMessage(Foreground::ConstPtr msg)
{
  ptime_ = msg->header.stamp.toBoost();
  //reconstructor_.update(msg);
  tracker_.update(msg);

  // -- Make a visualization using the color image and foreground.
  cv::Mat3b img = cv::Mat3b(cv::Size(320, 240), cv::Vec3b(127, 127, 127));
  map<size_t, Blob::Ptr>::iterator it;
  cout << "Displaying " << tracker_.tracks_.size() << " tracks." << endl;
  for(it = tracker_.tracks_.begin(); it != tracker_.tracks_.end(); ++it) {
    //size_t track_id = it->first;
    const Blob& blob = *it->second;
    for(size_t i = 0; i < blob.indices_.size(); ++i) {
      size_t idx = blob.indices_[i];
      img(idx)[2] = blob.color_[i*3+0];
      img(idx)[1] = blob.color_[i*3+1];
      img(idx)[0] = blob.color_[i*3+2];
    }
  }

  cv::Mat3b img_scaled;
  cv::resize(img, img_scaled, img.size() * 2, cv::INTER_NEAREST);
  cv::imshow("tracks", img_scaled);
  cv::waitKey(2);
}

void BagVis::handleBackgroundMessage(Background::ConstPtr msg)
{
  reconstructor_.update(msg);
  ptime_ = msg->header.stamp.toBoost();
}

void BagVis::handleMessage(const MessageInstance& msg)
{
  Foreground::ConstPtr fg = msg.instantiate<Foreground>();
  if(fg)
    handleForegroundMessage(fg);

  Background::ConstPtr bg = msg.instantiate<Background>();
  if(bg)
    handleBackgroundMessage(bg);

  buffer_.push_back(reconstructor_.img_.clone());
  addTimestamp(buffer_.back(), ptime_);
  if(buffer_.size() > max_buffer_size_)
    buffer_.pop_front();

  idx_ = buffer_.size() - 1;
}

void BagVis::addTimestamp(cv::Mat3b img, bpt::ptime ptime) const
{
  ostringstream oss;
  const bpt::time_facet* f = new bpt::time_facet("%Y-%m-%d %H:%M:%S UTC%Q");
  oss.imbue(locale(oss.getloc(), f));
  oss << ptime;
  
  float thickness = 1.5;
  float scale = 0.5;
  cv::putText(img, oss.str(), cv::Point(10, img.rows - 10),
              cv::FONT_HERSHEY_SIMPLEX, scale,
              cv::Scalar(0, 255, 0), thickness, CV_AA);
}

void BagVis::run()
{  
  while(true) {
    if(idx_ > 0 && (size_t)idx_ < buffer_.size())
      cv::imshow("Background", buffer_[idx_]);
    char key = cv::waitKey(1);
    handleKeypress(key);

    if(terminating_)
      break;

    if(paused_) {
      usleep(1e4);
      continue;
    }
    else
      increment(1);
  }
}

void BagVis::handleKeypress(char key)
{
  switch(key) {
  case 'q':
    terminating_ = true;
    break;
  case ' ':
    paused_ = !paused_;
    break;
  case '.':
    if(paused_) 
      increment(1);
    break;
  case ',':
    if(paused_)
      increment(-1);
    break;
  case '>':
    if(paused_) 
      increment(100);
    break;
  case '<':
    if(paused_)
      increment(-100);
    break;
  case -1:
    break;
  default:
    //cout << "Unknown key " << (int)key << endl;
    break;
  }
}

void BagVis::increment(int num)
{
  idx_ += num;
  idx_ = max(0, idx_);
  if(idx_ >= (int)buffer_.size()) { 
    read(idx_ - buffer_.size() + 1);
  }
}

void BagVis::read(int num)
{
  for(int i = 0; i < num; ++i) {
    ++it_;
    handleMessage(*it_);
  }
}

