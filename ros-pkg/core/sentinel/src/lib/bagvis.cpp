#include <sentinel/bagvis.h>
#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/date_time.hpp>

using namespace std;
using namespace rosbag;
using namespace sentinel;
namespace bpt = boost::posix_time;

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
  terminating_(false),
  paused_(false),
  idx_(false),
  max_buffer_size_(max_buffer_size)
{
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
  reconstructor_.update(msg);
  ptime_ = msg->header.stamp.toBoost();
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

void BagVis::addTimestamp(cv::Mat3b img, boost::posix_time::ptime ptime) const
{
  // Create a time_zone_ptr for the desired time zone and use it to create a local_date_time
  boost::local_time::time_zone_ptr zone(new boost::local_time::posix_time_zone("PST"));
  boost::local_time::local_date_time dt_with_zone(ptime_, zone);

  ostringstream oss;

  // Set the formatting facet on the stringstream and print the local_date_time to it.
  // Ownership of the boost::local_time::local_time_facet object goes to the created std::locale object.
  oss.imbue(locale(cout.getloc(), new boost::local_time::local_time_facet("%Y-%m-%d %H:%M:%S UTC%Q")));
  oss << dt_with_zone;
  
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

