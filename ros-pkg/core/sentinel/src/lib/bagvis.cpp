#include <sentinel/bagvis.h>
#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace rosbag;
using namespace sentinel;

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
}

void BagVis::handleBackgroundMessage(Background::ConstPtr msg)
{
  reconstructor_.update(msg);    
}

void BagVis::handleMessage(const MessageInstance& msg)
{
  int height = 0, width = 0;
  Foreground::ConstPtr fg = msg.instantiate<Foreground>();
  if(fg) {
    handleForegroundMessage(fg);
    height = fg->height;
    width = fg->width;
  }
  Background::ConstPtr bg = msg.instantiate<Background>();
  if(bg) {
    handleBackgroundMessage(bg);
    height = bg->height;
    width = bg->width;
  }
  ROS_ASSERT(height != 0);

  if(reconstructor_.img_.rows == 0)
    buffer_.push_back(cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0)));
  else
    buffer_.push_back(reconstructor_.img_.clone());
  
  if(buffer_.size() > max_buffer_size_)
    buffer_.pop_front();

  idx_ = buffer_.size() - 1;
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

