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
  //tracker_.visualize_ = true;
  
  std::vector<string> topics;
  topics.push_back("/foreground");
  topics.push_back("/background");

  bag_ = new Bag;
  bag_->open(path, bagmode::Read);
  view_ = new View(*bag_, TopicQuery(topics));
  it_ = view_->begin();

  //cv::namedWindow("Visualization", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
}

void BagVis::handleForegroundMessage(Foreground::ConstPtr msg)
{
  ptime_ = msg->header.stamp.toBoost();
  //reconstructor_.update(msg);
  tracker_.update(msg);

  // cv::Mat3b img = cv::Mat3b(cv::Size(320, 240), cv::Vec3b(127, 127, 127));
  // tracker_.draw(img);
  // cv::Mat3b img_scaled;
  // cv::resize(img, img_scaled, img.size() * 2, cv::INTER_NEAREST);
  // cv::imshow("tracks", img_scaled);
  // cv::waitKey(2);
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

  cv::Mat3b vis = reconstructor_.stylizedImage();
  tracker_.draw(vis);
  cv::Mat3b vis_scaled;
  cv::resize(vis, vis_scaled, vis.size() * 2, cv::INTER_NEAREST);
  buffer_.push_back(vis_scaled);
  overlayTimestamp(ptime_, buffer_.back());
  if(buffer_.size() > max_buffer_size_)
    buffer_.pop_front();

  idx_ = buffer_.size() - 1;
}

void BagVis::overlayTracks(cv::Mat3b track_img, cv::Mat3b img) const
{
  ROS_ASSERT(track_img.rows == img.rows);
  for(int i = 0; i < img.rows * img.cols; ++i)
    if(track_img(i) != cv::Vec3b(127, 127, 127))
      img(i) = track_img(i);
}

void BagVis::overlayTimestamp(bpt::ptime ptime, cv::Mat3b img) const
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
      cv::imshow("Visualization", buffer_[idx_]);
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
    if(it_ == view_->end()) {
      terminating_ = true;
      break;
    }
    handleMessage(*it_);
  }
}

