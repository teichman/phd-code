#ifndef BAGVIS_H
#define BAGVIS_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sentinel/reconstructor.h>
#include <bag_of_tricks/lockable.h>

//! Only a reverse buffer for now, so you can rewind.
class BufferingBagViewer : public SharedLockable
{
public:
  BufferingBagViewer(std::string path,
                     std::vector<std::string> topics,
                     size_t max_buffer_size);
  ~BufferingBagViewer() { delete bag_; delete view_; }

  //! Loads more messages if necessary.
  //! If you try to rewind too far, it will just stop at the furthest
  //! point back in the buffer.
  void increment(int num);
  const rosbag::MessageInstance& msg() const { return buffer_[idx_]; }

  
protected:
  rosbag::Bag* bag_;
  rosbag::View* view_;
  rosbag::View::iterator it_;
  std::deque<rosbag::MessageInstance> buffer_;
  size_t max_buffer_size_;
  int idx_;
  size_t num_to_read_;
  
  //! Sets idx_ to be at the end of the buffer.
  void read(int num);
};

class BagVis
{
public:
  BagVis(std::string path);
  void run();
  ~BagVis() { delete bag_; }

protected:
  BufferingBagViewer* bag_;
  Reconstructor reconstructor_;
  bool terminating_;
  bool paused_;
  size_t idx_;
  
  void handleKeypress(char key);
  void read(size_t num);
  void increment(int num);
  void handleMessage(const rosbag::MessageInstance& msg);
};

#endif // BAGVIS_H
