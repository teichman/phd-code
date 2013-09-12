#ifndef TRACKER_H
#define TRACKER_H

#include <sentinel/Foreground.h>
#include <sentinel/Background.h>

class Tracker
{
public:
  
  
  Tracker(size_t max_track_length);
  void update(sentinel::Foreground msg);
};

#endif // TRACKER_H
