#ifndef LOCKABLE_H
#define LOCKABLE_H

#include <pthread.h>
#include <errno.h>

class Lockable
{
public:
  pthread_mutex_t mutex_;
    
  Lockable();
  void lock();
  void unlock();
  bool trylock();
};

#endif // LOCKABLE_H
