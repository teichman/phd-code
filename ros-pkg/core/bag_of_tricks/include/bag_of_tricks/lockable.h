#ifndef LOCKABLE_H
#define LOCKABLE_H

#include <pthread.h>
#include <errno.h>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

#define scopeLockRead boost::shared_lock<boost::shared_mutex> lockable_shared_lock(shared_mutex_)
#define scopeLockWrite boost::unique_lock<boost::shared_mutex> lockable_unique_lock(shared_mutex_)

class SharedLockable
{
public:
  SharedLockable();

  void lockWrite();
  void unlockWrite();
  bool trylockWrite();
  
  void lockRead();
  void unlockRead();
  bool trylockRead();

protected:
  boost::shared_mutex shared_mutex_;
};

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
