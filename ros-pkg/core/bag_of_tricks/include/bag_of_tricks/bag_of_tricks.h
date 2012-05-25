#ifndef BAG_OF_TRICKS_H
#define BAG_OF_TRICKS_H

#include <map>
#include <assert.h>

#define SHOW(x) #x << ": " << x

// A std::map without the annoying const problem.
template<typename T, typename S>
class Dictionary : public std::map<T, S>
{
public:
  const S& operator[](const T& key) const
  {
    assert(count(key));
    return find(key)->second;
  }

  S& operator[](const T& key)
  {
    return std::map<T, S>::operator[](key);
  }
};

#endif // BAG_OF_TRICKS_H
