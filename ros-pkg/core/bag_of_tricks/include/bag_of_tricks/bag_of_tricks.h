#ifndef BAG_OF_TRICKS_H
#define BAG_OF_TRICKS_H

#include <map>
#include <vector>
#include <stdlib.h>
#include <assert.h>
#include <sstream>
#include <serializable/serializable.h>

#define SHOW(x) #x << ": " << x

// A std::map without the annoying const problem.
template<typename T, typename S>
class Dictionary : public std::map<T, S>, public Serializable
{
public:
  //std::vector<int> leak_;

  const S& operator[](const T& key) const
  {
    assert((std::map<T, S>::count(key)));
    return std::map<T, S>::find(key)->second;
  }

  S& operator[](const T& key)
  {
    return std::map<T, S>::operator[](key);
  }

  void serialize(std::ostream& out) const {
    typename std::map<T, S>::const_iterator it;
    out << "Dictionary (" << std::map<T, S>::size() << ")" << std::endl;
    for(it = std::map<T, S>::begin(); it != std::map<T, S>::end(); ++it)
      out << it->first << " " << it->second << std::endl;
  }
  
  void deserialize(std::istream& in) {
    std::map<T, S>::clear();

    std::string buf;
    in >> buf;
    assert(buf.compare("Dictionary") == 0);
    in >> buf;
    size_t num = atoi(buf.substr(1, buf.size() - 1).c_str());
    getline(in, buf);

    for(size_t i = 0; i < num; ++i) {
      T first;
      S second;
      in >> first;
      in >> second;
      std::map<T, S>::operator[](first) = second;
    }
    getline(in, buf);  // Eat final newline.
  }

  std::string status(const std::string& prefix = "") const
  {
    std::ostringstream oss;
    typename std::map<T, S>::const_iterator it;
    for(it = std::map<T, S>::begin(); it != std::map<T, S>::end(); ++it)
      oss << prefix << it->first << ": " << it->second << std::endl;
    return oss.str();
  }
  
};

#endif // BAG_OF_TRICKS_H
