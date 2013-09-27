#include <pipeline/common.h>

namespace pl
{
  uint64_t hashDjb2(const char *str)
  {
    uint64_t hash = 5381;
    int c;

    // See http://www.cse.yorku.ca/~oz/hash.html.
    while((c = *str++))
      hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
  
    return hash;
  }

  uint64_t hash(const YAML::Node& node)
  {
    return hashDjb2(YAML::Dump(node).c_str());
  }
}
