#ifndef XPL_CALIBRATION_UTILITY_FUNCTIONS_H_
#define XPL_CALIBRATION_UTILITY_FUNCTIONS_H_

#include <vector>
#include <string>
#include <algorithm>
#include <map>

template<class T>
bool ascending_single (T i, T j) { return i < j; }
template<class T>
bool descending_single (T i, T j) { return i > j; }
template<class T>
bool ascending_pair ( std::pair<T,size_t> i, std::pair<T,size_t> j) { return i.first < j.first; }
template<class T>
bool descending_pair ( std::pair<T,size_t> i, std::pair<T,size_t> j) { return i.first > j.first; }

const static bool ASCENDING = true;
const static bool DESCENDING = false;

template<class T>
void sortv(const std::vector<T> &inp, std::vector<T> &outp, bool ascending=true)
{
  outp = inp;
  bool (*fxn)(T, T);
  if(ascending) fxn = ascending_single<T>;
  else fxn = descending_single<T>;
  std::sort(outp.begin(), outp.end(), fxn);
}
template<class T>
void sortv(const std::vector<T> &inp, std::vector<T> &outp, std::vector<size_t> &indices, bool ascending=true)
{
  //Populate struct
    std::vector<std::pair<T,size_t> > sorted_pairs;
  for(size_t i = 0; i < inp.size(); i++)
    sorted_pairs.push_back(std::pair<T,size_t>(inp[i],i));
  bool (*fxn)(std::pair<T,size_t>,std::pair<T,size_t>);
  if(ascending) fxn = ascending_pair<T>;
  else fxn = descending_pair<T>;
  std::sort(sorted_pairs.begin(), sorted_pairs.end(), fxn );
  outp.resize(sorted_pairs.size());
  indices.resize(sorted_pairs.size());
  for(size_t i = 0; i < sorted_pairs.size(); i++){
    outp[i] = sorted_pairs[i].first;
    indices[i] = sorted_pairs[i].second;
  }
}
template<class K, class V> void 
keyValueLists(const std::map<K,V> &dict, std::vector<K> &keys, std::vector<V> &values)
{
  typename std::map<K,V>::const_iterator it;
  for(it = dict.begin(); it != dict.end(); ++it)
  {
    keys.push_back(it->first);
    values.push_back(it->second);
  }
}

#endif//XPL_CALIBRATION_UTILITY_FUNCTIONS_H_

