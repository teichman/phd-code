#include <jarvis/compression_helpers.h>
#include <string.h>  // memcpy

void writeToVec(size_t num, std::vector<uint8_t>* data)
{
  size_t idx = data->size();
  data->resize(data->size() + sizeof(size_t));
  memcpy(data->data() + idx, &num, sizeof(size_t));
}

size_t readFromVec(const std::vector<uint8_t>& data, size_t idx, size_t* num)
{
  memcpy(num, data.data() + idx, sizeof(size_t));
  return idx + sizeof(size_t);
}

void writeToVec(const std::vector<uint8_t>& chunk, std::vector<uint8_t>* data)
{
  // Assume that data is sufficiently large that we are not re-allocating constantly.
  size_t idx = data->size();
  data->resize(data->size() + chunk.size() + sizeof(size_t));
  size_t buf = chunk.size();
  memcpy(data->data() + idx, &buf, sizeof(size_t));
  memcpy(data->data() + idx + sizeof(size_t), chunk.data(), chunk.size());
}

size_t readFromVec(const std::vector<uint8_t>& data, size_t idx, std::vector<uint8_t>* chunk)
{
  size_t buf;
  memcpy(&buf, data.data() + idx, sizeof(size_t));
  chunk->resize(buf);
  memcpy(chunk->data(), data.data() + idx + sizeof(size_t), buf);
  return idx + buf + sizeof(size_t);
}
