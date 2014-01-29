#include <stdint.h>
#include <stddef.h>
#include <vector>

void writeToVec(size_t num, std::vector<uint8_t>* data);
size_t readFromVec(const std::vector<uint8_t>& data, size_t idx, size_t* num);
void writeToVec(const std::vector<uint8_t>& chunk, std::vector<uint8_t>* data);
size_t readFromVec(const std::vector<uint8_t>& data, size_t idx, std::vector<uint8_t>* chunk);
