#include <binary_blob_message/binary_blob_message.h>
#include <iostream>

using namespace std;
using namespace binary_blob_message;

void toBinaryBlobMessage(const Serializable& ser, BinaryBlob* msg)
{
  stringbuf sb;
  iostream ios(&sb);
  ser.serialize(ios);
  // Seems like this seekg should not be necessary.
  // Am I doing something wrong?
  // Maybe write does not increment stream pointer.
  ios.seekg(0, ios.end);  
  size_t num_bytes = ios.tellg();
  ios.seekg(0, ios.beg);
  msg->data.clear();
  msg->data.resize(num_bytes);
  ios.seekg(0, ios.beg);  // Go back to the beginning of the stream.
  ios.read((char*)msg->data.data(), num_bytes);
}

void fromBinaryBlobMessage(const BinaryBlob& msg, Serializable* ser)
{
  // Initialize the iostream without copying data.
  stringbuf sb;
  sb.pubsetbuf((char*)msg.data.data(), msg.data.size());
  iostream ios(&sb);
  ser->deserialize(ios);
}

