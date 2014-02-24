#ifndef BINARY_BLOB_MESSAGE_H
#define BINARY_BLOB_MESSAGE_H

#include <blob/BinaryBlob.h>
#include <serializable/serializable.h>

namespace blob
{
  void toBinaryBlob(const Serializable& ser, blob::BinaryBlob* msg);
  void fromBinaryBlob(const blob::BinaryBlob& msg, Serializable* ser);
}

#endif // BINARY_BLOB_MESSAGE_H
