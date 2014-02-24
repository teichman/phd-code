#ifndef BINARY_BLOB_MESSAGE_H
#define BINARY_BLOB_MESSAGE_H

#include <binary_blob_message/BinaryBlob.h>
#include <serializable/serializable.h>

void toBinaryBlobMessage(const Serializable& ser, binary_blob_message::BinaryBlob* msg);
void fromBinaryBlobMessage(const binary_blob_message::BinaryBlob& msg, Serializable* ser);

#endif // BINARY_BLOB_MESSAGE_H
