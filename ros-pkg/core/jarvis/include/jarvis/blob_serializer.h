#ifndef BLOB_SERIALIZER_H
#define BLOB_SERIALIZER_H

#include <online_learning/dataset.h>
#include <jarvis/tracker.h>

class BlobSerializer : public CustomSerializer
{
  typedef boost::shared_ptr<BlobSerializer> Ptr;

  std::string name() const { return "BlobSerializer"; }
  void serialize(const boost::any& raw, std::ostream& out) const;
  void deserialize(std::istream& in, boost::any* raw) const;
};

#endif // BLOB_SERIALIZER_H
