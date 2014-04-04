#include <jarvis/blob_serializer.h>

CustomSerializer::Ptr Instance::custom_serializer_ = CustomSerializer::Ptr(new BlobSerializer);

void BlobSerializer::serialize(const boost::any& raw, std::ostream& out) const
{
  Blob::ConstPtr blob = boost::any_cast<Blob::ConstPtr>(raw);
  blob->serialize(out);
}

void BlobSerializer::deserialize(std::istream& in, boost::any* raw) const
{
  Blob::Ptr blob(new Blob);
  blob->deserialize(in);
  Blob::ConstPtr cp = blob;
  *raw = cp;
}

