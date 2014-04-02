#ifndef INSTANCE_SERIALIZER_H_
#define INSTANCE_SERIALIZER_H_

#include <string>
#include "boost/shared_ptr.hpp"

class CustomSerializer
{
public:
  typedef boost::shared_ptr<CustomSerializer> Ptr;

  virtual std::string name() const = 0;
  virtual void serialize(const boost::any& raw, std::ostream& out) const = 0;
  virtual void deserialize(std::istream& in, boost::any* raw) const = 0;
};

// PCD is PointCloud data.
class PCDSerializer : public CustomSerializer
{
public:
  typedef boost::shared_ptr<PCDSerializer> Ptr;

  std::string name() const { return "PCDSerializer"; }
  void serialize(const boost::any& raw, std::ostream& out) const;
  void deserialize(std::istream& in, boost::any* raw) const;
};

//! Special custom serializer that will skip deserializing any custom data
//! that is present.
class EmptyCustomSerializer : public CustomSerializer
{
public:
  typedef boost::shared_ptr<EmptyCustomSerializer> Ptr;

  std::string name() const { return "EmptyCustomSerializer"; }
  void serialize(const boost::any& raw, std::ostream& out) const {}
  void deserialize(std::istream& in, boost::any* raw) const {}
};

//! Special custom serializer that will skip deserializing any custom data
//! that is present.  Writes are not allowed.  This is useful when you
//! want to read in a track dataset, don't need the raw_ data, and
//! want to be sure that you don't clobber the raw_ data with a write.
class ReadOnlyEmptyCustomSerializer : public CustomSerializer
{
public:
  typedef boost::shared_ptr<EmptyCustomSerializer> Ptr;

  std::string name() const { return "EmptyCustomSerializer"; }
  void serialize(const boost::any& raw, std::ostream& out) const
  {
    std::cerr << "ReadOnlyEmptyCustomSerializer cannot serialize." << std::endl;
    ROS_ASSERT(0);
  }
  void deserialize(std::istream& in, boost::any* raw) const {}
};

//! Special custom serializer that will deserialize custom data as
//! a binary blob and serialize it under its original name.
class PassthroughCustomSerializer : public CustomSerializer
{
public:
  typedef boost::shared_ptr<PassthroughCustomSerializer> Ptr;

  struct Data
  {
    typedef boost::shared_ptr<Data> Ptr;
    std::string name_;
    std::vector<uint8_t> data_;
  };

  std::string name() const { return "PassthroughCustomSerializer"; }
  //! Writes the original name, num bytes, and the binary blob.
  void serialize(const boost::any& raw, std::ostream& out) const;
  void deserialize(std::istream& in, boost::any* raw) const { ROS_ASSERT(0); }
  void deserialize(std::string original_name, size_t num_bytes,
                   std::istream& in, boost::any* raw) const;
};

class ReferenceSavingCustomSerializer : public CustomSerializer
{
public:
  typedef boost::shared_ptr<ReferenceSavingCustomSerializer> Ptr;

  std::string name() const { return "ReferenceSavingCustomSerializer"; }
  void serialize(const boost::any& raw, std::ostream& out) const {
    ROS_FATAL("ReferenceSavingCustomSerializer not to be called directly");
  }
  void deserialize(std::istream& in, boost::any* raw) const {
    ROS_FATAL("ReferenceSavingCustomSerializer not to be called directly");
  }
};



#endif /* INSTANCE_SERIALIZER_H_ */
