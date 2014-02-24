#include <stdint.h>
#include <gtest/gtest.h>
#include <timer/timer.h>
#include <serializable/serializable.h>
#include <binary_blob_message/compression.h>
#include <binary_blob_message/binary_blob_message.h>

using namespace std;
using namespace binary_blob_message;

class Example : public Serializable
{
public:
  std::vector<uint8_t> data_;

  Example(size_t num = 0)
  {
    // Get a vector of raw data.
    data_.resize(num);
    for(size_t i = 0; i < data_.size(); ++i)
      data_[i] = rand() % 255;
    
    // Set random blocks to a random number.
    int chunk_size = 100;
    for(size_t i = 0; i < data_.size(); ++i)
      if(rand() % 50 == 0)
        memset(data_.data() + i, rand() % 255, min<int>(chunk_size, data_.size() - i));
  }
  
  void serialize(std::ostream& out) const
  {
    size_t buf = data_.size();
    cout << "serialize num: " << buf << endl;
    out.write((char*)&buf, sizeof(buf));
    out.write((char*)data_.data(), data_.size());
  }
  
  void deserialize(std::istream& in)
  {
    size_t num;
    in.read((char*)&num, sizeof(num));
    cout << "deserialize num: " << num << endl;
    data_.resize(num);
    in.read((char*)data_.data(), data_.size());
  }
};

TEST(BinaryBlobMessage, Compression)
{
  Example ex(1e5);

  // Compress and decompress.
  cout << "Original size: " << ex.data_.size() << endl;
  vector<uint8_t> deflated;
  HighResTimer hrt;
  hrt.reset("deflation"); hrt.start();
  blob::deflate(ex.data_.data(), ex.data_.size(), deflated);
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  cout << "Compressed size: " << deflated.size() << endl;
  cout << "Compression ratio: " << (double)ex.data_.size() / deflated.size() << endl;
  vector<uint8_t> inflated;
  hrt.reset("inflation"); hrt.start();
  blob::inflate(deflated.data(), deflated.size(), inflated);
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;

  // Check that it is lossless.
  EXPECT_EQ(ex.data_.size(), inflated.size());
  for(size_t i = 0; i < ex.data_.size(); ++i)
    EXPECT_EQ(ex.data_[i], inflated[i]);
}

TEST(BinaryBlobMessage, Serialization)
{
  Example ex(1e5);

  // Check serialization to and from disk.
  ex.save("example");
  Example ex2;
  ex2.load("example");
  
  EXPECT_EQ(ex.data_.size(), ex2.data_.size());
  for(size_t i = 0; i < ex.data_.size(); ++i)
    EXPECT_EQ(ex.data_[i], ex2.data_[i]);

  // Check serialization to and from BinaryBlob message.
  BinaryBlob msg;
  toBinaryBlobMessage(ex, &msg);
  Example ex3;
  fromBinaryBlobMessage(msg, &ex3);

  EXPECT_EQ(ex.data_.size(), ex3.data_.size());
  for(size_t i = 0; i < ex.data_.size(); ++i)
    EXPECT_EQ(ex.data_[i], ex3.data_[i]);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
