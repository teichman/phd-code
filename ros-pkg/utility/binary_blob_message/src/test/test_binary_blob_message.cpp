#include <gtest/gtest.h>
#include <stdint.h>
#include <binary_blob_message/compression.h>
#include <timer/timer.h>

using namespace std;

TEST(BinaryBlobMessage, compression)
{
  // Get a vector of raw data.
  int num = 1e5;
  vector<uint8_t> raw(num);
  for(size_t i = 0; i < raw.size(); ++i)
    raw[i] = rand() % 255;

  // Set random blocks to a random number.
  // This is what the compression algorithm should be able to work with.
  int chunk_size = 100;
  for(size_t i = 0; i < raw.size(); ++i)
    if(rand() % 50 == 0)
      memset(raw.data() + i, rand() % 255, min<int>(chunk_size, raw.size() - i));


  // Compress and decompress.
  cout << "Original size: " << raw.size() << endl;
  vector<uint8_t> deflated;
  HighResTimer hrt;
  hrt.reset("deflation"); hrt.start();
  blob::deflate(raw.data(), raw.size(), deflated);
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  cout << "Compressed size: " << deflated.size() << endl;
  cout << "Compression ratio: " << (double)raw.size() / deflated.size() << endl;
  vector<uint8_t> inflated;
  hrt.reset("inflation"); hrt.start();
  blob::inflate(deflated.data(), deflated.size(), inflated);
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;

  // Check that it is lossless.
  EXPECT_EQ(raw.size(), inflated.size());
  for(size_t i = 0; i < raw.size(); ++i)
    EXPECT_EQ(raw[i], inflated[i]);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
