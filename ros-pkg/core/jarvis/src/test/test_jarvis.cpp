#include <jarvis/descriptor_pipeline.h>
#include <gtest/gtest.h>

using namespace std;

Blob randomBlob()
{
  Blob blob;
  blob.frame_id_ = 42;
  blob.sensor_timestamp_ = 2934.3049;
  blob.wall_timestamp_.fromNSec(rand() % 1000);
  blob.width_ = 640;
  blob.height_ = 480;

  int num = 83;

  blob.indices_.resize(num);
  for(size_t i = 0; i < blob.indices_.size(); ++i)
    blob.indices_[i] = rand() % 10000;

  blob.color_.resize(num * 3);
  for(size_t i = 0; i < blob.color_.size(); ++i)
    blob.color_[i] = rand() % 256;

  blob.depth_.resize(num);
  for(size_t i = 0; i < blob.depth_.size(); ++i)
    blob.depth_[i] = rand() % 10000;

  return blob;
}

TEST(Blob, Serialization)
{
  Blob blob = randomBlob();
  string path = "blob";
  blob.save(path);

  Blob blob2;
  blob2.load(path);

  EXPECT_TRUE(blob.frame_id_ == blob2.frame_id_);
  EXPECT_TRUE(blob.sensor_timestamp_ == blob2.sensor_timestamp_);
  EXPECT_TRUE(blob.wall_timestamp_.toNSec() == blob2.wall_timestamp_.toNSec());
  EXPECT_TRUE(blob.width_ == blob2.width_);
  EXPECT_TRUE(blob.height_ == blob2.height_);
  
  EXPECT_TRUE(blob.indices_.size() == blob2.indices_.size());
  for(size_t i = 0; i < blob2.indices_.size(); ++i)
    EXPECT_TRUE(blob.indices_[i] == blob2.indices_[i]);

  EXPECT_TRUE(blob.color_.size() == blob2.color_.size());
  for(size_t i = 0; i < blob2.color_.size(); ++i)
    EXPECT_TRUE(blob.color_[i] == blob2.color_[i]);
  
  EXPECT_TRUE(blob.depth_.size() == blob2.depth_.size());
  for(size_t i = 0; i < blob2.depth_.size(); ++i)
    EXPECT_TRUE(blob.depth_[i] == blob2.depth_[i]);
}

TEST(DescriptorPipeline, GraphViz)
{
  DescriptorPipeline dp;
  dp.pl_.writeGraphviz("graphviz");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
