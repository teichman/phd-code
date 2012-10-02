#include <gtest/gtest.h>
#include <pcl/visualization/cloud_viewer.h>
#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/primesense_model.h>

using namespace std;
using namespace rgbd;

TEST(Frame, Serialization)
{
  StreamSequence sseq;
  if(!getenv("TESTSEQ")) { 
    cout << "TESTSEQ is not set.  Skipping." << endl;
  }
  else { 
    sseq.load(getenv("TESTSEQ"));
    Frame frame;
    sseq.readFrame(5, &frame);
    frame.save("testframe");
    Frame frame2;
    frame2.load("testframe");
    EXPECT_TRUE(frame2.img_.rows == frame.img_.rows);
    EXPECT_TRUE(frame2.depth_->coeffRef(13, 13) == frame.depth_->coeffRef(13, 13));
    EXPECT_TRUE(frame2.timestamp_ == frame.timestamp_);
    EXPECT_TRUE(frame2.img_(13, 13)[1] == frame2.img_(13, 13)[1]);
  }
}

TEST(StreamSequence, loads)
{
  StreamSequence sseq;
  if(!getenv("TESTSEQ")) { 
    cout << "TESTSEQ is not set.  Skipping." << endl;
  }
  else { 
    sseq.load(getenv("TESTSEQ"));
    pcl::visualization::CloudViewer vis("Matched");
    for(size_t i = 0; i < sseq.size(); ++i) {
      Cloud::Ptr pcd = sseq.getCloud(i);
      vis.showCloud(pcd);
      usleep(30 * 1000);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
