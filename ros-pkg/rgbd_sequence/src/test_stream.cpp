#include <gtest/gtest.h>
#include <pcl/visualization/cloud_viewer.h>
#include <rgbd_sequence/stream_sequence.h>

using namespace std;
using namespace rgbd;

TEST(StreamSequence, loads)
{
  StreamSequence sseq;
  EXPECT_TRUE(getenv("TESTSEQ"));
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
