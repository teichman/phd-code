#include <gtest/gtest.h>
#include <pcl/visualization/cloud_viewer.h>
#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/projector.h>

using namespace std;
using namespace rgbd;

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

TEST(Projector, Projector)
{
  StreamSequence sseq;
  if(!getenv("TESTSEQ")) { 
    cout << "TESTSEQ is not set.  Skipping." << endl;
  }
  else {
    sseq.load(getenv("TESTSEQ"));
    Cloud pcd = *sseq.getCloud(1);
    Projector proj;
    proj.fx_ = sseq.fx_;
    proj.fy_ = sseq.fy_;
    proj.cx_ = sseq.cx_;
    proj.cy_ = sseq.cy_;
    proj.width_ = pcd.width;
    proj.height_ = pcd.height;

    int idx = 0;
    for(size_t v = 0; v < pcd.height; ++v) {
      for(size_t u = 0; u < pcd.width; ++u, ++idx) {
	const Point& pt = pcd[idx];
	if(!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z))
	  continue;

	double dx = pt.x - (pt.z * (u - sseq.cx_) / sseq.fx_);
	double dy = pt.y - (pt.z * (v - sseq.cy_) / sseq.fy_);
	//cout << "z: " << pt.z << ", dx: " << dx << ", dy: " << dy << endl;
	ASSERT_TRUE(fabs(dx) < 1e-6);
	ASSERT_TRUE(fabs(dy) < 1e-6);
      }
    }
    
    Frame frame;
    proj.cloudToFrame(pcd, &frame);
    Cloud pcd2;
    proj.frameToCloud(frame, &pcd2);
    ASSERT_TRUE(pcd2.size() == pcd.size());
    for(size_t i = 0; i < pcd.size(); ++i) {
      // cout << "Finite: " << pcl_isfinite(pcd[i].x) << " " << pcl_isfinite(pcd2[i].x) << endl;
      // cout << "Diffs: " << setprecision(16)
      // 	   << fabs(pcd[i].x - pcd2[i].x) << ", "
      // 	   << fabs(pcd[i].y - pcd2[i].y) << ", "
      // 	   << fabs(pcd[i].z - pcd2[i].z) << ", "
      // 	   << fabs(pcd[i].r - pcd2[i].r) << ", "
      // 	   << fabs(pcd[i].g - pcd2[i].g) << ", "
      // 	   << fabs(pcd[i].b - pcd2[i].b) << endl;

      ASSERT_TRUE((!pcl_isfinite(pcd[i].x) && !pcl_isfinite(pcd2[i].x)) ||
		  ((fabs(pcd[i].x - pcd2[i].x) < 1e-6) &&
		   (fabs(pcd[i].y - pcd2[i].y) < 1e-6) &&
		   (fabs(pcd[i].z - pcd2[i].z) < 1e-6) &&
		   (pcd[i].r == pcd2[i].r) &&
		   (pcd[i].g == pcd2[i].g) &&
		   (pcd[i].b == pcd2[i].b)));
    }

    Frame frame2;
    proj.cloudToFrame(pcd2, &frame2);
    ASSERT_TRUE(frame2.depth_.rows() == frame.depth_.rows());
    ASSERT_TRUE(frame2.depth_.cols() == frame.depth_.cols());
    ASSERT_TRUE(frame2.img_.rows == frame.img_.rows);
    ASSERT_TRUE(frame2.img_.cols == frame.img_.cols);
    ASSERT_TRUE(fabs(frame2.timestamp_ - frame.timestamp_) < 1e-6);

    for(int v = 0; v < frame2.depth_.rows(); ++v) {
      for(int u = 0; u < frame2.depth_.cols(); ++u) {
	double dd = (double)(frame2.depth_(v, u) - frame.depth_(v, u)) / 1000.0;
	//cout << dd << endl;
	ASSERT_TRUE(fabs(dd) < 0.002);
	ASSERT_TRUE(frame2.img_(v, u)[0] == frame.img_(v, u)[0]);
	ASSERT_TRUE(frame2.img_(v, u)[1] == frame.img_(v, u)[1]);
	ASSERT_TRUE(frame2.img_(v, u)[2] == frame.img_(v, u)[2]);
      }
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
