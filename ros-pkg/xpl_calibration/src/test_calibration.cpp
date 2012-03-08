#include <gtest/gtest.h>
#include <xpl_calibration/object_matching_calibrator.h>

using namespace std;
using namespace rgbd;

TEST(generateTransform, generateTransform)
{
  Eigen::Affine3f trans = generateTransform(1, 2, 3, 0.5, 0.35, 0.56);
  cout << trans.matrix() << endl;

  Cloud pcd0;
  Point pt;
  pt.x = 1;
  pt.y = 1;
  pt.z = 1;
  pcd0.push_back(pt);
  Cloud pcd1;
  Cloud pcd2;
  Cloud pcd3;

  Eigen::Affine3f t1 = generateTransform(1, 2, 3, 0.5, 0.35, 0.56);
  Eigen::Affine3f t2 = generateTransform(1, 2, 3, 0.1, 0.1, 0.1);
  pcl::transformPointCloud(pcd0, pcd1, t1);
  pcl::transformPointCloud(pcd1, pcd2, t2);

  pcl::transformPointCloud(pcd0, pcd3, t2 * t1);
  cout << pcd2[0] << endl;
  cout << pcd3[0] << endl;

  EXPECT_FLOAT_EQ(pcd2[0].x, pcd3[0].x);
}



int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

