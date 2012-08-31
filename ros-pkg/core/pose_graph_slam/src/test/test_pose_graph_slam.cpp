#include <gtest/gtest.h>
#include <pose_graph_slam/pose_graph_slam.h>

using namespace std;
using namespace g2o;


TEST(PoseGraphSLAM, Simple)
{
  PoseGraphSLAM pgs(4);
  Matrix6d covariance = Matrix6d::Identity();

  pgs.addEdge(0, 1,
	      Vector3d(0, 1, 0),
	      Matrix3d::Identity(),
	      covariance);
  pgs.addEdge(1, 2,
	      Vector3d(0, 1, 0),
	      Matrix3d(AngleAxis<double>(M_PI / 4, Vector3d(1, 0, 0))),
	      covariance);
  pgs.addEdge(2, 3,
	      Vector3d(0, 1, 0),
	      Matrix3d(AngleAxis<double>(M_PI / 4, Vector3d(1, 0, 0))),
	      covariance);

  pgs.solve();

  for(size_t i = 0; i < pgs.numNodes(); ++i) {
    cout << "--------------------" << endl;
    cout << "Transform " << i << endl;
    cout << pgs.transform(i).matrix() << endl;
  }

  EXPECT_TRUE((pgs.transform(0).matrix() - Matrix4d::Identity()).norm() < 1e-6);
  Vector4d transformed = pgs.transform(3) * Vector4d(0, 1, 0, 1);
  EXPECT_TRUE((transformed - Vector4d(0, 2.70711, 1.70711, 1)).norm() < 1e-3);
  cout << transformed.transpose() << endl;
}

TEST(PoseGraphSLAM, Covariances)
{
  PoseGraphSLAM pgs(4);
  // Very high confidence all around.
  Matrix6d cov0 = Matrix6d::Identity() * 1e-3;  
  // x translation can slide, everything else wants to be fixed.
  Matrix6d cov1 = Matrix6d::Identity() * 1e-3;
  cov1(0, 0) = 1;

  /*
   *
   *  ---- x
   *  |
   *  |
   *  y
   *
   *
   *   1 --- 0 --- 2
   *     \   |   /
   *       \ | /
   *         3
   *
   *  The horizontal links can slide around on the x axis.
   *  The "diagonal" links all want to be perfectly vertical.
   * 
   */
  
  pgs.addEdge(1, 0, Vector3d(1, 0, 0), Matrix3d::Identity(), cov1);
  pgs.addEdge(0, 2, Vector3d(1, 0, 0), Matrix3d::Identity(), cov1);
  pgs.addEdge(0, 3, Vector3d(0, 1, 0), Matrix3d::Identity(), cov0);
  pgs.addEdge(1, 3, Vector3d(0, 1, 0), Matrix3d::Identity(), cov0);
  pgs.addEdge(2, 3, Vector3d(0, 1, 0), Matrix3d::Identity(), cov0);

  pgs.solve();
  for(size_t i = 0; i < pgs.numNodes(); ++i) {
    cout << "--------------------" << endl;
    cout << "Location of node " << i << endl;
    cout << pgs.transform(i).translation() << endl;
  }

  // 1 and 2 should have moved to 0.
  // 3 should have stayed fixed.
  EXPECT_TRUE(pgs.transform(1).translation().norm() < 1e-2);
  EXPECT_TRUE(pgs.transform(2).translation().norm() < 1e-2);
  EXPECT_TRUE((pgs.transform(3).translation() - Vector3d(0, 1, 0)).norm() < 1e-3);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
