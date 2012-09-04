#include <gtest/gtest.h>
#include <pose_graph_slam/pose_graph_slam.h>

using namespace std;
using namespace g2o;


TEST(PoseGraphSlam, Simple)
{
  PoseGraphSlam pgs(4);
  Matrix6d covariance = Matrix6d::Identity();

  pgs.addEdge(0, 1,
	      (Affine3d)Translation3d(Vector3d(0, 1, 0)),
	      covariance);
  pgs.addEdge(1, 2,
	      Translation3d(Vector3d(0, 1, 0)) * Affine3d(AngleAxis<double>(M_PI / 4, Vector3d(1, 0, 0))),
	      covariance);
  pgs.addEdge(2, 3,
	      Translation3d(Vector3d(0, 1, 0)) * Affine3d(AngleAxis<double>(M_PI / 4, Vector3d(1, 0, 0))),
	      covariance);

  pgs.solve();

  Vector3d translation;
  Quaterniond quat;
  for(size_t i = 0; i < pgs.numNodes(); ++i) {
    cout << "--------------------" << endl;
    pgs.vertexData(i, &translation, &quat);
    cout << "Translation: " << translation.transpose() << endl;
    cout << "Quaternion: " << quat.vec().transpose() << endl;
    cout << "Quaternion (w, x, y, z): " << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() << endl;
    cout << "Quaternion in matrix form:" << endl << quat.toRotationMatrix() << endl;
    cout << "Transform " << i << endl;
    cout << pgs.transform(i).matrix() << endl;
  }

  EXPECT_TRUE((pgs.transform(0).matrix() - Matrix4d::Identity()).norm() < 1e-6);
  
  Vector4d transformed;
  Vector4d expected;
  transformed = pgs.transform(1) * Vector4d(0, 1, 0, 1);
  expected = Vector4d(0, 2, 0, 1);
  cout << "1: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  transformed = pgs.transform(2) * Vector4d(0, 1, 0, 1);
  expected = Vector4d(0, 1 + .70711 * 2, .70711 * 2, 1);
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);
  cout << "2: " << transformed.transpose() << ", expected " << expected.transpose() << endl;

  transformed = pgs.transform(3) * Vector4d(0, 1, 0, 1);
  expected = Vector4d(0, 1.70711, 2.70711, 1);
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);
  cout << "3: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
}

TEST(PoseGraphSlam, Covariances)
{
  PoseGraphSlam pgs(4);
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
  
  pgs.addEdge(1, 0, (Affine3d)Translation3d(Vector3d(1, 0, 0)), cov1);
  pgs.addEdge(0, 2, (Affine3d)Translation3d(Vector3d(1, 0, 0)), cov1);
  pgs.addEdge(0, 3, (Affine3d)Translation3d(Vector3d(0, 1, 0)), cov0);
  pgs.addEdge(1, 3, (Affine3d)Translation3d(Vector3d(0, 1, 0)), cov0);
  pgs.addEdge(2, 3, (Affine3d)Translation3d(Vector3d(0, 1, 0)), cov0);

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
