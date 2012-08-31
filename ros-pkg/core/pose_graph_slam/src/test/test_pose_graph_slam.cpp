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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
