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

  for(size_t i = 0; i < pgs.numNodes(); ++i)
    pgs.pose(i);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
