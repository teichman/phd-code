#include <gtest/gtest.h>
#include <xpl_calibration/object_matching_calibrator.h>

using namespace std;

TEST(generateTransform, generateTransform)
{
  Eigen::Affine3f trans = generateTransform(1, 2, 3, 0.5, 0.35, 0.56);
  cout << trans.matrix() << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

