#include <gtest/gtest.h>
#include <asp/simple_segmentation_pipeline.h>

using namespace std;
using namespace Eigen;
using namespace asp::example;

void registerPods()
{
  typedef cv::Mat3b cvMat3b;
  typedef cv::Mat1b cvMat1b;
  REGISTER_POD_TEMPLATE(EntryPoint, cvMat3b);
  REGISTER_POD_TEMPLATE(EntryPoint, cvMat1b);
}

TEST(NodePotentialGenerator, NodePotentialGenerator)
{
  // registration is needed for serialization, which we're not using, *and* for
  // producing nice error messages at runtime.
  registerPods();
  Asp asp(1);
  generateSimpleSegmentationPipeline(&asp);
  asp.addPod(new EntryPoint<cv::Mat1b>("UnusedEntryPoint"));  // This shouldn't cause a crash.
  
  cv::Mat3b img;
  if(getenv("IMAGE_PATH"))
    img = cv::imread(getenv("IMAGE_PATH"));
  else {
    img = cv::Mat3b(cv::Size(100, 100), cv::Vec3b(127, 127, 127));
    for(int y = 0; y < img.rows; ++y)
      for(int x = 0; x < img.cols; ++x)
	img(y, x) = cv::Vec3b(rand() % 255, rand() % 255, rand() % 255);
  }
  asp.setInput("ImageEntryPoint", img);
  
  cv::Mat1b mask(img.size(), 255);
  for(int y = mask.rows / 2; y < mask.rows; ++y)
    for(int x = mask.cols / 2; x < mask.cols; ++x)
      mask(y, x) = 0;
  asp.setInput("MaskEntryPoint", mask);
  
  cv::Mat1b seed(img.size(), 127);
  for(int y = 0; y < 10; ++y)
    for(int x = 0; x < 20; ++x)
      seed(y, x) = 255;
  for(int y = 50; y < 80; ++y)
    for(int x = 10; x < 40; ++x)
      seed(y, x) = 0;
  asp.setInput("SeedEntryPoint", seed);
  asp.setDebug(true);
  asp.compute();

  asp.setDebug(false);
  asp.compute();
  cout << asp.reportTiming() << endl;

  asp.writeGraphviz("graphvis");
}

TEST(SparseMat, SparseMat)
{
  {
    ScopedTimer st("SparseMat fill");
    SparseMat mat(640, 480);
    mat.reserve(64 * 48 * 2);
    for(int y = 0; y < mat.rows(); y += 10)
      for(int x = 0; x < mat.cols(); x += 10)
	mat.insert(y, x) = 1;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
