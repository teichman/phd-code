#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
#include <online_learning/synthetic_data_generator.h>
#include <online_learning/collage_generator.h>

using namespace std;

class TDCG : public TrackDatasetCollageGenerator
{
public:
  TDCG(TrackDataset::ConstPtr td, const std::string& name) :
    TrackDatasetCollageGenerator(td, name)
  {
  }

  cv::Mat3b visualize(__attribute__((unused)) const Instance& inst)
  {
    int cols = 200;
    int rows = 200;
    cv::Mat3b img(cv::Size(cols, rows), cv::Vec3b(255, 255, 255));
    int thickness = 1;
    float scale = 0.5;
    cv::putText(img, "Hello", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), thickness, CV_AA);
    return img;
  }
};

TEST(TrackDatasetCollageGenerator, SimpleGrid)
{
  int num_descriptors = 3;
  SyntheticDataGenerator sdg(getDefaultDimensionality(num_descriptors),
                             10, 1, 0.1, defaultClassMap(),
                             getStubDescriptorMap(num_descriptors));
  TrackDataset::Ptr td = sdg.sampleTrackDataset(70, 1000);

  TDCG tdcg(td, "Test set");
  cv::Mat3b grid5x5 = tdcg.generateGrid(5, 5);
  cv::imwrite("grid5x5.png", grid5x5);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
