#include <gtest/gtest.h>
#include <asp/asp.h>

using namespace std;
using namespace Eigen;
using namespace asp;

//! Sets node potential randomly.
class ExampleNPG : public NodePotentialGenerator
{
public:
  DECLARE_POD(ExampleNPG);
  ExampleNPG(std::string name) :
    NodePotentialGenerator(name)
  {
  }

  void compute();
  void debug();
};

void ExampleNPG::compute()
{
  initializeStorage();

  for(int y = 0; y < source_.rows(); ++y) {
    for(int x = 0; x < source_.cols(); ++x) {
      source_(y, x) = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
      sink_(y, x) = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
    }
  }

  push<const MatrixXf*>("Source", &source_);
  push<const MatrixXf*>("Sink", &sink_);
}

void ExampleNPG::debug()
{
  writeNodePotentialVisualization();
}

TEST(NodePotentialGenerator, NodePotentialGenerator)
{
  Asp asp;
  asp.addPod(new ExampleNPG("ExampleNPG0"));
  asp.pod("ExampleNPG0")->registerInput("BackgroundImage",
					asp.getPod("ImageEntryPoint"),
					"Output");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
