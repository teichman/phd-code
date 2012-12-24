#include <gtest/gtest.h>
#include <asp/asp.h>

using namespace std;
using namespace Eigen;
using namespace asp;

typedef cv::Mat3b cvMat3b;

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
  void debug() const { writeNodePotentialVisualization(); }
};

void ExampleNPG::compute()
{
  cout << "ExampleNPG::compute()" << endl;
  initializeStorage();

  for(int y = 0; y < source_.rows(); ++y) {
    for(int x = 0; x < source_.cols(); ++x) {
      source_(y, x) = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
      sink_(y, x) = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
    }
  }

  push<const MatrixXd*>("Source", &source_);
  push<const MatrixXd*>("Sink", &sink_);
}

class ExampleEPG : public EdgePotentialGenerator
{
public:
  DECLARE_POD(ExampleEPG);
  ExampleEPG(std::string name) :
    EdgePotentialGenerator(name)
  {
  }

  void compute();
  void debug() const { writeEdgePotentialVisualization(); }
};

void ExampleEPG::compute()
{
  initializeStorage();

  for(int y = 0; y < edge_->rows(); ++y) {
    if(rand() % 20 == 0) {
      int x = rand() % edge_->cols();
      edge_->coeffRef(y, x) = (double)rand() / RAND_MAX;
    }
  }

  push<DynamicSparseMatConstPtr>("Edge", edge_);
}
  
void registerPods()
{
  REGISTER_POD_TEMPLATE(EntryPoint, cvMat3b);
}

TEST(NodePotentialGenerator, NodePotentialGenerator)
{
  registerPods();
  Asp asp(1);
  asp.addPod(new ExampleNPG("ExampleNPG0"));
  asp.connect("ImageEntryPoint:Output -> ExampleNPG0:BackgroundImage");
  asp.connect("ExampleNPG0:Source -> NodePotentialAggregator:UnweightedSource");
  asp.connect("ExampleNPG0:Sink -> NodePotentialAggregator:UnweightedSink");
  asp.addPod(new ExampleNPG("ExampleNPG1"));
  asp.connect("ImageEntryPoint:Output -> ExampleNPG1:BackgroundImage");
  asp.connect("ExampleNPG1:Source -> NodePotentialAggregator:UnweightedSource");
  asp.connect("ExampleNPG1:Sink -> NodePotentialAggregator:UnweightedSink");
  asp.addPod(new ExampleEPG("ExampleEPG0"));
  asp.connect("ImageEntryPoint:Output -> ExampleEPG0:BackgroundImage");
  asp.connect("ExampleEPG0:Edge -> EdgePotentialAggregator:UnweightedEdge");
  asp.addPod(new ExampleEPG("ExampleEPG1"));
  asp.connect("ImageEntryPoint:Output -> ExampleEPG1:BackgroundImage");
  asp.connect("ExampleEPG1:Edge -> EdgePotentialAggregator:UnweightedEdge");
  

  Model model = asp.defaultModel();
  model.nweights_.setConstant(1);
  model.eweights_.setConstant(1);
  asp.setModel(model);
  
  cv::Mat3b img(cv::Size(100, 100), cv::Vec3b(127, 127, 127));
  ((EntryPoint<cv::Mat3b>*)asp.getPod("ImageEntryPoint"))->setData(img);
  asp.setDebug(true);
  asp.compute();
  // cv::Mat1b seg;
  // asp.segment(&seg);
  // cv::imshow("seg", seg);
  // cv::waitKey();

  asp.setDebug(false);
  asp.compute();
  cout << asp.reportTiming() << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
