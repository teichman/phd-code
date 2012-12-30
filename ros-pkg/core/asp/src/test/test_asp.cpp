#include <gtest/gtest.h>
#include <asp/asp.h>

using namespace std;
using namespace Eigen;
using namespace asp;

typedef cv::Mat3b cvMat3b;
typedef cv::Mat1b cvMat1b;

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
  const SparseMat& structure = *pull<const SparseMat*>("EdgeStructure");

  for(int i = 0; i < structure.rows(); ++i) {
    SparseMat::InnerIterator it(structure, i);
    for(; it; ++it)
      edge_.insert(it.row(), it.col()) = (double)rand() / RAND_MAX;
  }

  push<const SparseMat*>("Edge", &edge_);
}
  
void registerPods()
{
  REGISTER_POD_TEMPLATE(EntryPoint, cvMat3b);
  REGISTER_POD_TEMPLATE(EntryPoint, cvMat1b);
}

TEST(NodePotentialGenerator, NodePotentialGenerator)
{
  registerPods();
  Asp asp(1);

  asp.addPod(new ExampleNPG("ExampleNPG0"));
  asp.connect("ImageEntryPoint:Output -> ExampleNPG0:Image");
  asp.connect("ExampleNPG0:Source -> NodePotentialAggregator:UnweightedSource");
  asp.connect("ExampleNPG0:Sink -> NodePotentialAggregator:UnweightedSink");

  asp.addPod(new EdgeStructureGenerator("GridESG"));
  asp.setParam("GridESG", "Grid", true);
  asp.connect("ImageEntryPoint:Output -> GridESG:Image");
  asp.addPod(new EdgeStructureGenerator("DiagonalESG"));
  asp.setParam("DiagonalESG", "Diagonal", true);
  asp.connect("ImageEntryPoint:Output -> DiagonalESG:Image");
  asp.addPod(new EdgeStructureGenerator("WebESG"));
  asp.setParam("WebESG", "Web", true);
  asp.connect("ImageEntryPoint:Output -> WebESG:Image");
  
  asp.addPod(new SmoothnessEPG("SmoothnessEPG0"));
  asp.connect("ImageEntryPoint:Output -> SmoothnessEPG0:Image");
  asp.connect("SmoothnessEPG0:Edge -> EdgePotentialAggregator:UnweightedEdge");
  asp.connect("GridESG:EdgeStructure -> SmoothnessEPG0:EdgeStructure");
  asp.addPod(new SmoothnessEPG("SmoothnessEPG1"));
  asp.connect("ImageEntryPoint:Output -> SmoothnessEPG1:Image");
  asp.connect("SmoothnessEPG1:Edge -> EdgePotentialAggregator:UnweightedEdge");
  asp.connect("DiagonalESG:EdgeStructure -> SmoothnessEPG1:EdgeStructure");

  asp.addPod(new SimpleColorDifferenceEPG("GridSimpleColorDifferenceEPG"));
  asp.connect("ImageEntryPoint:Output -> GridSimpleColorDifferenceEPG:Image");
  asp.connect("GridESG:EdgeStructure -> GridSimpleColorDifferenceEPG:EdgeStructure");
  asp.connect("GridSimpleColorDifferenceEPG:Edge -> EdgePotentialAggregator:UnweightedEdge");
  asp.addPod(new SimpleColorDifferenceEPG("DiagonalSimpleColorDifferenceEPG"));
  asp.connect("ImageEntryPoint:Output -> DiagonalSimpleColorDifferenceEPG:Image");
  asp.connect("DiagonalESG:EdgeStructure -> DiagonalSimpleColorDifferenceEPG:EdgeStructure");
  asp.connect("DiagonalSimpleColorDifferenceEPG:Edge -> EdgePotentialAggregator:UnweightedEdge");
  asp.addPod(new SimpleColorDifferenceEPG("WebSimpleColorDifferenceEPG"));
  asp.connect("ImageEntryPoint:Output -> WebSimpleColorDifferenceEPG:Image");
  asp.connect("WebESG:EdgeStructure -> WebSimpleColorDifferenceEPG:EdgeStructure");
  asp.connect("WebSimpleColorDifferenceEPG:Edge -> EdgePotentialAggregator:UnweightedEdge");
  
  Model model = asp.defaultModel();
  model.nweights_.setConstant(1);
  model.nweights_(model.nameMapping("nmap").toId("SeedNPG")) = 2;
  model.nweights_(model.nameMapping("nmap").toId("ExampleNPG0")) = 0;
  model.eweights_.setConstant(1);
  model.eweights_(model.nameMapping("emap").toId("SmoothnessEPG0")) = 0.1;
  model.eweights_(model.nameMapping("emap").toId("SmoothnessEPG1")) = 0.1;
  model.eweights_(model.nameMapping("emap").toId("WebSimpleColorDifferenceEPG")) = 0.3;
  asp.setModel(model);

  cv::Mat3b img;
  if(getenv("IMAGE_PATH"))
    img = cv::imread(getenv("IMAGE_PATH"));
  else {
    img = cv::Mat3b(cv::Size(100, 100), cv::Vec3b(127, 127, 127));
    for(int y = 0; y < img.rows; ++y)
      for(int x = 0; x < img.cols; ++x)
	img(y, x) = cv::Vec3b(rand() % 255, rand() % 255, rand() % 255);
  }
  
  asp.pod< EntryPoint<cv::Mat3b> >("ImageEntryPoint")->setData(img);
  cv::Mat1b seed(img.size(), 127);
  for(int y = 0; y < 10; ++y)
    for(int x = 0; x < 20; ++x)
      seed(y, x) = 255;
  for(int y = 50; y < 80; ++y)
    for(int x = 50; x < 80; ++x)
      seed(y, x) = 0;
  asp.pod< EntryPoint<cv::Mat1b> >("SeedEntryPoint")->setData(seed);
  asp.setDebug(true);
  asp.compute();
  // cv::Mat1b seg;
  // asp.segment(&seg);
  // cv::imshow("seg", seg);
  // cv::waitKey();

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
