#ifndef SIMPLE_SEGMENTATION_PIPELINE_H
#define SIMPLE_SEGMENTATION_PIPELINE_H

#include <asp/asp.h>

namespace asp
{
  namespace example
  {
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

      for(int y = 0; y < node_.rows(); ++y)
	for(int x = 0; x < node_.cols(); ++x)
	  node_(y, x) = ((double)rand() / RAND_MAX) * 2.0 - 1.0;

      push<const MatrixXd*>("Node", &node_);
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

    
    void generateSimpleSegmentationPipeline(Asp* asp)
    {
      asp->addPod(new ExampleNPG("ExampleNPG0"));
      asp->connect("ImageEntryPoint:Output -> ExampleNPG0:Image");
      asp->connect("ExampleNPG0:Node -> NodePotentialAggregator:UnweightedNode");

      asp->addPod(new EdgeStructureGenerator("GridESG"));
      asp->setParam("GridESG", "Grid", true);
      asp->connect("ImageEntryPoint:Output -> GridESG:Image");
      asp->connect("MaskEntryPoint:Output -> GridESG:Mask");
      asp->addPod(new EdgeStructureGenerator("DiagonalESG"));
      asp->setParam("DiagonalESG", "Diagonal", true);
      asp->connect("ImageEntryPoint:Output -> DiagonalESG:Image");
      asp->connect("MaskEntryPoint:Output -> DiagonalESG:Mask");
      asp->addPod(new EdgeStructureGenerator("WebESG"));
      asp->setParam("WebESG", "Web", true);
      asp->setParam("WebESG", "WebNumOutgoing", 5);
      asp->connect("ImageEntryPoint:Output -> WebESG:Image");
      asp->connect("MaskEntryPoint:Output -> WebESG:Mask");
  
      asp->addPod(new SmoothnessEPG("SmoothnessEPG0"));
      asp->connect("ImageEntryPoint:Output -> SmoothnessEPG0:Image");
      asp->connect("SmoothnessEPG0:Edge -> EdgePotentialAggregator:UnweightedEdge");
      asp->connect("GridESG:EdgeStructure -> SmoothnessEPG0:EdgeStructure");
      asp->addPod(new SmoothnessEPG("SmoothnessEPG1"));
      asp->connect("ImageEntryPoint:Output -> SmoothnessEPG1:Image");
      asp->connect("SmoothnessEPG1:Edge -> EdgePotentialAggregator:UnweightedEdge");
      asp->connect("DiagonalESG:EdgeStructure -> SmoothnessEPG1:EdgeStructure");

      asp->addPod(new SimpleColorDifferenceEPG("GridSimpleColorDifferenceEPG"));
      asp->connect("ImageEntryPoint:Output -> GridSimpleColorDifferenceEPG:Image");
      asp->connect("GridESG:EdgeStructure -> GridSimpleColorDifferenceEPG:EdgeStructure");
      asp->connect("GridSimpleColorDifferenceEPG:Edge -> EdgePotentialAggregator:UnweightedEdge");
      asp->addPod(new SimpleColorDifferenceEPG("DiagonalSimpleColorDifferenceEPG"));
      asp->connect("ImageEntryPoint:Output -> DiagonalSimpleColorDifferenceEPG:Image");
      asp->connect("DiagonalESG:EdgeStructure -> DiagonalSimpleColorDifferenceEPG:EdgeStructure");
      asp->connect("DiagonalSimpleColorDifferenceEPG:Edge -> EdgePotentialAggregator:UnweightedEdge");
      asp->addPod(new SimpleColorDifferenceEPG("WebSimpleColorDifferenceEPG"));
      asp->connect("ImageEntryPoint:Output -> WebSimpleColorDifferenceEPG:Image");
      asp->connect("WebESG:EdgeStructure -> WebSimpleColorDifferenceEPG:EdgeStructure");
      asp->connect("WebSimpleColorDifferenceEPG:Edge -> EdgePotentialAggregator:UnweightedEdge");

      Model model = asp->defaultModel();
      model.nweights_.setConstant(1);
      model.nweights_(model.nameMapping("nmap").toId("SeedNPG")) = 2;
      model.nweights_(model.nameMapping("nmap").toId("ExampleNPG0")) = 0;
      model.eweights_.setConstant(1);
      model.eweights_(model.nameMapping("emap").toId("SmoothnessEPG0")) = 0.1;
      model.eweights_(model.nameMapping("emap").toId("SmoothnessEPG1")) = 0.1;
      model.eweights_(model.nameMapping("emap").toId("WebSimpleColorDifferenceEPG")) = 0.3;
      asp->setModel(model);
    }
  }
}
  

#endif // SIMPLE_SEGMENTATION_PIPELINE_H
