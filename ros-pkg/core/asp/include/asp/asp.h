#ifndef ASP_H
#define ASP_H

#include <pipeline/pipeline.h>
#include <graphcuts/potentials_cache.h>
#include <name_mapping/name_mapping.h>

namespace asp {
  using namespace gc;
  using namespace pl;

  typedef boost::shared_ptr<Eigen::MatrixXf> MatrixXfPtr;
  typedef boost::shared_ptr<const Eigen::MatrixXf> MatrixXfConstPtr;
  
  //! Abstract Segmentation Pipeline.
  class Asp
  {
  public:
    Pipeline pipeline_;

    Asp();
    virtual ~Asp() {};

    //! Names are automatically filled in based on Pipeline pod names.
    //! Weights need not be set in NPA and EPA.
    Model defaultModel() const;
    //! Names are automatically filled in based on Pipeline pod names.
    //! Weights must be set in NPA and EPA.
    Model model() const;
    //! Installs weights into NPA and EPA.
    //! Names in nmap and emap must match node potential and edge potential pod names.
    void setModel(const Model& model);

    //! You have to set the input to the pipeline yourself first.
    //! Also, setModel() must have been called.
    //! segmentation will be resized if it is not the correct length.
    //! cache will be filled if it is not null.
    void segment(Eigen::VectorXi* segmentation, PotentialsCache* cache = NULL);

  protected:
    //! Initializes pipeline_ with NodePotentialAggregator, EdgePotentialAggregator, 
    //! and GraphCuts pods.  You then hook up your own things to this.
    void initializePipeline();
  };

  class NodePotentialGenerator : public pl::Pod
  {
  public:
    NodePotentialGenerator(std::string name) :
      Pod(name),
      source_(new MatrixXf),
      sink_(new MatrixXf)
    {
      declareInput<cv::Mat3b>("Image");
      declareOutput<MatrixXfConstPtr>("SourcePotentials");
      declareOutput<MatrixXfConstPtr>("SinkPotentials");
    }

  protected:
    MatrixXfConstPtr source_;
    MatrixXfConstPtr sink_;

    //void displayNodePotentials(const cv::Mat3b background = cv::Mat3b()) const;
    void displayNodePotentials() const;
  };
  
  class NodePotentialAggregator : public NodePotentialGenerator
  {
  public:
    DECLARE_POD(NodePotentialAggregator);
    NodePotentialAggregator(std::string name) :
      NodePotentialGenerator(name)
    {
      declareInput<MatrixXfConstPtr>("UnweightedSourcePotentials");
      declareInput<MatrixXfConstPtr>("UnweightedSinkPotentials");
    }

    //! Checks that names match.
    void setWeights(const Model& model);
    //! Sets nmap names based on input pod names.
    //! Sets model->nweights_ to local nweights_.
    void fillModel(Model* model) const;
    
  protected:
    MatrixXfPtr aggregated_;
    Eigen::VectorXf nweights_;

    void compute();
    void debug() const;
  };
  
  class EdgePotentialGenerator : public pl::Pod
  {
  public:
    EdgePotentialGenerator(std::string name) :
      Pod(name),
      edge_(new DynamicSparseMat)
    {
      declareInput<cv::Mat3b>("Image");
      declareOutput<DynamicSparseMatConstPtr>("EdgePotentials");
    }

    DynamicSparseMatPtr edge_;

    //! Allocates new sparse matrix if necessary; just sets to zero otherwise.
    void initializeStorage(int num_nodes, double reserve_per_node = 2);
    //void displayEdges(cv::Mat3b img) const;
    void displayEdgePotentials() const;
  };

  class EdgePotentialAggregator : public EdgePotentialGenerator
  {
  public:

  protected:
    void debug() const;
  };

  class GraphcutsPod : public pl::Pod
  {
  public:
    DECLARE_POD(GraphcutsPod);
    GraphcutsPod(std::string name) :
      Pod(name)
    {
      declareInput<MatrixXfConstPtr>("AggregatedNodePotentials");
      declareInput<DynamicSparseMatConstPtr>("AggregatedEdgePotentials");
      
    }
  };
  
}

#endif // ASP_H
