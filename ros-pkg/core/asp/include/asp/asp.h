#ifndef ASP_H
#define ASP_H

#include <pipeline/pipeline.h>
#include <graphcuts/potentials_cache.h>
#include <name_mapping/name_mapping.h>

namespace asp
{
  using namespace gc;
  using namespace pl;

  typedef boost::shared_ptr<Eigen::MatrixXf> MatrixXfPtr;
  typedef boost::shared_ptr<const Eigen::MatrixXf> MatrixXfConstPtr;
  
  //! Abstract Segmentation Pipeline.
  //! See test_asp.cpp for example usage.
  class Asp : public Pipeline
  {
  public:
    Asp(int num_threads);
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
    //!
    //! Also creates a debugging directory for output to go into.
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
      Pod(name)
    {
      // If provided, this is used for writing the overlay debug image.
      // Otherwise it will just write the raw debug image and skip the overlay.
      declareInput<cv::Mat3b>("BackgroundImage");  
      declareOutput<const Eigen::MatrixXf*>("Source");
      declareOutput<const Eigen::MatrixXf*>("Sink");
    }

  protected:
    Eigen::MatrixXf source_;
    Eigen::MatrixXf sink_;

    //! Uses BackgroundImage to set the size of source_ and sink_ if necessary
    //! and initialize their elements to zero.
    //! This is typically called at the start of compute().
    void initializeStorage();
    
    //! Displays node potentials in a common format.
    //! Typically, you call this in debug().
    //! Saves to disk as pod::debugBasePath() + "-raw.png" and
    //! (if BackgroundImage is provided) as pod::debugBasePath() + "-overlay.png".
    void writeNodePotentialVisualization() const;
  };
  
  class NodePotentialAggregator : public NodePotentialGenerator
  {
  public:
    DECLARE_POD(NodePotentialAggregator);
    NodePotentialAggregator(std::string name) :
      NodePotentialGenerator(name)
    {
      declareInput<const MatrixXf*>("UnweightedSource");
      declareInput<const MatrixXf*>("UnweightedSink");
    }

    NameMapping generateNameMapping() const;
    //! Copy of model is translated using local name mapping.
    void setWeights(Model model);
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

  protected:
    //! Saves image of final segmentation.
    void debug() const;
  };
  
}

#endif // ASP_H
