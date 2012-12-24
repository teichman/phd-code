#ifndef ASP_H
#define ASP_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pipeline/pipeline.h>
#include <graphcuts/potentials_cache.h>
#include <name_mapping/name_mapping.h>

namespace asp
{
  using namespace graphcuts;
  using namespace pipeline;

  typedef boost::shared_ptr<Eigen::MatrixXd> MatrixXdPtr;
  typedef boost::shared_ptr<const Eigen::MatrixXd> MatrixXdConstPtr;
  
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
    //! segmentation is 255 for fg, 0 for bg, 127 for unknown.
    void segment(cv::Mat1b* segmentation, PotentialsCache* cache = NULL);

  protected:
    //! Initializes pipeline_ with NodePotentialAggregator, EdgePotentialAggregator, 
    //! and GraphCuts pods.  You then hook up your own things to this.
    void initializePipeline();
  };
  
  class NodePotentialGenerator : public Pod
  {
  public:
    NodePotentialGenerator(std::string name) :
      Pod(name)
    {
      // If provided, this is used for writing the overlay debug image.
      // Otherwise it will just write the raw debug image and skip the overlay.
      declareInput<cv::Mat3b>("BackgroundImage");  
      declareOutput<const Eigen::MatrixXd*>("Source");
      declareOutput<const Eigen::MatrixXd*>("Sink");
    }

  protected:
    Eigen::MatrixXd source_;
    Eigen::MatrixXd sink_;

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
      declareInput<const Eigen::MatrixXd*>("UnweightedSource");
      declareInput<const Eigen::MatrixXd*>("UnweightedSink");
    }

    NameMapping generateNameMapping() const;
    //! Copy of model is translated using local name mapping.
    void setWeights(Model model);
    //! Sets nmap names based on input pod names.
    //! Sets model->nweights_ to local nweights_.
    void fillModel(Model* model) const;
    
  protected:
    Eigen::MatrixXd aggregated_;
    Eigen::VectorXd nweights_;

    void compute();
    void debug() const;
  };
  
  class EdgePotentialGenerator : public Pod
  {
  public:
    EdgePotentialGenerator(std::string name) :
      Pod(name),
      edge_(new DynamicSparseMat)
    {
      declareInput<cv::Mat3b>("BackgroundImage");
      declareOutput<DynamicSparseMatConstPtr>("Edge");
    }

    DynamicSparseMatPtr edge_;

    //! Allocates new sparse matrix if necessary; just sets to zero otherwise.
    //! Size is determined by BackgroundImage.
    //! For efficiency, you should set reserve_per_node to higher than the expected
    //! number of edges per node.
    void initializeStorage(double reserve_per_node = 2);

    //! Common function so there is no confusion about the use of row-major.
    int index(int row, int col) const { return col + row * edge_->cols(); }
    void writeEdgePotentialVisualization() const;
  };

  class EdgePotentialAggregator : public EdgePotentialGenerator
  {
  public:
    DECLARE_POD(EdgePotentialAggregator);
    EdgePotentialAggregator(std::string name) :
      EdgePotentialGenerator(name)
    {
      declareInput<DynamicSparseMatConstPtr>("UnweightedEdge");
    }

    NameMapping generateNameMapping() const;
    //! Copy of model is translated using local name mapping.
    void setWeights(Model model);
    //! Sets nmap names based on input pod names.
    //! Sets model->nweights_ to local nweights_.
    void fillModel(Model* model) const;
    
  protected:
    DynamicSparseMatPtr aggregated_;
    Eigen::VectorXd eweights_;

    void compute();
    void debug() const;
  };

  // class GraphcutsPod : public Pod
  // {
  // public:
  //   DECLARE_POD(GraphcutsPod);
  //   GraphcutsPod(std::string name) :
  //     Pod(name)
  //   {
  //     declareInput<MatrixXdConstPtr>("AggregatedNodePotentials");
  //     declareInput<DynamicSparseMatConstPtr>("AggregatedEdgePotentials");
  //   }

  // protected:
  //   //! Saves image of final segmentation.
  //   void debug() const;
  // };
  
}

#endif // ASP_H
