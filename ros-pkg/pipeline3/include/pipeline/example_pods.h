#ifndef EXAMPLE_PODS_H
#define EXAMPLE_PODS_H

#include <pipeline/pod.h>

namespace pipeline
{
  namespace example
  {

    typedef std::vector<double> Vec;
    typedef boost::shared_ptr<Vec> VecPtr;
    typedef boost::shared_ptr<const Vec> VecConstPtr;

    //! Sorts a set of points.
    class Sorter : public Pod
    {
    public:
      DECLARE_POD(Sorter);
      Sorter(std::string name) :
	Pod(name),
	sorted_(new Vec)
      {
	declareInput<VecConstPtr>("Points");
	declareOutput<VecConstPtr>("Sorted");
      }

      VecPtr sorted_;
      void compute();
    };

    //! Computes descriptors on a set of points.
    //! Requires the input to be sorted.
    class Summarizer : public Pod
    {
    public:
      DECLARE_POD(Summarizer);
      Summarizer(std::string name) :
	Pod(name)
      {
	declareInput<VecConstPtr>("Points"); // Must be sorted.
	declareOutput<double>("Mean");
	declareOutput<double>("Stdev");
	declareOutput<double>("MeanNeighborSeparation");
      }

      void compute();
      //! This demonstrates one method of making Pod functionality available outside of a Pipeline.
      static void _compute(const Vec& points, double* mean, double* stdev, double* mean_neighbor_separation);
    };

    //! Computes a histogram on a set of points.
    //! Requires the input to be sorted.
    class HistogramGenerator : public Pod
    {
    public:
      DECLARE_POD(HistogramGenerator);
      HistogramGenerator(std::string name) :
	Pod(name),
	hist_(new Vec),
	lower_bounds_(new Vec)
      {
	declareParam<double>("BinWidth");
	declareParam<double>("Min");
	declareParam<double>("Max");
	declareParam<bool>("Normalize", false); // Whether or not the final histogram should sum to one. Default false.
	declareInput<VecConstPtr>("Points"); // Must be sorted.
	declareOutput<VecConstPtr>("Histogram");
	declareOutput<VecConstPtr>("LowerBounds"); // Vector of the lower bounds of each bin.
      }

      VecPtr hist_;
      VecPtr lower_bounds_;
      double num_points_;
      
      void compute();
      //! This demonstrates another method of making Pod functionality available outside of a Pipeline.
      void _compute(const Vec& points, VecPtr* hist, VecPtr* lower_bounds);
      void debug() const;
    };
    
    //! Merges multiple sets of points.
    //! Requires the inputs to be sorted.
    class Aggregator : public Pod
    {
    public:
      DECLARE_POD(Aggregator);
      Aggregator(std::string name) :
	Pod(name),
	aggregated_(new Vec)
      {
	declareInput<VecConstPtr>("PointSets");
	declareOutput<VecConstPtr>("Aggregated");
      }

      VecPtr aggregated_;
      void compute();
    };

    //! Assembles individual real numbers and subvectors
    //! into a single descriptor vector
    //! for consumption by a classifier or dataset manager.
    class DescriptorAssembler : public Pod
    {
    public:
      DECLARE_POD(DescriptorAssembler);
      DescriptorAssembler(std::string name) :
	Pod(name),
	descriptor_(new Vec)
      {
	declareInput<double>("Elements");
	declareInput<VecConstPtr>("SubVectors");
	declareOutput<VecConstPtr>("Descriptor");
      }

      VecPtr descriptor_;
      void compute();
    };

    
  } 
}

#endif // EXAMPLE_PODS_H
