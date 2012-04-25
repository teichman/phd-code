#ifndef __MULTI_SEQUENCE_MULTI_SEQUENCE_H__
#define __MULTI_SEQUENCE_MULTI_SEQUENCE_H__

#define BOOST_FILESYSTEM_VERSION 2
#include <vector>
#include <serializable/serializable.h>
#include <rgbd_sequence/stream_sequence.h>
#include <opencv2/core/core.hpp>

namespace multi_sequence
{
  class MultiSequence
  {
  public:
    typedef boost::shared_ptr<MultiSequence> Ptr;
    typedef boost::shared_ptr<const MultiSequence> ConstPtr;

    MultiSequence(double dt=0);
    MultiSequence(double dt, const std::vector<rgbd::StreamSequence::Ptr> &seqs);
    void addSequence(const rgbd::StreamSequence::Ptr &seq );
    size_t size() const;
    size_t num_sequences() const;
    //! Fetch the clouds at aligned frame i -- does not copy data
    void getClouds(int frame, std::vector<rgbd::Cloud::Ptr> &pcds) const;
    //! Fetch the images at aligned frame i -- does not copy data
    void getImages(int frame, std::vector<cv::Mat3b> &imgs) const;

    void save(const std::string &dir) const;
    void load(const std::string &dir);

  protected:
    std::vector<rgbd::StreamSequence::Ptr> seqs_;
    //! Populated by alignSequences()
    std::vector<std::vector<int> > alignments_;
    //! Flag for whether or not sequences have been aligned yet
    bool aligned_;
    //! Amount of time allowed between min and max frame in sequence set
    double dt_;
    //! Compute the alignment, store in alignments_
    void alignSequences();

  };

  template<class T>
  void minMax(const std::vector<T> &vec, T &min_val, size_t &min_idx, 
      T &max_val, size_t &max_idx )
  {
    min_val = max_val = vec[0];
    min_idx = max_idx = 0;
    for(size_t i = 1; i < vec.size(); i++)
    {
      if(vec[i] > max_val)
      {
        max_val = vec[i];
        max_idx = i;
      }
      if(vec[i] < min_val)
      {
        min_val = vec[i];
        min_idx = i;
      }
    }
  }

}

#endif
