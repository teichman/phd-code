#include <multi_sequence/multi_sequence.h>

namespace multi_sequence
{
  namespace bfs = boost::filesystem;

  MultiSequence::MultiSequence(double dt): 
    aligned_(false), dt_(dt) { }

  MultiSequence::MultiSequence(double dt, const std::vector<rgbd::StreamSequenceBase::Ptr> &seqs):
    aligned_(false),
    dt_(dt),
    seqs_(seqs)
  {
    alignSequences();
  }
  void MultiSequence::addSequence(const rgbd::StreamSequenceBase::Ptr &seq )
  {
    seqs_.push_back(seq);
    alignSequences();
  }
  size_t MultiSequence::size() const
  {
    return alignments_.size();
  }
  size_t MultiSequence::num_sequences() const
  {
    return seqs_.size();
  }
  void MultiSequence::getClouds(int frame, std::vector<rgbd::Cloud::Ptr> &pcds) const
  {
    ROS_ASSERT(aligned_ && frame < alignments_.size() );
    pcds.resize(num_sequences());
    for(size_t i=0; i < alignments_[frame].size(); i++)
    {
      pcds[i] = seqs_[i]->getCloud(alignments_[frame][i]);
    }
  }
  void MultiSequence::getImages(int frame, std::vector<cv::Mat3b> &imgs) const
  {
    ROS_ASSERT(aligned_ && frame < alignments_.size() );
    imgs.resize(seqs_.size());
    for(size_t i=0; i < alignments_[frame].size(); i++)
    {
      imgs[i] = seqs_[i]->getImage(alignments_[frame][i]);
    }
  }

  void MultiSequence::alignSequences()
  {
    alignments_.clear();
    std::vector<int> cur_frames(seqs_.size(),0);
    std::vector<double> timestamps(seqs_.size(),0);
    bool hit_end = false;
    while(true)
    {
      for(size_t i = 0; i < seqs_.size(); i++)
      {
        if(cur_frames[i] >= seqs_[i]->size()){
          hit_end = true;
          break;
        }
        timestamps[i] = seqs_[i]->timestamps_[cur_frames[i]];
      }
      if(hit_end)
        break;
      double min_stamp, max_stamp;
      size_t min_idx, max_idx;
      minMax(timestamps, min_stamp, min_idx, max_stamp, max_idx);
      // dt = max - min
      double dt = max_stamp - min_stamp;
      if(dt <= dt_)
      {
        alignments_.push_back(std::vector<int>(cur_frames));
        for(size_t i = 0; i < seqs_.size(); i++)
        {
          cur_frames[i]++;
        }
      }
      else
      {
        cur_frames[min_idx]++;
      }
    }
    aligned_ = true;
  }
}
