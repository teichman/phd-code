#ifndef TRACK_DATASET_COLLAGE_GENERATOR_H
#define TRACK_DATASET_COLLAGE_GENERATOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <online_learning/dataset.h>

class TrackDatasetCollageGenerator
{
public:
  //! Name will be printed on the image.
  TrackDatasetCollageGenerator(TrackDataset::ConstPtr td, const std::string& name = "");
  //! Fill in with however you generate a visualization of your raw data.
  //! These must all be the same size!
  virtual cv::Mat3b visualize(const Instance& inst) = 0;
  //! Randomly samples which tracks to display and uses an Instance from
  //! the middle of the track.
  //! Rows and cols are in number of images.
  cv::Mat3b generateGrid(size_t rows, size_t cols);
  //! indices maps into td_.
  //! instance_idx says which instance of each track to view.  -1 => use the middle.
  //! If instance_idx is too large it will wrap around.
  cv::Mat3b generateGrid(size_t rows, size_t cols,
                         const std::vector<size_t>& indices,
                         int instance_idx = -1);
  //! num_frames of zero means the video will have the length of the longest track.
  void writeVideo(size_t rows, size_t cols, 
                  const std::string& path,
                  size_t num_frames = 0,
                  bool keep_images = false);

protected:
  TrackDataset::ConstPtr td_;
  std::string name_;

  std::vector<size_t> generateRandomIndices(size_t max) const;
};


#endif // TRACK_DATASET_COLLAGE_GENERATOR_H
