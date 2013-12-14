#include <online_learning/collage_generator.h>

using namespace std;
namespace bfs = boost::filesystem;

TrackDatasetCollageGenerator::TrackDatasetCollageGenerator(TrackDataset::ConstPtr td,
                                                           const std::string& name) :
  td_(td),
  name_(name)
{
}

cv::Mat3b TrackDatasetCollageGenerator::generateGrid(size_t rows, size_t cols,
                                                     const std::vector<size_t>& indices,
                                                     int instance_idx)
{
  ROS_ASSERT(indices.size() <= rows * cols);
  
  vector<cv::Mat3b> images;
  images.resize(indices.size());
  for(size_t i = 0; i < images.size(); ++i) {
    const Dataset& track = (*td_)[indices[i]];
    if(instance_idx == -1)
      images[i] = visualize(track[track.size() / 2]);
    else
      images[i] = visualize(track[instance_idx % track.size()]);

    if(i > 0) {
      ROS_ASSERT(images[i].rows == images[i-1].rows);
      ROS_ASSERT(images[i].cols == images[i-1].cols);
    }
  }

  // -- Fill in the grid.
  cv::Mat3b grid(cv::Size(cols * images[0].cols, rows * images[0].rows), cv::Vec3b(0, 0, 0));
  for(size_t i = 0; i < images.size(); ++i) {
    size_t ridx = i / rows;
    size_t cidx = i - ridx * cols;

    cv::Rect roi;
    roi.width = images[0].cols;
    roi.height = images[0].rows;
    roi.x = cidx * images[0].cols;
    roi.y = ridx * images[0].rows;
    cv::Mat window = grid(roi);
    images[i].copyTo(window);
  }

  // -- Add text.
  if(name_ != "") {
    int pad = 20;
    int thickness = 1;
    float scale = 0.5;
    cv::Scalar color(0, 0, 0);
    ostringstream oss;
    
    oss << name_ << ": ";
    oss << td_->size() << " tracks.";
    cv::putText(grid, oss.str(), cv::Point(pad, grid.rows - pad),
                cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, CV_AA);
  }
  
  return grid;
}

cv::Mat3b TrackDatasetCollageGenerator::generateGrid(size_t rows, size_t cols)
{
  vector<size_t> indices = generateRandomIndices(rows * cols);
  return generateGrid(rows, cols, indices, -1);
}

std::vector<size_t> TrackDatasetCollageGenerator::generateRandomIndices(size_t max) const
{
  vector<size_t> indices(td_->size());
  for(size_t i = 0; i < td_->size(); ++i)
    indices[i] = i;
  random_shuffle(indices.begin(), indices.end());
  size_t num = min(indices.size(), max);
  indices.erase(indices.begin() + num, indices.end());
  return indices;
}

void TrackDatasetCollageGenerator::writeVideo(size_t rows, size_t cols,
                                              const std::string& path,
                                              size_t num_frames,
                                              float fps,
                                              bool keep_images)
{
  string dir = path + ".d";
  ROS_ASSERT(!bfs::exists(dir));
  bfs::create_directory(dir);

  if(num_frames == 0)
    for(size_t i = 0; i < td_->size(); ++i)
      num_frames = max(num_frames, (*td_)[i].size());

  vector<size_t> indices = generateRandomIndices(rows * cols);
  for(size_t i = 0; i < num_frames; ++i) {
    cv::Mat3b img = generateGrid(rows, cols, indices, (int)i);
    ostringstream oss;
    oss << dir << "/image" << setw(5) << setfill('0') << i << ".png";
    cv::imwrite(oss.str(), img);
  }

  ostringstream oss;
  oss << "mencoder \"mf://" << dir << "/*.png\" -mf fps=" << fps << " -o " << path
      << " -ovc x264 -x264encopts crf=13 1> /dev/null 2>&1";
  int retval = system(oss.str().c_str());
  if(retval != 0) {
    ROS_WARN_STREAM("Video generation command failed.  retval: " << retval);
    ROS_WARN_STREAM("  Line: " << oss.str());
  }

  if(!keep_images)
    bfs::remove_all(dir);
}

