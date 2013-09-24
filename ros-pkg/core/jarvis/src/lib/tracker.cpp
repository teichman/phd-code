#include <jarvis/tracker.h>
#include <bag_of_tricks/connected_components.h>
#include <sentinel/background_model.h>
#include <pcl/common/centroid.h>

using namespace std;
namespace bpt = boost::posix_time;
using namespace Eigen;

void Blob::project()
{
  cloud_ = Cloud::Ptr(new Cloud);
  cloud_->reserve(indices_.size());
  cloud_->is_dense = true;
  float f = 525 * ((float)width_ / 640);
  float cu = width_ / 2;
  float cv = height_ / 2;
  
  for(size_t i = 0; i < indices_.size(); ++i) {
    ROS_ASSERT(depth_[i] > 0);
    int idx = indices_[i];
    int v = idx / width_;
    int u = idx - v * width_;
    Point pt;
    pt.z = depth_[i];
    pt.x = pt.z * (u - cu) / f;
    pt.y = pt.z * (v - cv) / f;
    pt.r = color_[i*3+0];
    pt.g = color_[i*3+1];
    pt.b = color_[i*3+2];
    cloud_->push_back(pt);
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_, centroid);
  centroid_.setZero();
  centroid_ = centroid.head(3);

  kdtree_ = KdTree::Ptr(new KdTree(false));  // Don't sort the points.
  kdtree_->setInputCloud(cloud_);
}

Tracker::Tracker(size_t max_track_length) :
  visualize_(false),
  max_track_length_(max_track_length),
  next_track_id_(0)
{
}

void Tracker::update(sentinel::ForegroundConstPtr msg)
{
  HighResTimer hrt;
  frame_id_ = msg->frame_id;
  ptime_ = msg->header.stamp.toBoost();

  // -- Debugging.
  // cout << "Got a detection with " << msg->indices.size() << " points." << endl;
  // cout << msg->depth.size() << " " << msg->color.size() << endl;

  // -- Allocate memory if necessary.
  if(depth_.rows != msg->height) {
    color_ = cv::Mat3b(cv::Size(msg->width, msg->height), cv::Vec3b(0, 0, 0));
    depth_ = cv::Mat1f(cv::Size(msg->width, msg->height), 0);
    foreground_ = cv::Mat1b(cv::Size(msg->width, msg->height), 0);
    assignments_ = cv::Mat1i(cv::Size(msg->width, msg->height), 0);
  }

  // -- Check the data.
  ROS_ASSERT(msg->indices.size() == msg->depth.size());
  ROS_ASSERT(msg->color.size() == msg->depth.size() * 3);
  ROS_ASSERT((int)msg->height == depth_.rows);
  ROS_ASSERT(msg->width % msg->width_step == 0);
  ROS_ASSERT(msg->height % msg->height_step == 0);
  
  // -- Build the depth image.
  depth_ = 0;
  for(size_t i = 0; i < msg->indices.size(); ++i) {
    uint32_t idx = msg->indices[i];
    depth_(idx) = msg->depth[i] * 0.001;
  }

  // -- Build the color image.
  color_ = cv::Vec3b(0, 0, 0);
  for(size_t i = 0; i < msg->indices.size(); ++i) {
    uint32_t idx = msg->indices[i];
    color_(idx)[2] = msg->color[i*3+0];
    color_(idx)[1] = msg->color[i*3+1];
    color_(idx)[0] = msg->color[i*3+2];
  }

  // -- Run the bilateral filter to get the detailed foreground mask.
  reconstructForeground(msg, depth_, foreground_);

  // -- Floodfill clustering.
  //    Ignore parts of the depth image that were not part of the foreground.
  for(int y = 0; y < depth_.rows; ++y)
    for(int x = 0; x < depth_.cols; ++x)
      if(foreground_(y, x) == 0)
        depth_(y, x) = 0;

  // Run the clustering.
  hrt.reset("clustering"); hrt.start();
  // The min points argument here shouldn't matter, because size filtering
  // is done in Sentinel now.  It's currently a workaround for a bug in cluster().
  cluster(depth_, 0.2, 50, &assignments_, &indices_);  
  //hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  if(visualize_) { 
    cv::Mat3b clustering_scaled;
    cv::resize(colorAssignments(assignments_), clustering_scaled, assignments_.size() * 2, cv::INTER_NEAREST);
    cv::imshow("Clustering", clustering_scaled);
  }

  // -- The foreground should only contain points in large-enough clusters.
  for(int y = 0; y < assignments_.rows; ++y)
    for(int x = 0; x < assignments_.cols; ++x)
      if(assignments_(y, x) < 0)
        foreground_(y, x) = 0;

  // -- Construct blobs.
  vector<Blob::Ptr> current_blobs;
  for(size_t i = 0; i < indices_.size(); ++i) {
    Blob::Ptr blob(new Blob);
    blob->frame_id_ = msg->frame_id;
    blob->sensor_timestamp_ = msg->sensor_timestamp;
    blob->wall_timestamp_ = msg->header.stamp;
    blob->height_ = msg->height;
    blob->width_ = msg->width;
    
    blob->indices_.resize(indices_[i].size());
    blob->color_.resize(indices_[i].size() * 3);
    blob->depth_.resize(indices_[i].size());
    for(size_t j = 0; j < indices_[i].size(); ++j) {
      size_t idx = indices_[i][j];
      ROS_ASSERT((int)idx < color_.rows * color_.cols);
      ROS_ASSERT((int)idx < depth_.rows * depth_.cols);
      blob->indices_[j] = idx;
      blob->color_[j*3+0] = color_(idx)[2];
      blob->color_[j*3+1] = color_(idx)[1];
      blob->color_[j*3+2] = color_(idx)[0];
      blob->depth_[j] = depth_(idx);
      ROS_ASSERT(depth_(idx) > 0);
      ROS_ASSERT(blob->depth_[j] > 0);
    }

    blob->project();
    current_blobs.push_back(blob);
    ROS_ASSERT(blob->indices_.size() > 10);
  }

  // -- Simple correspondence method.
  vector<bool> matched_blobs(current_blobs.size(), false);
  vector<bool> matched_tracks(tracks_.size(), false);
  vector<size_t> track_indices(tracks_.size());
  if(!tracks_.empty() && !current_blobs.empty()) {
    // Compute pairwise distances.
    MatrixXd distances = MatrixXd::Zero(tracks_.size(), current_blobs.size());
    map<size_t, Blob::Ptr>::iterator it;
    size_t r = 0;
    for(it = tracks_.begin(); it != tracks_.end(); ++it, ++r) {
      track_indices[r] = it->first;
      for(size_t c = 0; c < current_blobs.size(); ++c)
        distances(r, c) = (distance(*it->second, *current_blobs[c]) + distance(*current_blobs[c], *it->second)) / 2.0;
    }

    // cout << endl;
    // cout << "Distances: " << endl;
    // cout << distances << endl;
    
    // Greedy assignment.
    while(distances.minCoeff() < numeric_limits<double>::max()) {
      int r, c;
      double dist = distances.minCoeff(&r, &c);
      if(dist < 2) {
        ROS_ASSERT(tracks_.find(track_indices[r]) != tracks_.end());
        tracks_[track_indices[r]] = current_blobs[c];
        matched_blobs[c] = true;
        matched_tracks[r] = true;
      }
      distances.row(r).setConstant(numeric_limits<double>::max());
      distances.col(c).setConstant(numeric_limits<double>::max());
    }
  }

  // -- Delete any unmatched tracks.
  for(size_t i = 0; i < matched_tracks.size(); ++i)
    if(!matched_tracks[i])
      tracks_.erase(track_indices[i]);
  
  // -- Delete old and unmatched tracks.
  // vector<size_t> to_erase;
  // to_erase.reserve(tracks_.size());
  // map<size_t, Blob::Ptr>::iterator it;
  // for(it = tracks_.begin(); it != tracks_.end(); ++it)
  //   if(msg->header.stamp.toSec() - it->second->wall_timestamp_.toSec() > 1.0)
  //     to_erase.push_back(it->first);
  // for(size_t i = 0; i < to_erase.size(); ++i) {
  //   ROS_ASSERT(tracks_.find(to_erase[i]) != tracks_.end());
  //   tracks_.erase(to_erase[i]);
  // }

  // -- Generate tracks for each unmatched blob.
  for(size_t i = 0; i < current_blobs.size(); ++i) {
    if(!matched_blobs[i]) {
      ROS_ASSERT(tracks_.find(next_track_id_) == tracks_.end());
      tracks_[next_track_id_] = current_blobs[i];
      ++next_track_id_;
      //cout << "Creating new track." << endl;
    }
  }
}

void Tracker::reconstructForeground(sentinel::Foreground::ConstPtr msg,
                                    cv::Mat1f depth, cv::Mat1b foreground) const
{
  //ScopedTimer st("Tracker::reconstructForeground", TimeUnit::MS);
  
  cv::Mat1b indices_mask(cv::Size(msg->width, msg->height), 0);
  for(size_t i = 0; i < msg->indices.size(); ++i) {
    uint32_t idx = msg->indices[i];
    int y = idx / depth_.cols;
    int x = idx - y * depth_.cols;
    indices_mask(y, x) = 255;
  }
    
  cv::Mat1f values(cv::Size(msg->width, msg->height), 0);
  double sigma_p = 10;
  double sigma_d = 0.2;

  for(size_t i = 0; i < msg->fg_indices.size(); ++i) {
    uint32_t idx = msg->fg_indices[i];
    int y = idx / msg->width;
    int x = idx - y * msg->width;
    int dx = msg->width_step / 2 + 1;
    int dy = msg->height_step / 2 + 1;
    for(int y2 = y - dy; y2 <= y + dy; ++y2) {
      for(int x2 = x - dx; x2 <= x + dx; ++x2) {
        if(y2 < 0 || y2 >= msg->height ||
           x2 < 0 || x2 >= msg->width)
        {
          continue;
        }
        double img_coef = exp(-sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2)) / sigma_p);
        double depth_coef = exp(-fabs(depth(y, x) - depth(y2, x2)) / sigma_d);
        values(y2, x2) += img_coef * depth_coef;
      }
    }
  }
  for(size_t i = 0; i < msg->bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg->bg_fringe_indices[i];
    int y = idx / msg->width;
    int x = idx - y * msg->width;
    int dx = msg->width_step / 2 + 1;
    int dy = msg->height_step / 2 + 1;
    for(int y2 = y - dy; y2 <= y + dy; ++y2) {
      for(int x2 = x - dx; x2 <= x + dx; ++x2) {
        if(y2 < 0 || y2 >= msg->height ||
           x2 < 0 || x2 >= msg->width)
        {
          continue;
        }
        double img_coef = exp(-sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2)) / sigma_p);
        double depth_coef = exp(-fabs(depth(y, x) - depth(y2, x2)) / sigma_d);
        values(y2, x2) -= img_coef * depth_coef;
      }
    }
  }

  // -- Fill the output values.
  foreground = 0;
  for(int y = 0; y < values.rows; ++y)
    for(int x = 0; x < values.cols; ++x)
      if(values(y, x) > 0 && indices_mask(y, x) == 255 && depth(y, x) > 1e-3)
        foreground(y, x) = 255;
}

double Tracker::distance(const Blob& prev, const Blob& curr) const
{
  ROS_ASSERT(prev.cloud_ && curr.cloud_);
  ROS_ASSERT(prev.kdtree_ && curr.kdtree_);

  // -- If the timestamps are too far apart, this isn't a match.
  if(curr.wall_timestamp_.toSec() - prev.wall_timestamp_.toSec() > 1.0)
    return numeric_limits<double>::max();
  
  // -- If the centroids are quite far apart, don't bother doing anything else.
  if((prev.centroid_ - curr.centroid_).norm() > 2)
    return numeric_limits<double>::max();

  // -- Choose some random points in one object and compute distance to the other.
  int num_samples = 100;
  vector<int> indices;
  vector<float> squared_distances;
  double mean_distance = 0;
  for(int i = 0; i < num_samples; ++i) {
    Point pt = (*prev.cloud_)[rand() % prev.cloud_->size()];
    indices.clear();
    squared_distances.clear();
    curr.kdtree_->nearestKSearch(pt, 1, indices, squared_distances);
    ROS_ASSERT(squared_distances.size() == 1);
    mean_distance += sqrt(squared_distances[0]);
  }
  mean_distance /= num_samples;

  return mean_distance;
}

cv::Mat3b Tracker::draw() const
{
  if(tracks_.empty())
    return cv::Mat3b();

  int width = tracks_.begin()->second->width_;
  int height = tracks_.begin()->second->height_;

  cv::Mat3b img(cv::Size(width, height));
  draw(img);
  return img;
}

void Tracker::draw(cv::Mat3b img) const
{
  // -- Draw the points.
  map<size_t, Blob::Ptr>::const_iterator it;
  for(it = tracks_.begin(); it != tracks_.end(); ++it) {
    size_t track_id = it->first;
    const Blob& blob = *it->second;
    ROS_ASSERT(blob.height_ == img.rows);
    // Don't show old tracks.
    if(blob.frame_id_ != frame_id_)
      continue;

    for(size_t i = 0; i < blob.indices_.size(); ++i) {
      size_t idx = blob.indices_[i];
      ROS_ASSERT(idx < (size_t)img.rows * img.cols);
      img(idx)[2] = blob.color_[i*3+0];
      img(idx)[1] = blob.color_[i*3+1];
      img(idx)[0] = blob.color_[i*3+2];
    }
  }

  // -- Set up coloring.
  cv::Vec3b color0(255, 0, 0);
  cv::Vec3b color1(0, 255, 0);
  cv::Vec3b color2(0, 0, 255);
  static map<size_t, cv::Vec3b> colormap;
  
  // -- Draw the halos.
  ROS_ASSERT(img.rows % 4 == 0 && img.cols % 4 == 0);
  cv::Mat1b mask(img.size(), 0);
  cv::Mat1b mask_small(cv::Size(img.cols / 4, img.rows / 4), 0);
  cv::Mat1b dilated_mask(mask.size(), 0);
  cv::Mat1b dilated_mask_small(mask_small.size(), 0); 
  for(it = tracks_.begin(); it != tracks_.end(); ++it) {
    size_t track_id = it->first;
    const Blob& blob = *it->second;
    ROS_ASSERT(blob.height_ == img.rows);
    // Don't show old tracks.
    if(blob.frame_id_ != frame_id_)
      continue;
    
    // Make mask for this object.
    mask = 0;
    for(size_t i = 0; i < blob.indices_.size(); ++i)
      mask(blob.indices_[i]) = 255;
    cv::resize(mask, mask_small, mask_small.size(), cv::INTER_NEAREST);

    // Get a dilated mask.
    //cv::dilate(mask, dilated_mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::GaussianBlur(mask_small, dilated_mask_small, cv::Size(9, 9), 5);
    cv::resize(dilated_mask_small, dilated_mask, mask.size(), cv::INTER_NEAREST);

    // Color all points that are in the dilated mask but not the actual mask.
    if(colormap.find(track_id) == colormap.end())
      colormap[track_id] = mix(color0, color1, color2, 0.2);
    cv::Vec3b color = colormap[track_id];
    for(int i = 0; i < mask.rows * mask.cols; ++i) {
      if(mask(i) == 0 && dilated_mask(i) != 0) {
        double coef = min(1.0, 2.0 * dilated_mask(i) / 255.0);
        img(i) = (1.0 - coef) * img(i) + coef * color;
      }
    }
  }
}

void addTimestamp(const bpt::ptime& ptime, cv::Mat3b img)
{
  ostringstream oss;
  const bpt::time_facet* f = new bpt::time_facet("%Y-%m-%d %H:%M:%S UTC%Q");
  oss.imbue(locale(oss.getloc(), f));
  oss << ptime;
  
  float thickness = 1.5;
  float scale = 0.5;
  cv::putText(img, oss.str(), cv::Point(10, img.rows - 10),
              cv::FONT_HERSHEY_SIMPLEX, scale,
              cv::Scalar(0, 255, 0), thickness, CV_AA);
}

void orient(int rotation, cv::Mat3b* img)
{
  ROS_ASSERT(rotation == 0 || rotation == 90 || rotation == 180 || rotation == 270);
  
  if(rotation == 90) {    
    cv::transpose(*img, *img);
    cv::flip(*img, *img, 0);  // Flip y.
  }
  else if(rotation == 180) {
    cv::flip(*img, *img, -1); // Flip both x and y.
  }
  else if(rotation == 270) {
    cv::transpose(*img, *img);
    cv::flip(*img, *img, 1);  // Flip y.
  }
}
