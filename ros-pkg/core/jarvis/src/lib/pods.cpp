#include <map>
#include <vector>
#include <string>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <jarvis/pods.h>

using namespace std;
using namespace Eigen;
using namespace pl;


/************************************************************
 * BlobProjector
 ************************************************************/

void BlobProjector::compute()
{
  Blob::Ptr blob = pull<Blob::Ptr>("Blob");
  if(!blob->cloud_)
    blob->project();

  push<Blob::ConstPtr>("ProjectedBlob", blob);
  push<Cloud::ConstPtr>("Cloud", blob->cloud_);
}

void BoundingBoxSize::compute()
{
  const Cloud& cloud = *pull<Cloud::ConstPtr>("Cloud");

  Vector4f minpt, maxpt;
  pcl::getMinMax3D(cloud, minpt, maxpt);
  size_ = (maxpt - minpt).head(3);

  push<const VectorXf*>("BoundingBoxSize", &size_);
}

void BoundingBoxSize::debug() const
{
  ofstream f((debugBasePath() + ".txt").c_str());
  f << "size: " << size_.transpose() << endl;
  f.close();
}



/************************************************************
 * DescriptorAggregator
 ************************************************************/

void DescriptorAggregator::compute()
{
  multiPull("Descriptors", &aggregated_);
  push<const std::vector<const VectorXf*>* >("AggregatedDescriptors", &aggregated_);

  if(dmap_.size() == 0)
    dmap_ = dmap();

  push<const NameMapping*>("DMap", &dmap_);
}

void DescriptorAggregator::debug() const
{
  ofstream f((debugBasePath() + ".txt").c_str());

  vector<string> names = upstreamOutputNames("Descriptors");
  
  int total = 0;
  for(size_t i = 0; i < aggregated_.size(); ++i) {
    f << "Descriptor " << i << " (" << names[i] << "): ";
    if(aggregated_[i]) {
      f << "present";
      total += aggregated_[i]->rows();
    }
    else
      f << "missing";
    f << endl;
  }

  f << endl;
  f.close();
}

NameMapping DescriptorAggregator::dmap() const
{
  NameMapping dmap;
  map<string, vector<const pl::Outlet*> >::const_iterator it;
  for(it = inputPipes().begin(); it != inputPipes().end(); ++it) {
    const vector<const pl::Outlet*>& outlets = it->second;
    for(size_t i = 0; i < outlets.size(); ++i) {
      Pod* upstream = outlets[i]->pod();
      string output_name = outlets[i]->name();
      dmap.addName(upstream->uniqueReadableId(output_name));
    }
  }

  return dmap;
}


/************************************************************
 * CloudOrienter
 ************************************************************/

void CloudOrienter::compute()
{
  const Blob& blob = *pull<Blob::ConstPtr>("ProjectedBlob");
  Cloud::ConstPtr cloud = blob.cloud_;

  pcl::PCA<Point> pca;
  pca.setInputCloud(cloud);
  pca.project(*cloud, *oriented_);
 
  push<Cloud::ConstPtr>("OrientedCloud", oriented_);
}

void CloudOrienter::debug() const
{
  const Blob& blob = *pull<Blob::ConstPtr>("ProjectedBlob");
  pcl::io::savePCDFileBinary(debugBasePath() + "-original.pcd", *blob.cloud_);
  pcl::io::savePCDFileBinary(debugBasePath() + "-oriented.pcd", *oriented_);
}


/************************************************************
 * GravitationalCloudOrienter
 ************************************************************/

void GravitationalCloudOrienter::setUpVector(const Eigen::Vector3f& up)
{
  up_ = up;
  up_.normalize();
  
  // Get the transform by generating three points that correspond and using SVD.
  pcl::TransformationFromCorrespondences tfc;
  tfc.add(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
  tfc.add(up_, Vector3f(0, 0, 1));
  Vector3f xrefpt(1.0 / up_(0), 1.0 / up_(1), -2.0 / up_(2));
  xrefpt.normalize();
  tfc.add(xrefpt, Vector3f(1, 0, 0));
  raw_to_up_ = tfc.getTransformation();

  // cout << up_.transpose() << endl;
  // cout << endl;
  // cout << xrefpt.transpose() << endl;
  // cout << endl;
  // cout << raw_to_up_.matrix() << endl;
  // cout << endl;
  // cout << (raw_to_up_.matrix().block<3, 3>(0, 0) * up_).transpose() << endl;
  // cout << endl;
  // cout << (raw_to_up_.matrix().block<3, 3>(0, 0) * up_ - Vector3f(0, 0, 1)).norm() << endl;

  ROS_ASSERT(fabs(xrefpt.dot(up_)) < 1e-6);
  ROS_ASSERT((raw_to_up_.matrix().block<3, 3>(0, 0) * up_ - Vector3f(0, 0, 1)).norm() < 1e-6);
}

void GravitationalCloudOrienter::compute()
{
  const Blob& blob = *pull<Blob::ConstPtr>("ProjectedBlob");
  Cloud::ConstPtr cloud = blob.cloud_;
  ROS_ASSERT(up_.rows() == 3);

  // -- Demean the input cloud.
  Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::demeanPointCloud(*cloud, centroid, *demeaned_);
  ROS_ASSERT(!demeaned_->empty());

  // -- Rotate the demeaned cloud so that it points up.  Project into the ground plane.
  pcl::transformPointCloud(*demeaned_, *upped_, raw_to_up_);
  *projected_ = *upped_;
  for(size_t i = 0; i < projected_->size(); ++i)
    projected_->at(i).z = 0;
  
  // -- Run SVD on the projection of the points into the ground plane.
  pcl::PCA<Point> pca;
  pca.setInputCloud(projected_);
  Matrix4f rotation = Matrix4f::Identity();
  rotation.block<3, 3>(0, 0) = pca.getEigenVectors();
  pcl::transformPointCloud(*upped_, *oriented_, rotation);
    
  // -- Get height of the visible object.
  Vector4f minpt, maxpt;
  pcl::getMinMax3D(*oriented_, minpt, maxpt);
  height_(0) = maxpt(2) - minpt(2);

  // -- Get the height of the highest point.  This is a surrogate for height when we
  //    can't see the full object.
  highest_point_(0) = -numeric_limits<float>::max();
  for(size_t i = 0; i < cloud->size(); ++i)
    highest_point_(0) = max(highest_point_(0), up_.dot(cloud->at(i).getVector3fMap()));
  
  push<Cloud::ConstPtr>("OrientedCloud", oriented_);
  push<const VectorXf*>("Height", &height_);
  push<const VectorXf*>("HighestPoint", &highest_point_);
}

void GravitationalCloudOrienter::debug() const
{
  const Blob& blob = *pull<Blob::ConstPtr>("ProjectedBlob");
  pcl::io::savePCDFileBinary(debugBasePath() + "-00-original.pcd", *blob.cloud_);
  pcl::io::savePCDFileBinary(debugBasePath() + "-01-demeaned.pcd", *demeaned_);
  pcl::io::savePCDFileBinary(debugBasePath() + "-02-upped.pcd", *upped_);
  pcl::io::savePCDFileBinary(debugBasePath() + "-03-oriented.pcd", *oriented_);
  ofstream f((debugBasePath() + ".txt").c_str());
  f << "height_: " << height_.transpose() << endl;
  f << "highest_point_: " << highest_point_.transpose() << endl;
  f << "raw_to_up_: " << endl << raw_to_up_.matrix() << endl;
  f.close();
}

/************************************************************
 * CentroidFinder
 ************************************************************/

void CentroidFinder::compute()
{
  const Cloud& cloud = *pull<Cloud::ConstPtr>("Cloud");
  pcl::compute3DCentroid(cloud, centroid_);
  descriptor_ = centroid_.head(3);
  push<const VectorXf*>("Centroid", &descriptor_);
}

void CentroidFinder::debug() const
{
  ofstream f((debugBasePath() + ".txt").c_str());
  f << "centroid_: " << centroid_.transpose() << endl;
  f << "descriptor_: " << descriptor_.transpose() << endl;
  f.close();
}


/************************************************************
 * IntensityHistogram
 ************************************************************/

void IntensityHistogram::compute()
{
  const Blob& blob = *pull<Blob::ConstPtr>("Blob");
  ROS_ASSERT(blob.color_.size() % 3 == 0);

  // -- Initialize bins.
  int num_bins = param<double>("NumBins");
  if(hist_.rows() != num_bins)
    hist_ = VectorXf(num_bins);
  hist_.setZero();

  // -- Accumulate counts.  Normalize for number of points.
  float bin_width = 255.0 / num_bins;
  for(size_t i = 0; i < blob.color_.size(); i+=3) {
    float intensity = ((float)blob.color_[i] + blob.color_[i+1] + blob.color_[i+2]) / 3.0;
    int idx = max<int>(0, min<int>(num_bins - 1, intensity / bin_width));
    ++hist_(idx);
  }
  hist_ /= hist_.sum();
  
  push<const VectorXf*>("Histogram", &hist_);
}

void IntensityHistogram::debug() const
{
  ofstream f((debugBasePath() + ".txt").c_str());
  f << "hist_: " << hist_.transpose() << endl;
  f.close();
}


/************************************************************
 * NormalizedDensityHistogram
 ************************************************************/

void NormalizedDensityHistogram::compute()
{
  const Cloud& cloud = *pull<Cloud::ConstPtr>("Cloud");
  
  // -- Initialize bins if necessary.
  ROS_ASSERT(lower_limits_.size() == 3 && bins_.size() == 3);
  int num_bins = param<double>("NumBins");
  if(lower_limits_[0].rows() != num_bins) {
    for(size_t i = 0; i < lower_limits_.size(); ++i) {
      lower_limits_[i] = VectorXf::Zero(num_bins);
      bins_[i] = VectorXf::Zero(num_bins);
    }
  }

  // -- Compute min and max.
  Vector4f minpt, maxpt;
  pcl::getMinMax3D(cloud, minpt, maxpt);

  
  for(int i = 0; i < 3; ++i) {
    // Set lower limits of the bins.
    VectorXf& lower_limits = lower_limits_[i];
    VectorXf& bins = bins_[i];
    float minval = minpt(i);
    float maxval = maxpt(i);
    float binwidth = (maxval - minval) / num_bins;
    for(int j = 0; j < lower_limits.rows(); ++j)
      lower_limits(j) = minval + binwidth * j;
    
    // Fill bins with counts.
    for(size_t j = 0; j < cloud.size(); ++j) {
      float val = cloud[j].getVector3fMap().coeffRef(i);
      int idx = max<int>(0, min<int>(num_bins - 1, (val - minval) / binwidth));
      ++bins.coeffRef(idx);
    }

    // Normalize to sum to one.
    bins /= bins.sum();
    push<const VectorXf*>(names_[i], &bins);
  }
}

void NormalizedDensityHistogram::debug() const
{
  ofstream f((debugBasePath() + ".txt").c_str());
  for(size_t i = 0; i < lower_limits_.size(); ++i) {
    f << names_[i] << endl;
    f << "  lower_limits_: " << lower_limits_[i].transpose() << endl;
    f << "  bins_: " << bins_[i].transpose() << endl;
  }
  f.close();
}

/************************************************************
 * CloudProjector
 ************************************************************/

void CloudProjector::compute()
{
  const Cloud& pcd = *pull<Cloud::ConstPtr>("Cloud");

  // -- Initialize data.
  int num_rows = param<double>("NumRows");
  int num_cols = param<double>("NumCols");
  if(img1f_.rows != num_rows || img1f_.cols != num_cols) {
    img1f_ = cv::Mat1f(cv::Size(num_cols, num_rows));
    counts_ = cv::Mat1f(cv::Size(num_cols, num_rows));
    vectorized_.resize(num_rows * num_cols);
  }
  img1f_ = 0;
  counts_ = 0;
  vectorized_.setZero();

  string view = param<string>("View");
  double ppm = param<double>("PixelsPerMeter");
  float min_intensity = param<double>("MinIntensity");
  float mult = (1.0 - min_intensity) / 255.0;
  if(view == "XZ") {
    for(size_t i = 0; i < pcd.size(); ++i) {
      int u = pcd[i].x * ppm + num_cols / 2;
      int v = -pcd[i].z * ppm + num_rows / 2;
      float intensity = min_intensity + pcd[i].r * mult;
      increment(v, u, intensity);
    }
  }
  else if(view == "YZ") {
    for(size_t i = 0; i < pcd.size(); ++i) {
      int u = pcd[i].y * ppm + num_cols / 2;
      int v = -pcd[i].z * ppm + num_rows / 2;
      float intensity = min_intensity + pcd[i].r * mult;
      increment(v, u, intensity);
    }
  }
  else if(view == "XY") {
    for(size_t i = 0; i < pcd.size(); ++i) {
      int u = pcd[i].x * ppm + num_rows / 2;
      int v = pcd[i].y * ppm + num_cols / 2;
      float intensity = min_intensity + pcd[i].r * mult;
      increment(v, u, intensity);
    }
  }
  else
    ROS_ASSERT(0);
    
  // -- Normalize, smooth, and vectorize.
  for(int v = 0; v < img1f_.rows; ++v)
    for(int u = 0; u < img1f_.cols; ++u)
      if(counts_(v, u) > 0)
        img1f_(v, u) /= counts_(v, u);
  int ksize = param<double>("KernelSize");
  if(ksize > 0) {
    ROS_ASSERT(ksize % 2 == 1);
    cv::GaussianBlur(img1f_, img1f_, cv::Size(ksize, ksize), 0);
  }

  img1f_.convertTo(img_, CV_8UC1, -255, 255);
  push("Image", img_);
}

inline void CloudProjector::increment(int v, int u, float intensity)
{
  if(u < 0 || u >= img1f_.cols || v < 0 || v >= img1f_.rows)
    return;
  
  ++counts_(v, u);
  img1f_(v, u) += intensity;
}

void CloudProjector::debug() const
{ 
  cv::Mat1b vis;
  float scale = 10;
  cv::resize(img_, vis, cv::Size(img_.cols * scale, img_.rows * scale), cv::INTER_NEAREST);
  cv::imwrite(debugBasePath() + ".png", vis);

  ofstream f((debugBasePath() + ".txt").c_str());
  const Cloud& pcd = *pull<Cloud::ConstPtr>("Cloud");
  Vector4f mean;
  pcl::compute3DCentroid(pcd, mean);
  f << "Mean: " << mean.transpose() << endl;
  f << "Vectorized: " << vectorized_.transpose() << endl;
  f.close();
}


/************************************************************
 * DynamicImageWindow
 ************************************************************/

void DynamicImageWindow::compute()
{
  cv::Mat1b src = pull<cv::Mat1b>("Image");

  // -- Set up the roi width and height.
  double width_pct = param<double>("WidthPercent");
  double height_pct = param<double>("HeightPercent");
  ROS_ASSERT(width_pct > 0 && width_pct <= 1.0);
  ROS_ASSERT(height_pct > 0 && height_pct <= 1.0);
  cv::Rect roi;
  roi.width = src.cols * width_pct;
  roi.height = src.rows * height_pct;

  // -- Set up the size of the final image.
  double scaling = param<double>("Scaling");
  ROS_ASSERT(scaling <= 1.0 && scaling > 0);
  int rows = roi.height * scaling;
  int cols = roi.width * scaling;
  if(img_.rows != rows || img_.cols != cols) {
    img_ = cv::Mat1b(cv::Size(cols, rows));
    vectorized_.resize(rows * cols);
  }
  img_ = 0;
  vectorized_.setZero();

  // -- Get min and max, with a small buffer.
  int min_u = src.cols - 1;
  int max_u = 0;
  int min_v = src.rows - 1;
  int max_v = 0;
  for(int v = 0; v < src.rows; ++v) {
    for(int u = 0; u < src.cols; ++u) {
      if(src(v, u) < 255) {
        min_u = min(u, min_u);
        max_u = max(u, max_u);
        min_v = min(v, min_v);
        max_v = max(v, max_v);
      }
    }
  }
  int buffer = 2;
  min_u = max(min_u - buffer, 0);
  min_v = max(min_v - buffer, 0);
  max_u = min(max_u + buffer, src.cols - 1);
  max_v = min(max_v + buffer, src.rows - 1);
  
  // -- Determine the roi position in the source image
  //    based on where the image is not zero.
  string va = param<string>("VerticalAlignment");
  if(va == "Top")
    roi.y = min(min_v, src.rows - roi.height);
  else if(va == "Center")
    roi.y = src.rows / 2 - roi.height / 2;
  else if(va == "Bottom")
    roi.y = max(max_v - roi.height, 0);
  else
    ROS_ASSERT(0);
  
  string ha = param<string>("HorizontalAlignment");
  if(ha == "Left")
    roi.x = min(min_u, src.cols - roi.width);
  else if(ha == "Center")
    roi.x = src.cols / 2 - roi.width / 2;
  else if(ha == "Right")
    roi.x = max(max_u - roi.width, 0);
  else
    ROS_ASSERT(0);

  // -- Copy and resize.
  cv::Mat window = src(roi);
  cv::resize(window, img_, img_.size(), 0, 0, cv::INTER_NEAREST);
  ROS_ASSERT(vectorized_.rows() == img_.rows * img_.cols);
  int idx = 0;
  for(int v = 0; v < img_.rows; ++v)
    for(int u = 0; u < img_.cols; ++u, ++idx)
      vectorized_.coeffRef(idx) = img_(v, u);
  
  push<cv::Mat1b>("Image", img_);
  push<const VectorXf*>("Vectorized", &vectorized_);
}

void DynamicImageWindow::debug() const
{
  cv::Mat1b vis;
  float scale = 10;
  cv::resize(img_, vis, cv::Size(img_.cols * scale, img_.rows * scale), cv::INTER_NEAREST);
  cv::imwrite(debugBasePath() + "-scaled.png", vis);
}


/************************************************************
 * HogArray
 ************************************************************/

void HogArray::compute()
{
  cv::Mat1b img = pull<cv::Mat1b>("Image");

  // -- Initialize HOG if necessary.
  if(!hog_) {
    // -- Set up UVPattern.
    if(param<string>("UVPattern") == "Original") {
      // 0.28ms
      u_offset_pcts_.push_back(0.0); v_offset_pcts_.push_back(1.0);
      u_offset_pcts_.push_back(0.5); v_offset_pcts_.push_back(0.0);
      u_offset_pcts_.push_back(0.5); v_offset_pcts_.push_back(0.5);
      u_offset_pcts_.push_back(1.0); v_offset_pcts_.push_back(1.0);
    }
    else if(param<string>("UVPattern") == "Center") {
      // 0.18ms
      u_offset_pcts_.push_back(0.5); v_offset_pcts_.push_back(0.5);
    }
    else if(param<string>("UVPattern") == "Dense") {
      // 0.31ms
      u_offset_pcts_.push_back(0.0); v_offset_pcts_.push_back(0.0);
      u_offset_pcts_.push_back(0.0); v_offset_pcts_.push_back(0.5);
      u_offset_pcts_.push_back(0.0); v_offset_pcts_.push_back(1.0);
      u_offset_pcts_.push_back(0.5); v_offset_pcts_.push_back(0.0);
      u_offset_pcts_.push_back(0.5); v_offset_pcts_.push_back(0.5);
      u_offset_pcts_.push_back(0.5); v_offset_pcts_.push_back(1.0);
      u_offset_pcts_.push_back(1.0); v_offset_pcts_.push_back(0.0);
      u_offset_pcts_.push_back(1.0); v_offset_pcts_.push_back(0.5);
      u_offset_pcts_.push_back(1.0); v_offset_pcts_.push_back(1.0);
    }
    else {
      ROS_ASSERT(0);
    }
    
    // -- Set up HOG params.
    win_size_ = cv::Size(param<double>("WindowWidth"), param<double>("WindowHeight"));
    block_size_ = cv::Size(param<double>("BlockWidth"), param<double>("BlockHeight"));
    block_stride_ = cv::Size(param<double>("BlockStride"), param<double>("BlockStride"));
    cell_size_ = cv::Size(param<double>("CellSize"), param<double>("CellSize"));
    num_bins_ = param<double>("NumBins");
    ROS_ASSERT(img.cols >= win_size_.width && img.rows >= win_size_.height);
    
    hog_ = new cv::HOGDescriptor(win_size_, block_size_, block_stride_,
                                 cell_size_, num_bins_,
                                 1, -1, 0, 0.2, false); 
    
    // -- Get list of Points to do computation at from u and v offset percents.
    coords_ = vector<cv::Point>(u_offset_pcts_.size());
    for(size_t i = 0; i < u_offset_pcts_.size(); i++) {
      int u = u_offset_pcts_[i] * img.cols;
      int v = v_offset_pcts_[i] * img.rows;
      
      //Subtracting off half winSize since the feature is computed in a window where location[i] is 
      //the upper left corner.  points[i] is the center of the window.
      coords_[i] = cv::Point(u - hog_->winSize.width / 2, v - hog_->winSize.height / 2);
    }

    // -- Shift any points so that they don't make the window fall off the edge of the image.
    for(size_t i = 0; i < coords_.size(); i++) {
      if(coords_[i].x + hog_->winSize.width >= img.cols)
        coords_[i].x = img.cols - hog_->winSize.width;
      else if(coords_[i].x < 0)
        coords_[i].x = 0;

      if(coords_[i].y + hog_->winSize.height >= img.rows)
        coords_[i].y = img.rows - hog_->winSize.height;
      else if(coords_[i].y < 0)
        coords_[i].y = 0;
    }
  }

  // -- Call opencv.
  //    winStride and padding are set to default
  cv_result_.clear();
  hog_->compute(img, cv_result_, cv::Size(), cv::Size(), coords_);

  concatenated_.resize(cv_result_.size());  // This is a no-op most of the time.
  for(int i = 0; i < concatenated_.rows(); ++i)
    concatenated_.coeffRef(i) = cv_result_[i];
  
  push<const Eigen::VectorXf*>("ConcatenatedDescriptors", &concatenated_);
}

ostream& operator<<(ostream& out, const cv::Size& sz)
{
  out << sz.width << " " << sz.height;
  return out;
}

void HogArray::debug() const
{
  cv::Mat1b img = pull<cv::Mat1b>("Image");
  cv::imwrite(debugBasePath() + "-original_image.png", img);
  
  ofstream f((debugBasePath() + ".txt").c_str());
  f << "win_size_: " << win_size_ << endl;
  f << "block_size_: " << block_size_ << endl;
  f << "block_stride_: " << block_stride_ << endl;
  f << "cell_size_: " << cell_size_ << endl;
  f << "num_bins_: " << num_bins_ << endl;
  f << "offset pcts: " << endl;
  for(size_t i = 0; i < u_offset_pcts_.size(); ++i)
    f << "  " << u_offset_pcts_[i] << " " << v_offset_pcts_[i] << endl;
  f << "coords_: " << endl;
  for(size_t i = 0; i < coords_.size(); ++i)
    f << "  " << coords_[i] << endl;
  f << "concatenated_: " << concatenated_.transpose() << endl;
  f.close();
}
