#ifndef JARVIS_PODS_H
#define JARVIS_PODS_H

#include <opencv2/objdetect/objdetect.hpp>
#include <pipeline/pipeline.h>
#include <name_mapping/name_mapping.h>
#include <jarvis/tracker.h>
#include <eigen_extensions/random.h>

class BlobProjector : public pl::Pod
{
public:
  DECLARE_POD(BlobProjector);
  BlobProjector(std::string name) :
    Pod(name)
  {
    declareInput<Blob::Ptr>("Blob");
    declareOutput<Blob::ConstPtr>("ProjectedBlob");
    declareOutput<Cloud::ConstPtr>("Cloud");
  }

  void compute();
};

class BoundingBoxSize : public pl::Pod
{
public:
  DECLARE_POD(BoundingBoxSize);
  BoundingBoxSize(std::string name) :
    Pod(name),
    size_(Eigen::VectorXf::Zero(3))
  {
    declareInput<Cloud::ConstPtr>("Cloud");
    declareOutput<const Eigen::VectorXf*>("BoundingBoxSize");
  }

  void compute();
  void debug() const;

protected:
  Eigen::VectorXf size_;
};

class IntensityHistogram : public pl::Pod
{
public:
  DECLARE_POD(IntensityHistogram);
  IntensityHistogram(std::string name) :
    Pod(name)
  {
    declareInput<Blob::ConstPtr>("Blob");
    declareParam<double>("NumBins", 10);
    declareOutput<const Eigen::VectorXf*>("Histogram");
  }

  void compute();
  void debug() const;

protected:
  Eigen::VectorXf hist_;
};

class HSVHistogram : public pl::Pod
{
public:
  DECLARE_POD(HSVHistogram);
  HSVHistogram(std::string name) :
    Pod(name)
  {
    declareInput<Blob::ConstPtr>("Blob");
    declareParam<double>("NumBins", 10);
    declareParam<double>("ValueThreshold", 0.2);  // Pixel must be brighter than this to use its hue & saturation.  [0, 1].
    declareParam<double>("SaturationThreshold", 0.2);  // Pixel must be more saturated than this to use its hue.  [0, 1].
    declareOutput<const Eigen::VectorXf*>("Hue");
    declareOutput<const Eigen::VectorXf*>("Saturation");
    declareOutput<const Eigen::VectorXf*>("Value");
  }

  void compute();
  void debug() const;

protected:
  cv::Mat3f hsv_;
  Eigen::VectorXf hue_;
  Eigen::VectorXf sat_;
  Eigen::VectorXf val_;
};


class DescriptorConcatenator : public pl::Pod
{
public:
  DECLARE_POD(DescriptorConcatenator);
  DescriptorConcatenator(std::string name) :
    Pod(name)
  {
    declareMultiInput<const Eigen::VectorXf*>("Descriptors");
    declareOutput<const Eigen::VectorXf*>("ConcatenatedDescriptors");
  }

protected:
  Eigen::VectorXf concatenated_;
  
  void compute();
  void debug() const;
};

class DescriptorAggregator : public pl::Pod
{
public:
  DECLARE_POD(DescriptorAggregator);
  DescriptorAggregator(std::string name) :
    Pod(name)
  {
    declareMultiInput<const Eigen::VectorXf*>("Descriptors");
    declareOutput<const std::vector<const Eigen::VectorXf*>* >("AggregatedDescriptors");
    declareOutput<const NameMapping*>("DMap");
  }

  NameMapping dmap() const;
  
protected:
  std::vector<const Eigen::VectorXf*> aggregated_;
  NameMapping dmap_;
  
  void compute();
  void debug() const;
};

//! Uses SVD to rotate the cloud so that x is the long axis, z is the short axis, and y is in between.
class CloudOrienter : public pl::Pod
{
public:
  DECLARE_POD(CloudOrienter);
  CloudOrienter(std::string name) :
    Pod(name),
    oriented_(new Cloud),
    relative_curvature_(1)
  {
    declareInput<Blob::ConstPtr>("ProjectedBlob");
    declareParam<bool>("OrientCloud", true);  // If you're only using the descriptors, you can set this to false.
    declareOutput<Cloud::ConstPtr>("OrientedCloud");
    declareOutput<const Eigen::VectorXf*>("Eigenvalues");
    declareOutput<const Eigen::VectorXf*>("RelativeCurvature");
  }

  void compute();
  void debug() const;

protected:
  Cloud::Ptr oriented_;
  Eigen::VectorXf eigenvalues_;
  Eigen::VectorXf relative_curvature_;
  Eigen::Matrix3f eigenvectors_;
};

class GravitationalCloudOrienter : public pl::Pod
{
public:
  DECLARE_POD(GravitationalCloudOrienter);
  GravitationalCloudOrienter(std::string name) :
    Pod(name),
    demeaned_(new Cloud),
    upped_(new Cloud),
    projected_(new Cloud),
    oriented_(new Cloud),
    height_(1),
    highest_point_(1)
  {
    declareInput<Blob::ConstPtr>("ProjectedBlob");
    declareOutput<Cloud::ConstPtr>("OrientedCloud");
    declareOutput<const Eigen::VectorXf*>("Height");
    declareOutput<const Eigen::VectorXf*>("HighestPoint");
  }

  //! This must be called before compute().
  void setUpVector(const Eigen::Vector3f& up);
  void compute();
  void debug() const;

protected:
  Eigen::VectorXf up_;
  Eigen::Affine3f raw_to_up_;
  Cloud::Ptr demeaned_;
  Cloud::Ptr upped_;
  Cloud::Ptr projected_;
  Cloud::Ptr oriented_;
  Eigen::VectorXf height_;
  Eigen::VectorXf highest_point_;
};

class CloudSelector : public pl::Pod
{
public:
  DECLARE_POD(CloudSelector);
  CloudSelector(std::string name) :
    Pod(name)
  {
    declareInput<Blob::ConstPtr>("ProjectedBlob");
    declareOutput<Cloud::ConstPtr>("Cloud");
  }

  void compute() {
    Cloud::ConstPtr cloud = pull<Blob::ConstPtr>("ProjectedBlob")->cloud_;
    push("Cloud", cloud);
  }
};

class CentroidFinder : public pl::Pod
{
public:
  DECLARE_POD(CentroidFinder);
  CentroidFinder(std::string name) :
    Pod(name)
  {
    declareInput<Cloud::ConstPtr>("Cloud");
    declareOutput<const Eigen::VectorXf*>("Centroid");
  }

  void compute();
  void debug() const;

protected:
  Eigen::Vector4f centroid_;
  Eigen::VectorXf descriptor_;
};

class NormalizedDensityHistogram : public pl::Pod
{
public:
  DECLARE_POD(NormalizedDensityHistogram);
  NormalizedDensityHistogram(std::string name) :
    Pod(name),
    lower_limits_(3),
    bins_(3)
  {
    names_.push_back("X");
    names_.push_back("Y");
    names_.push_back("Z");

    declareInput<Cloud::ConstPtr>("Cloud");
    declareParam<double>("NumBins", 10);
    declareOutput<const Eigen::VectorXf*>("X");
    declareOutput<const Eigen::VectorXf*>("Y");
    declareOutput<const Eigen::VectorXf*>("Z");
  }

  void compute();
  void debug() const;
  
protected:
  std::vector<Eigen::VectorXf> lower_limits_;
  std::vector<Eigen::VectorXf> bins_;
  std::vector<std::string> names_;
};

class NormalsPod : public pl::Pod
{
public:
  DECLARE_POD(NormalsPod);
  NormalsPod(std::string name) :
    Pod(name)
  {
    declareInput<Cloud::ConstPtr>("Cloud");
    declareParam<double>("Radius", 0.05);
    declareOutput<NormalsCloud>("Normals");
  }

  void compute();
  void debug() const;
};

//! Projects canonically-oriented clusters into a virtual orthographic camera image plane.
class CloudProjector : public pl::Pod
{
public:
  DECLARE_POD(CloudProjector);
  CloudProjector(std::string name) :
    Pod(name)
  {
    // Assumed to be zero-mean and oriented.
    // Assumes intensity channel can be found in pt.r.
    declareInput<Cloud::ConstPtr>("Cloud");
    // "XZ" : u <- x, v <- z.
    // "YZ" : u <- y, v <- z.
    // "XY" : u <- y, v <- x.
    declareParam<std::string>("View");  
    declareParam<double>("PixelsPerMeter");    
    declareParam<double>("NumRows");
    declareParam<double>("NumCols");
    // To distinguish min-intensity laser points
    // from the background color on the projection.
    // In [0, 1].
    declareParam<double>("MinIntensity");
    //declareParam<bool>("EqualizeHist");
    // For smoothing.  Must be positive and odd.
    declareParam<double>("KernelSize", 1);
    declareOutput<cv::Mat1b>("Image");
  }

protected:
  cv::Mat1f img1f_;
  cv::Mat1f counts_;
  cv::Mat1b img_;
  Eigen::VectorXf vectorized_;
  
  void compute();
  void debug() const;
  //! intensity is in [0, 1].
  void increment(int v, int u, float intensity);
};

class DynamicImageWindow : public pl::Pod
{
public:
  DECLARE_POD(DynamicImageWindow);
  DynamicImageWindow(std::string name) :
    Pod(name)
  {
    // Should be the same size every time.
    declareInput<cv::Mat1b>("Image");
    // "Top", "Center", "Bottom"
    declareParam<std::string>("VerticalAlignment");
    // "Left", "Center", "Right"
    declareParam<std::string>("HorizontalAlignment");
    declareParam<double>("HeightPercent");
    declareParam<double>("WidthPercent");
    // (0, 1].
    declareParam<double>("Scaling");
    declareOutput<cv::Mat1b>("Image");
    declareOutput<const Eigen::VectorXf*>("Vectorized");
  }

protected:
  cv::Mat1b img_;
  Eigen::VectorXf vectorized_;

  void compute();
  void debug() const;
};

class HogArray : public pl::Pod
{
public:
  DECLARE_POD(HogArray);
  HogArray(std::string name) :
    Pod(name),
    hog_(NULL)
  {
    declareInput<cv::Mat1b>("Image");
    declareParam<double>("WindowWidth");
    declareParam<double>("WindowHeight");
    declareParam<double>("BlockWidth");
    declareParam<double>("BlockHeight");
    declareParam<double>("BlockStride");
    declareParam<double>("CellSize");
    declareParam<double>("NumBins");
    declareParam<std::string>("UVPattern");  // Original, Dense, Center
    declareOutput<const Eigen::VectorXf*>("ConcatenatedDescriptors");
  }

  ~HogArray() { if(hog_) delete hog_; }
  
protected:
  std::vector<double> u_offset_pcts_;
  std::vector<double> v_offset_pcts_;
  cv::Size win_size_;
  cv::Size block_size_;
  cv::Size block_stride_;
  cv::Size cell_size_;
  int num_bins_;
  cv::HOGDescriptor* hog_;
  //! Locations to compute HOG, derived from {u,v}_offset_pcts_ and stored for use by HogWindow's display function.
  std::vector<cv::Point> coords_;
  std::vector<float> cv_result_;
  Eigen::VectorXf concatenated_;
  cv::Mat1b img_;

  void compute();
  void debug() const;
};

class RandomProjector : public pl::Pod
{
public:
  DECLARE_POD(RandomProjector);
  RandomProjector(std::string name) :
    Pod(name)
  {
    declareInput<const Eigen::VectorXf*>("Descriptor");
    declareParam<double>("Seed");
    declareParam<double>("NumProjections");
    declareOutput<const Eigen::VectorXf*>("Projected");
  }

protected:
  eigen_extensions::UniformSampler sampler_;
  Eigen::MatrixXf projector_;
  Eigen::VectorXf projected_;

  void compute();
  void debug() const;
  void generateProjectionMatrix(int input_dim, int output_dim, uint64_t seed,
                                Eigen::MatrixXf* projector) const;

};


#endif // JARVIS_PODS_H
