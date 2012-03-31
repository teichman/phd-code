#include <xpl_calibration/background_subtractor.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

void BackgroundSubtractor::compute()
{
  const BackgroundModel& model = *pull<const BackgroundModel*>("BackgroundModel");
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  ROS_ASSERT(seq.size() > 0);
  ROS_ASSERT(seq.pcds_[0]->size() == model.size());

  // -- Reset the foreground images and indices.
  if(fg_imgs_.size() != seq.size()) { 
    fg_imgs_.clear();
    fg_imgs_.reserve(seq.size());
    for(size_t i = 0; i < seq.size(); ++i)
      fg_imgs_.push_back(cv::Mat1b(cv::Size(seq.pcds_[0]->width, seq.pcds_[0]->height), 0));
  }
  else
    for(size_t i = 0; i < seq.size(); ++i)
      fg_imgs_[i] = 0;
  
  fg_indices_.resize(seq.size());
  for(size_t i = 0; i < fg_indices_.size(); ++i)
    fg_indices_[i].clear();

  // -- Compute foreground.
  for(size_t i = 0; i < seq.size(); ++i)
    findForeground(*seq.pcds_[i], model, &fg_indices_[i], fg_imgs_[i]);
  
  push<const vector< vector<int> >*>("ForegroundIndices", &fg_indices_);
  push<const vector<cv::Mat1b>*>("ForegroundImages", &fg_imgs_);
}

void BackgroundSubtractor::debug() const
{
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  for(size_t i = 0; i < seq.size(); ++i) {
    if(i % 25)
      continue;
    
    const Cloud& pcd = *seq.pcds_[i];
    const vector<int>& indices = fg_indices_[i];

    Cloud::Ptr fg(new Cloud);
    *fg = pcd;
    for(size_t j = 0; j < indices.size(); ++j) { 
      fg->at(indices[j]).r = 255;
      fg->at(indices[j]).g = 0;
      fg->at(indices[j]).b = 0;
    }

    ostringstream oss;
    oss << getDebugPath() << "-cloud" << setw(4) << setfill('0') << i << ".pcd";
    pcl::io::savePCDFileBinary(oss.str(), *fg);
  }
}

void BackgroundSubtractor::findForeground(const Cloud& pcd,
					  const BackgroundModel& model,
					  vector<int>* indices,
					  cv::Mat1b img) const
{
  indices->clear();

  for(size_t i = 0; i < pcd.size(); ++i) {        
    double z = pcd[i].z;
    if(!pcl_isfinite(z))
      continue;
    if(!model.isBackground(i, z)) {
      int y = i / pcd.width;
      int x = i - y * pcd.width;
      img(y, x) = 255;
    }
  }

  cv::erode(img, img, cv::Mat(), cv::Point(-1, -1), param<int>("NumErosions"));
  cv::dilate(img, img, cv::Mat(), cv::Point(-1, -1), param<int>("NumDilations"));
  
  for(int y = 0; y < img.rows; ++y)
    for(int x = 0; x < img.cols; ++x)
      if(img(y, x) != 0)
	indices->push_back(y * img.cols + x);
  
  cout << "Foreground: " << indices->size() << ", Total: " << pcd.size() << endl;
}

