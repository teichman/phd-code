#include <xpl_calibration/avv_multiview_model.h>

using namespace std;
using namespace rgbd;

AVVMultiviewModel::AVVMultiviewModel() :
  step_(1)
{
}

rgbd::Cloud::Ptr AVVMultiviewModel::filterVelo(const VeloToAsusCalibration& extrinsics,
					       rgbd::Cloud::ConstPtr velo) const
{
  Point pt;
  pt.x = 0;
  pt.y = 0;
  pt.z = 10;
  pt.getVector3fMap() = extrinsics.asusToVelo() * pt.getVector3fMap();
  double center = -atan2(pt.y, pt.x);
  double thresh = 45.0 * M_PI / 180.0;
  double theta_lower = center - thresh;
  double theta_upper = center + thresh;

  rgbd::Cloud::Ptr filtered(new Cloud);
  filtered->height = 1;
  assert(!filtered->isOrganized());
  
  filtered->reserve(velo->size() / 10.0);
  for(size_t i = 0; i < velo->size(); ++i) {
    const Point& pt = velo->at(i);
    if(!isFinite(pt))
      continue;
    double range = pt.getVector3fMap().norm();
    if(range > 15 || range < 0.5)
      continue;
    
    double yaw = -atan2(pt.y, pt.x);
    if(!(yaw > theta_lower && yaw < theta_upper))
      continue;
    
    filtered->push_back(pt);
  }

  //filtered->header.stamp.fromSec(velo->header.stamp.toSec());
  filtered->header.stamp = velo->header.stamp;
  return filtered;
}

DiscreteDepthDistortionModel AVVMultiviewModel::learnDiscreteDistortionModel() const
{
  PrimeSenseModel initial_model = sequences_[0].sseq_->model_;
  initial_model.resetDepthDistortionModel();
  DiscreteDepthDistortionModel dddm(initial_model);
  
  for(size_t i = 0; i < sequences_.size(); ++i) {
    const StreamSequence& sseq = *sequences_[i].sseq_;
    const VeloSequence& vseq = *sequences_[i].vseq_;
    const VeloToAsusCalibration& extrinsics = sequences_[i].extrinsics_;

    for(size_t i = step_; i < vseq.size(); i += step_) { 
      double dt;
      size_t idx = sseq.seek(sseq.timestamps_[0] + extrinsics.offset_ + vseq.timestamps_[i], &dt);
      if(dt > 0.01)
	continue;
      
      cout << "Adding frame " << i << endl;
      Frame frame;
      sseq.readFrame(idx, &frame);
      Cloud::Ptr filtered = filterVelo(extrinsics, vseq.getCloud(i));
      pcl::transformPointCloud(*filtered, *filtered, extrinsics.veloToAsus());
      Frame vframe;
      initial_model.cloudToFrame(*filtered, &vframe);
      dddm.accumulate(vframe, frame);
    }
  }

  return dddm;
}

rgbd::PrimeSenseModel AVVMultiviewModel::learnDistortionModel() const
{
  PrimeSenseModel initial_model = sequences_[0].sseq_->model_;
  initial_model.resetDepthDistortionModel();
  DepthDistortionLearner ddl(initial_model);
  ddl.use_filters_ = false;

  for(size_t i = 0; i < sequences_.size(); ++i) {
    const StreamSequence& sseq = *sequences_[i].sseq_;
    const VeloSequence& vseq = *sequences_[i].vseq_;
    const VeloToAsusCalibration& extrinsics = sequences_[i].extrinsics_;

    for(size_t i = step_; i < vseq.size(); i += step_) { 
      double dt;
      size_t idx = sseq.seek(sseq.timestamps_[0] + extrinsics.offset_ + vseq.timestamps_[i], &dt);
      if(dt > 0.01)
	continue;
      
      cout << "Adding frame " << i << endl;
      Frame frame;
      sseq.readFrame(idx, &frame);
      Cloud::Ptr filtered = filterVelo(extrinsics, vseq.getCloud(i));
      ddl.addFrame(frame, filtered, extrinsics.veloToAsus().cast<double>());
    }
  }

  PrimeSenseModel model = ddl.fitModel();
  return model;
}


// VeloToAsusCalibrator AVVMultiviewModel::setupCalibrator(const PrimeSenseModel& initial_model) const
// {
//   VeloToAsusCalibrator calibrator(initial_model, NULL);

//   for(size_t i = 0; i < sseqs_.size(); ++i)
//     updateCalibrator(*sseqs_[i], *vseqs_[i], extrinsics_[i], &calibrator);

// }

// void AVVMultiviewModel::updateCalibrator(const StreamSequence& sseq, const VeloSequence& vseq,
// 					 const VeloToAsusCalibration& extrinsics,
// 					 VeloToAsusCalibrator* calibrator) const
// {
//   // -- Choose Velodyne keyframes.
//   int max_num_keyframes = 3000;
//   int num_keyframes = 0;
//   int buffer = 100;
//   int spacing = 5;
//   int idx = buffer;
//   while(true) {
//     rgbd::Cloud::Ptr pcd = filterVelo(vseq.getCloud(idx));
//     // Apply initial transform.  Grid search will return a transform to apply on top of the initial one.
//     pcl::transformPointCloud(*pcd, *pcd, extrinsics.veloToAsus());  
//     // Apply initial sync offset.  Grid search will return an update to add to extrinsics.offset_.
//     pcd->header.stamp.fromSec(pcd->header.stamp.toSec() + extrinsics.offset_);  
//     calibrator->pcds_.push_back(pcd);
//     ++num_keyframes;
    
//     idx += spacing;
//     if(num_keyframes >= max_num_keyframes)
//       break;
//     if(idx > ((int)vseq.size() - buffer))
//       break;
//   }
//   cout << "Loaded " << calibrator->pcds_.size() << " velodyne keyframes." << endl;
  
//   // -- Load Asus frames in the vicinity of the Velodyne keyframes.
//   int window = 30;
//   for(size_t i = 0; i < calibrator.pcds_.size(); ++i) {
//     int idx = findAsusIdx(calibrator.pcds_[i]->header.stamp.toSec());
//     cout << "- Loading nearby asus frames for velo keyframe " << i << " / " << calibrator.pcds_.size() << endl;

//     vector<size_t> indices;
//     for(int j = max(0, idx - window); j <= min(idx + window, (int)sseq_->size()); ++j)
//       indices.push_back(j);
    
//     vector<Frame> frames(indices.size());
//     #pragma omp parallel for
//     for(size_t j = 0; j < indices.size(); ++j) { 
//       sseq_->readFrame(indices[j], &frames[j]);
//       model_.undistort(&frames[j]);  // Very important: get the best extrinsics using the given depth distortion model.
//       frames[j].timestamp_ -= sseq_start_;
//     }
//     calibrator.frames_.insert(calibrator.frames_.end(), frames.begin(), frames.end());
    
//     // for(int j = max(0, idx - window); j <= min(idx + window, (int)sseq_->size()); ++j) {
//     //   Frame frame;
//     //   sseq_->readFrame(j, &frame);
//     //   model_.undistort(&frame);  // Very important: get the best extrinsics using the given depth distortion model.
//     //   frame.timestamp_ -= sseq_start_;
//     //   calibrator.frames_.push_back(frame);
//     //   cout << j << " ";
//     // }
//     cout << endl;
//   }

//   return calibrator;
// }
