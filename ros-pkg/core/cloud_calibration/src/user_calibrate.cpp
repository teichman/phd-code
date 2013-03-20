#include <cloud_calibration/user_calibrator.h>
#include <cloud_calibration/checker_calibrator.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/stream_sequence.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace cloud_calibration;
using rgbd::StreamSequence;

string usageString()
{
  ostringstream oss;
  oss << "Usage: user_calibrate SEQ SEQ EIG [DT_THRESH]" << endl;
  oss << "Where SEQ is the reference StreamSequence" << endl;
  oss << "Where EIG is the location of the output transformation .eig.txt file" << endl;
  oss << "which brings the second sequence into the first sequence's frame" << endl;
  oss << "Where DT_THRESH is how close frames must be in time to be considered" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 4) {
    cout << usageString() << endl;
    return 0;
  }
  string seq_ref_dir = argv[1];
  StreamSequence::Ptr seq_ref( new StreamSequence );
  seq_ref->load(seq_ref_dir);
  string seq_target_dir = argv[2];
  StreamSequence::Ptr seq_target( new StreamSequence );
  seq_target->load(seq_target_dir);
  string eig_out = argv[3];
  double dt_thresh;
  if(argc == 5)
    dt_thresh = atof(argv[4]);
  else
    dt_thresh = 0.02;
  // Load sequence
  MultiSequence::Ptr mseq( new MultiSequence(dt_thresh) );
  mseq->addSequence(seq_ref);
  mseq->addSequence(seq_target);
  vector<rgbd::Point> cc_ref, cc_target;
  CheckerCalibrator cc;
  float std_trans, std_rot;
  cc.calibrate(mseq, 0, 1, cc_ref, cc_target, std_trans, std_rot);
  pcl::TransformationFromCorrespondences tc;
  for(size_t i = 0; i < cc_ref.size(); i++){
    tc.add(cc_target[i].getVector3fMap(), cc_ref[i].getVector3fMap() );
  }
  cout << "Adding corresponding points from checkerboard" << endl;
  Eigen::Affine3f transf;
  //float weight = (cc_ref.size()+1) / 50.;
  float weight = 1;
  pcl::visualization::CloudViewer vis("Overlay");
  while(true){
    transf = tc.getTransformation();
    cout << transf.matrix() << endl;
    for(size_t i = 0; i < mseq->size(); i++){
      vector<Cloud::Ptr> clouds;
      mseq->getClouds(i,clouds);
      pcl::transformPointCloud(*clouds[1], *clouds[1], transf);
      Cloud::Ptr overlay( new Cloud );
      *overlay = *clouds[0];
      *overlay += *clouds[1];
      vis.showCloud( overlay);
      usleep(30 * 1000);
    }
    UserCalibrator uc("UserCalibrator", mseq, 0, 1  );
    vector<rgbd::Point> user_ref, user_target;
    uc.get_points( user_ref, user_target );
    if(user_ref.size() == 0)
      break;
    cout << "Adding " << user_ref.size() << " points with weight " << weight << endl;
    for(size_t i = 0; i < user_ref.size(); i++){
      tc.add(user_target[i].getVector3fMap(), user_ref[i].getVector3fMap(), weight );
    }
  }
  transf = tc.getTransformation();
  cout << transf.matrix() << endl; 
  cout << "Saving to " << eig_out << endl;
  eigen_extensions::saveASCII( transf.matrix(), eig_out );

  return 0;
}

