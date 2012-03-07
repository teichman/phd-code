#include <cloud_calibration/checker_calibrator.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transformation_from_correspondences.h>

#define VISUALIZE (getenv("VISUALIZE") ? atoi(getenv("VISUALIZE")) : 0)
#define INTERACTIVE (getenv("INTERACTIVE") ? atoi(getenv("INTERACTIVE")) : 0)

namespace cloud_calibration
{
  using std::vector;
  using std::cout;
  using std::endl;
  using cv::TermCriteria;
  using cv::Mat;

  CheckerCalibrator::CheckerCalibrator():
    checker_rows_(6), checker_cols_(7)
  {
  }
  
  CheckerCalibrator::CheckerCalibrator(int rows, int cols):
    checker_rows_(rows), checker_cols_(cols)
  {
  }
  
  Eigen::Affine3f CheckerCalibrator::calibrate( const StreamSequence::Ptr &seq_ref, 
      const StreamSequence::Ptr &seq_target, double dt_thresh) const
  {
    // Create a multi sequence from the two
    MultiSequence::Ptr seq( new MultiSequence(dt_thresh) );
    seq->addSequence(seq_ref);
    seq->addSequence(seq_target);
    return calibrate(seq, 0, 1);
  }

  Eigen::Affine3f CheckerCalibrator::calibrate( const MultiSequence::ConstPtr &seq, 
                             size_t ref_idx, size_t target_idx ) const
  {
    cout << "Considering " << seq->size() << " frames" << endl;
    //Iterate over the board being flipped or not
    Eigen::Affine3f trans;
    float bestVar;
    float reject_pct;
    for(int flip = 0; flip <= 1; flip++){
      cout << "Hypothesis: flip = " << flip << endl;
      vector<vector<Point> > points_ref_all, points_target_all;
      vector<Eigen::Affine3f> transforms;
      for(int fast_check = 1; fast_check >= 0; fast_check--){
        for(size_t i = 0; i < seq->size(); i++)
        {
          vector<Point> points_ref, points_target;
          Eigen::Affine3f frame_transform;
          bool found = estimateAffineFromFrame(seq, i, ref_idx, target_idx, frame_transform, points_ref, points_target,
              (bool) flip, (bool) fast_check);
          if( found ){
            transforms.push_back(frame_transform);
            points_ref_all.push_back(points_ref);
            points_target_all.push_back(points_target);
          }
        }
        if(transforms.size() > 0 && fast_check){
          reject_pct = 0.25;
          break;
        }
        else{
          reject_pct = 0.5;
          cout << "Couldn't find a checkerboard. Trying more in-depth search (slow)" << endl;
        }
      }
      // Estimate the transform throw out the furthest 25% from the mean
      ROS_ASSERT(transforms.size() > 0);
      cout << "Found " << transforms.size() << " valid checkerboard frames" << endl;
      Eigen::Matrix4f mean_transform = Eigen::Matrix4f::Zero();
      for(size_t i = 0; i < transforms.size(); i++){
        mean_transform += transforms[i].matrix()/float(transforms.size());
      }
      // Compute variance
      Eigen::VectorXf variances(transforms.size());
      for(size_t i = 0; i < transforms.size(); i++){
        Eigen::Matrix4f diff = transforms[i].matrix()-mean_transform;
        variances(i) = diff.squaredNorm();
      }
      // Store variance for later
      if(flip==0)
        bestVar = variances.sum();
      else{
        float curVar = variances.sum();
        cout << "Variance std: " << bestVar << ", Variance flip: " << curVar << endl;
        if( curVar > bestVar ){
          cout << "Chose not to flip" << endl;
          return trans;
        } else{
          cout << "Chose to flip" << endl;
        }
      }
      // Throw all but N into the best section
      vector<Point> points_ref_best, points_target_best;
      for(size_t i = 0; i < (1-reject_pct)*transforms.size(); i++){
        cout << "Rejecting " << 100*reject_pct << "\% highest variance samples" << endl;
        size_t frame;
        variances.minCoeff(&frame);
        //Make sure this won't be counted again
        variances(frame) = std::numeric_limits<float>::infinity();
        //Add it to points_ref_best, points_target_best
        for(size_t j = 0; j < points_ref_all[frame].size(); j++){
          points_ref_best.push_back(points_ref_all[frame][j]);
          points_target_best.push_back(points_target_all[frame][j]);
        }
      }
      trans = estimateAffine(points_ref_best, points_target_best);
    }

    return trans;
  }



  bool CheckerCalibrator::estimateAffineFromFrame( const MultiSequence::ConstPtr &seq, 
      size_t frame, size_t ref_idx, size_t target_idx, 
      Eigen::Affine3f &trans, vector<Point> &points_ref, vector<Point> &points_target,
      bool flip, bool fast_check, bool interactive) const
  {
    vector<Mat3b> imgs;
    seq->getImages( frame, imgs );
    Mat3b& img_ref = imgs[ref_idx], img_target = imgs[target_idx];
    //Find the checkerboard -- assumes first point is above last, unless flip is set
    vector<cv::Point2f> corners_ref, corners_target;
    int flags;
    if(fast_check)
      flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK;
    else
      flags = cv::CALIB_CB_ADAPTIVE_THRESH;
    if(!cv::findChessboardCorners(img_ref, cv::Size(checker_cols_, checker_rows_), corners_ref, 
                      flags ) )
      return false;
    if(!cv::findChessboardCorners(img_target, cv::Size(checker_cols_, checker_rows_), corners_target, 
                      flags ) )
      return false;
    //Do flipping
    if(corners_ref[0].y > corners_ref[corners_ref.size()-1].y)
      std::reverse(corners_ref.begin(),corners_ref.end());
    if(corners_target[0].y > corners_target[corners_target.size()-1].y)
      std::reverse(corners_target.begin(),corners_target.end());
    if(flip)
      std::reverse(corners_target.begin(),corners_target.end());
    //If interactive, ask user if this is good
    if(VISUALIZE){
      Mat3b drawn_ref, drawn_target;
      img_ref.copyTo(drawn_ref);
      img_target.copyTo(drawn_target);
      cv::drawChessboardCorners( drawn_ref, cv::Size(checker_cols_, checker_rows_ ), 
          corners_ref, true );
      cv::drawChessboardCorners( drawn_target, cv::Size(checker_cols_, checker_rows_ ), 
          corners_target, true );
      cv::circle( drawn_ref, corners_ref[0], 8, cv::Scalar(0,0,0), -1 );
      cv::circle( drawn_target, corners_target[0], 8, cv::Scalar(0,0,0), -1 );
      cv::imshow("Reference Image", drawn_ref);
      cv::imshow("Target Image", drawn_target);
      cv::waitKey(10);
    } 
    if(VISUALIZE && (interactive || INTERACTIVE)){
      cout << "Hit y to use this image, f to flip it" << endl;
      char result = cv::waitKey(0);
      if(result == 'f'){
        std::reverse(corners_target.begin(),corners_target.end());
      }else if(result != 'y'){
        return false;
      }
    }
    //Lookup corresponding points in cloud
    vector<Cloud::Ptr> clouds;
    seq->getClouds( frame, clouds );
    Cloud::Ptr& cloud_ref = clouds[ref_idx], cloud_target = clouds[target_idx];
    int found_count = 0;
    for(size_t i = 0; i < corners_ref.size(); i++){
      Point& pt_ref = cloud_ref->at(corners_ref[i].x, corners_ref[i].y);
      if(isnan(pt_ref.z))
        continue;
      Point& pt_target = cloud_target->at(corners_target[i].x, corners_target[i].y);
      if(isnan(pt_target.z))
        continue;
      //If both are good, add them to the points
      points_ref.push_back(pt_ref);
      points_target.push_back(pt_target);
      found_count++;
    }
    if(found_count < 3){
      return false;
    }
    trans = estimateAffine(points_ref, points_target);
    return true;  
  }

  Eigen::Affine3f CheckerCalibrator::estimateAffine( const vector<Point> &points_ref, 
      const vector<Point> &points_target ) const
  {
    pcl::TransformationFromCorrespondences tc;
    for(size_t i = 0; i < points_ref.size(); i++)
    {
      tc.add(pointToEigen(points_target[i]), pointToEigen(points_ref[i]));
    }
    return tc.getTransformation();
  }
    
  Eigen::Vector3f pointToEigen( const Point &point )
  {
    return Eigen::Vector3f( point.x, point.y, point.z );
  }
}
