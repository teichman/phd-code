#include <cloud_calibration/checker_calibrator.h>
#include <matrix_tricks/conversions.h>
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
    focal_length_(525), checker_rows_(7), checker_cols_(9), square_size_(0.1)
  {
  }
  
  CheckerCalibrator::CheckerCalibrator(int rows, int cols, float square_size):
    focal_length_(525), checker_rows_(rows), checker_cols_(cols), square_size_(square_size)
  {
  }

  Eigen::Affine3f CheckerCalibrator::calibrate( MultiSequence::ConstPtr &seq, 
                             size_t ref_idx, size_t target_idx ) const
  {
    //Iterate over the board being flipped or not
    Eigen::Affine3f trans;
    float bestVar;
    for(int flip = 0; flip <= 1; flip++){
      vector<vector<Point> > points_ref_all, points_target_all;
      vector<Eigen::Affine3f> transforms;
      for(size_t i = 0; i < seq->size(); i++)
      {
        vector<Point> points_ref, points_target;
        Eigen::Affine3f frame_transform;
        bool found = estimateAffineFromFrame(seq, i, ref_idx, target_idx, frame_transform, points_ref, points_target,
            (bool) flip);
        if( found ){
          transforms.push_back(frame_transform);
          points_ref_all.push_back(points_ref);
          points_target_all.push_back(points_target);
        }
      }
      // Estimate the transform throw out the furthest 25% from the mean
      ROS_ASSERT(transforms.size() > 0);
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
      else if( variances.sum() > bestVar ){
        cout << "Chose not to flip" << endl;
        break;
      } else{
        cout << "Chose to flip" << endl;
      }
      // Throw all but N into the best section
      vector<Point> points_ref_best, points_target_best;
      for(size_t i = 0; i < 0.75*transforms.size(); i++){
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

    cout << "Final affine: " << trans.matrix() << endl;
    return trans;
  }

  Eigen::Affine3f CheckerCalibrator::calibrateInteractive( MultiSequence::ConstPtr &seq, 
                             size_t ref_idx, size_t target_idx ) const
  {
    pcl::visualization::CloudViewer viewer("Cloud"); 
    vector<vector<cv::Point2f> > corners_src(seq->size()), corners_target(seq->size());
    vector<vector<cv::Point3f> > object_pts(seq->size());
    vector<int> good_frames;
    Mat3b img_src, img_target;
    Mat1d camera_src, camera_target;
    getCameraMatrix(camera_src, 480, 640);
    getCameraMatrix(camera_target, 480, 640);
    vector<float> dist_src, dist_target;
    getDistCoeffs(dist_src);
    getDistCoeffs(dist_target);
    for( size_t i = 0; i < seq->size(); i++)
    {
      vector<Mat3b> imgs;
      seq->getImages( i, imgs );
      img_src = imgs[ref_idx];
      img_target = imgs[target_idx];
      if(!cv::findChessboardCorners(img_src, cv::Size(checker_cols_, checker_rows_), corners_src[i], 
                        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK ) )
        continue;
      if(!cv::findChessboardCorners(img_target, cv::Size(checker_cols_, checker_rows_), corners_target[i], 
                        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK ) )
        continue;
      
      // Show the detected boards
      Mat3b drawn_src, drawn_target;
      img_src.copyTo(drawn_src);
      img_target.copyTo(drawn_target);
      cv::drawChessboardCorners( drawn_src, cv::Size(checker_cols_, checker_rows_ ), 
          corners_src[i], true );
      cv::drawChessboardCorners( drawn_target, cv::Size(checker_cols_, checker_rows_ ), 
          corners_target[i], true );
      cv::circle( drawn_src, corners_src[i][0], 8, cv::Scalar(0,0,0), -1 );
      cv::circle( drawn_target, corners_target[i][0], 8, cv::Scalar(0,0,0), -1 );
      cv::imshow("img_src", drawn_src);
      cv::imshow("img_target", drawn_target);
      cout << "Hit y to use this image" << endl;
      char result = cv::waitKey(0);
      if (result == 'q'){
        cout << "Quitting!" << endl;
        break;
      }else if(result == 'y'){
        cout << "Adding frame " << i << endl;
        good_frames.push_back(i);
      }else{
        continue;
      }
      // Get subpixel accuracy
      cv::Mat1b img_src_gray, img_target_gray;
      cv::cvtColor(img_src, img_src_gray, CV_BGR2GRAY);
      cv::cvtColor(img_target, img_target_gray, CV_BGR2GRAY);
      TermCriteria crit(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 50, 0.01 );
      cv::cornerSubPix(img_src_gray, corners_src[i], cv::Size(5,5), cv::Size(-1,-1), 
          crit );
      cv::cornerSubPix(img_target_gray, corners_target[i], cv::Size(5,5), cv::Size(-1,-1), 
          crit );
      // Get object points
      getObjectPoints(object_pts[i]);
      //// Solve PnP for both
      //Mat rvec_src, tvec_src, rvec_target, tvec_target;
      //cv::solvePnP( object_pts[i], corners_src[i], camera_src, dist_src, rvec_src, tvec_src );
      //cv::solvePnP( object_pts[i], corners_target[i], camera_target, dist_target, rvec_target, tvec_target );
      ////Show clouds w/ transforms
      //cout << "T_src:" << tvec_src << endl;
      //cout << "R_src:" << rvec_src << endl;
      //cout << "T_target:" << tvec_target << endl;
      //cout << "R_target:" << rvec_target << endl;
      //Mat R_src, R_targ;
      //cv::Rodrigues(rvec_src, R_src);
      //cv::Rodrigues(rvec_target, R_targ);
      //Mat R = R_src * R_targ.inv();
      //Mat T = tvec_src - R*tvec_target;
      //Mat1f Tf = T;
      //Mat1f Rf = R;
      //Eigen::Matrix3f R_eig;
      //Eigen::Vector3f T_eig;
      //matrix_tricks::cv_to_eigen( Rf, R_eig );
      //matrix_tricks::cv_to_eigen( Tf, T_eig );
      //cout << "Rotation: \n" << R_eig << endl;
      //cout << "Translation: \n" << T_eig << endl;
      //Eigen::AngleAxisf aa; aa = R_eig;
      //Eigen::Translation3f t(Tf(0),Tf(1),Tf(2));
      //Eigen::Affine3f trans;
      //trans = aa*t;
      //vector<rgbd::Cloud::Ptr> pcds;
      //seq->getClouds(i, pcds);
      //rgbd::Point pt;
      //Mat T_inv = R_src.inv()*tvec_src;
      //pt.x = tvec_src.at<double>(0,0);
      //pt.y = tvec_src.at<double>(1,0);
      //pt.z = tvec_src.at<double>(2,0);
      //cout << "Pt.x: " << pt.x << ", Pt.y: " << pt.y << ", Pt.z: " << pt.z << endl;
      //pt.r = 255;
      //pt.g = pt.b = 0;
      //pcds[0]->points.push_back( pt );
      //cout << "pcds[0] size: " << pcds[i]->points.size() << endl;
      //viewer.showCloud(pcds[0]);
      //cv::waitKey(0);
      //return trans; 
    }
    vector<vector<cv::Point2f> > corners_src_good, corners_target_good;
    vector<vector<cv::Point3f> > object_pts_good;
    for(size_t i = 0; i < good_frames.size(); i++){
      int frame = good_frames[i];
      corners_src_good.push_back(corners_src[frame]);
      corners_target_good.push_back(corners_target[frame]);
      object_pts_good.push_back(object_pts[frame]);
    }
    cout << "Good frames has size: " << good_frames.size() << endl;
    //Estimate intrinsics
    Mat intrinsics_src = cv::initCameraMatrix2D( object_pts_good, corners_src_good, img_src.size() );
    Mat intrinsics_target = cv::initCameraMatrix2D( object_pts_good, corners_target_good, img_src.size() );
    cout << "Intrinsics_Src: " << intrinsics_src << endl;
    cout << "Instrinsics_Target: " << intrinsics_target << endl;
    cv::waitKey(0);
    // Do the calibration
    cv::Mat R,T,E,F;
    double rms = cv::stereoCalibrate( object_pts_good, corners_src_good, corners_target_good,
                         camera_src, dist_src, camera_target, dist_target, img_src.size(),
                         R, T, E, F, 
                         TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-7),
                         //CV_CALIB_FIX_ASPECT_RATIO +
                         //CV_CALIB_USE_INTRINSIC_GUESS +
                         CV_CALIB_FIX_INTRINSIC );
                         //CV_CALIB_SAME_FOCAL_LENGTH + 
                         //CV_CALIB_RATIONAL_MODEL + 
                         //CV_CALIB_FIX_FOCAL_LENGTH+
                         //CV_CALIB_ZERO_TANGENT_DIST);
                         //CV_CALIB_ZERO_TANGENT_DIST +
                         //CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5 );
    cout << "camera_src: " << camera_src << endl;
    cout << "camera_target: " << camera_target << endl;
                          
    cout << "Got rms error: " << rms << endl;
    // Convert from R, T to Eigen::Affine3f
    cout << "R: " << R << endl;
    cout << "T: " << T << endl;
    Eigen::Matrix4f trans_mat = Eigen::Matrix4f::Zero();
    for( int i = 0; i < 3; i++){
      trans_mat(i,3) = T.at<double>(i);
      for(int j = 0; j < 3; j++){
        trans_mat(i,j) = R.at<double>(i,j);
      }
    }
    trans_mat(3,3) = 1;
    //Eigen::Matrix3f R_eig;
    //Eigen::Vector3f T_eig;
    //matrix_tricks::cv_to_eigen( R, R_eig );
    //matrix_tricks::cv_to_eigen( T, T_eig );
    //cout << "Rotation: " << R_eig << endl;
    //cout << "Translation: " << T_eig << endl;
    //Eigen::AngleAxisf aa; aa = R_eig;
    //Eigen::Translation3f t(T(0),T(1),T(2));
    Eigen::Affine3f trans(trans_mat);
    //trans = aa*t;
    return trans; 
  }

  void CheckerCalibrator::getCameraMatrix( Mat1d &mat, int img_rows, int img_cols ) const
  {
    mat = Mat1d::zeros(3,3);
    mat(0,0) = focal_length_;
    mat(1,1) = focal_length_;
    mat(2,2) = 1;
    mat(0,2) = img_cols / 2.;
    mat(1,2) = img_rows / 2.;
  }
  void CheckerCalibrator::getDistCoeffs( vector<float> &dst ) const
  {
    dst = vector<float>(5,0);
  }
  void CheckerCalibrator::getObjectPoints( vector<cv::Point3f> &pts ) const
  {
    pts.resize( checker_rows_*checker_cols_ );
    int i = 0;
    for( int r = 0; r < checker_rows_; r++){
      for (int c = 0; c < checker_cols_; c++){
        pts[i] = cv::Point3f( c*square_size_, r*square_size_, 0 );
        i++;
      }
    }
  }

  bool CheckerCalibrator::estimateAffineFromFrame( MultiSequence::ConstPtr &seq, 
      size_t frame, size_t ref_idx, size_t target_idx, 
      Eigen::Affine3f &trans, vector<Point> &points_ref, vector<Point> &points_target,
      bool flip, bool interactive) const
  {
    vector<Mat3b> imgs;
    seq->getImages( frame, imgs );
    Mat3b& img_ref = imgs[ref_idx], img_target = imgs[target_idx];
    //Find the checkerboard -- assumes first point is above last, unless flip is set
    vector<cv::Point2f> corners_ref, corners_target;
    if(!cv::findChessboardCorners(img_ref, cv::Size(checker_cols_, checker_rows_), corners_ref, 
                      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK ) )
      return false;
    if(!cv::findChessboardCorners(img_target, cv::Size(checker_cols_, checker_rows_), corners_target, 
                      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK ) )
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
    bool found_any = false;
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
      found_any = true;
    }
    trans = estimateAffine(points_ref, points_target);
    return found_any;  
  }

  Eigen::Affine3f CheckerCalibrator::estimateAffine( const vector<Point> &points_ref, 
      const vector<Point> &points_target ) const
  {
    pcl::TransformationFromCorrespondences tc;
    for(size_t i = 0; i < points_ref.size(); i++)
    {
      tc.add(pointToEigen(points_ref[i]), pointToEigen(points_target[i]));
    }
    return tc.getTransformation();
  }
    
  Eigen::Vector3f pointToEigen( const Point &point )
  {
    return Eigen::Vector3f( point.x, point.y, point.z );
  }
}
