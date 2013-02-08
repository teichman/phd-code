/*
 * =====================================================================================
 *
 *       Filename:  segment_labeler.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  02/19/2012 08:16:43 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@stanford.edu
 *        Company:  Stanford University
 *
 * =====================================================================================
 */

#include <cloud_calibration/user_calibrator.h>

namespace cloud_calibration {
  

  UserCalibratorController :: UserCalibratorController( const MultiSequence::ConstPtr &seq, 
      size_t ref_idx, size_t target_idx, OpenCVView* view):
    seq_( seq ), view_( view ), ref_idx_(ref_idx), target_idx_(target_idx),
    ref_click_pts_( seq->size() ), target_click_pts_( seq->size() ),
    ref_clicked_(false), target_clicked_(false),
    frame_( 0 ), quit_( false )
  {
    updateActiveFrame();
  }

  void UserCalibratorController::run()
  {
    while(!quit_) {
      Mat3b drawn_image;
      cur_image_.copyTo(drawn_image);
      for(size_t i = 0; i < ref_click_pts_[frame_].size(); i++){
        cv::circle( drawn_image, ref_click_pts_[frame_][i], 1, cv::Scalar(128,128,128), -1 );
        cv::circle( drawn_image, target_click_pts_[frame_][i]+cv::Point2f(640,0), 1, cv::Scalar(128,128,128), -1 );
      }
      if(ref_clicked_)
        cv::circle( drawn_image, ref_click_pt_, 1, cv::Scalar(0,0,255), -1 );
      if(target_clicked_)
        cv::circle( drawn_image, target_click_pt_+cv::Point2f(640,0), 1, cv::Scalar(0,0,255), -1 );

      view_->updateImage(drawn_image);
      char key = cvWaitKey(33);
      handleKeyPress(key);    
    }
  }
    
  void UserCalibratorController::mouseEvent( int event, int x, int y, int flags, void* param)
  {
    switch(event) {
      case CV_EVENT_LBUTTONUP:
        if(x <= 640){
          ref_click_pt_ = cv::Point2f(x,y);
          ref_clicked_ = true;
        } else
        {
          target_click_pt_ = cv::Point2f(x-640,y);
          target_clicked_ = true;
        }
        break;
    }
    if( ref_clicked_ && target_clicked_ ){
      rgbd::Point ref_pt, target_pt;
      if (get3DPoints( ref_click_pt_, target_click_pt_, ref_pt, target_pt) ){
        ref_pts_.push_back(ref_pt);
        target_pts_.push_back(target_pt);
        ref_click_pts_[frame_].push_back(ref_click_pt_);
        target_click_pts_[frame_].push_back(target_click_pt_);
      } else{
        cout << "Invalid depths!" << endl;
      }
      ref_clicked_ = target_clicked_ = false;
    }
  }

  void UserCalibratorController::handleKeyPress( char key )
  {
    switch(key) {
      case '=':
        if(frame_+1 < seq_->size()){
          frame_++;
          updateActiveFrame();
        }
        break;
      case '-':
        if(frame_ > 0){
          frame_--;
          updateActiveFrame();
        }
        break;
      case 'q':
        quit();
        break;
    }
  }

  void UserCalibratorController::updateActiveFrame()
  {
    cout << "On frame " << frame_ << endl;
    //Clear click points
    ref_clicked_ = target_clicked_ = false;
    //Update images
    cur_image_ = Mat3b(480,640*2);
    vector<Mat3b> images;
    seq_->getImages(frame_, images);
    Mat3b part1 = cur_image_.colRange(0,640);
    images[ref_idx_].copyTo(part1);
    Mat3b part2 = cur_image_.colRange(640,640*2);
    images[target_idx_].copyTo(part2);
  }


  bool UserCalibratorController::get3DPoints( const cv::Point2f &ref_click, const cv::Point2f &target_click, 
                  rgbd::Point &ref_pt, rgbd::Point &target_pt ) const
  {
    //Get clouds
    vector<Cloud::Ptr> clouds;
    seq_->getClouds(frame_, clouds);
    ref_pt = rgbd::Point(clouds[ref_idx_]->at(ref_click.x, ref_click.y));
    if(isnan(ref_pt.z))
      return false;
    target_pt = rgbd::Point(clouds[target_idx_]->at(target_click.x, target_click.y));
    if(isnan(target_pt.z))
      return false;
    return true;
  }

  void UserCalibratorController::quit()
  {
    quit_ = true;

    view_->updateImage(cv::Mat1b::zeros(1,1));
    cv::waitKey(20);
  }

/*  UserCalibrator */
  UserCalibrator::UserCalibrator( const string &name, const MultiSequence::ConstPtr &seq, size_t ref_idx, size_t target_idx ):
    view_(name, 1.5), controller_(seq, ref_idx, target_idx, &view_)
  {
    view_.setDelegate(&controller_);
  }
  UserCalibrator::~UserCalibrator()
  {
  }
  
  void UserCalibrator::get_points( vector<rgbd::Point> &points_ref, vector<rgbd::Point> &points_target)
  {
    controller_.run();
    points_ref = vector<rgbd::Point>(controller_.ref_pts_);
    points_target = vector<rgbd::Point>(controller_.target_pts_);
  }
}                /* -----  end of namespace cloud_calibration  ----- */

