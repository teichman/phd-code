/*
 * =====================================================================================
 *
 *       Filename:  segment_labeler.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  02/19/2012 08:07:44 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@stanford.edu
 *        Company:  Stanford University
 *
 * =====================================================================================
 */

#ifndef  ___CLOUD_CALIBRATION_USER_CALIBRATOR_H___
#define   ___CLOUD_CALIBRATION_USER_CALIBRATOR_H___

#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <multi_sequence/multi_sequence.h>
#include <pcl/common/transforms.h>
#include <image_labeler/opencv_view.h> //Gives us OpenCVViewDelegate

namespace cloud_calibration {
  using cv::Mat3b;
  using namespace std;
  using rgbd::Cloud;
  using multi_sequence::MultiSequence;
  class UserCalibratorController : public OpenCVViewDelegate{
    
    public:
      UserCalibratorController( const MultiSequence::ConstPtr &seq, size_t ref_idx, size_t target_idx, OpenCVView* view);
  
      void run();
      void mouseEvent( int event, int x, int y, int flags, void* param);

      MultiSequence::ConstPtr seq_;

      vector<rgbd::Point> ref_pts_, target_pts_;
      cv::Point2f ref_click_pt_, target_click_pt_;
      bool ref_clicked_, target_clicked_;
      vector< vector<cv::Point2f> > ref_click_pts_, target_click_pts_;
    private:
      bool get3DPoints( const cv::Point2f &ref_click, const cv::Point2f &target_click, 
                        rgbd::Point &ref_pt, rgbd::Point &target_pt ) const;

      OpenCVView* view_;
      size_t frame_;
      Mat3b cur_image_;
      size_t ref_idx_, target_idx_;
      bool quit_;

      void handleKeyPress( char key );
      void updateActiveFrame();
      void quit();
      enum MouseMode
      {
        MM_NONE,
        LDOWN,
        MDOWN,
        RDOWN
      };
      MouseMode mouse_mode_;

  };

  class UserCalibrator{
    public:
      UserCalibrator( const string &name, const MultiSequence::ConstPtr &seq, size_t ref_idx, size_t target_idx );
      ~UserCalibrator( );
  
      void get_points( vector<rgbd::Point> &points_ref, vector<rgbd::Point> &points_target);
    private:
      MultiSequence::ConstPtr seq_;
      OpenCVView view_;
      UserCalibratorController controller_;
  };
}		/* -----  end of namespace cloud_calibration  ----- */

#endif 

