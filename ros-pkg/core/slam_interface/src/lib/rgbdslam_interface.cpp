/*
 * =====================================================================================
 *
 *       Filename:  rgbdslam_interface.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  08/02/2012 02:55:06 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <slam_interface/rgbdslam_interface.h>
#include <fstream>
#include <sstream>
#include <iostream>
namespace slam_interface
{
  using std::ifstream;
  using std::cout;
  using std::endl;

  //! Must have a directory created by RgbdSlam
  RgbdSlamInterface::RgbdSlamInterface(const string &traj_file)
  {
    loadTransformTxtFile(traj_file);  
  }
  RgbdSlamInterface::~RgbdSlamInterface()
  {

  }
  //! For any frame, get the corresponding camera transform
  Eigen::Affine3f 
  RgbdSlamInterface::getTransform(size_t frame) const
  {
    //TODO interpolate if trajectory is missing. For now assume it got all
    ROS_ASSERT(frame == frames_[frame]);
    return transforms_[frame];
  }
    
  //! Get all transforms at once
  void
  RgbdSlamInterface::getTransforms(vector<Eigen::Affine3f> &transforms) const
  {
    transforms.resize(transforms_.size());
    for(size_t i = 0; i < transforms_.size(); i++)
    {
      transforms[i] = transforms_[i];
    }
  }
    
  //! See how many frames I have
  size_t
  RgbdSlamInterface::size() const
  {
    return frames_[frames_.size()-1]+1;
  }
    
  //! Loads transforms from a file
  void
  RgbdSlamInterface::loadTransformTxtFile(const string &txtfile)
  {
    ifstream f(txtfile.c_str());
    string line; getline(f, line); //First line is garbage;
    bool first = true;
    //First add identity for first frame
    transforms_.clear();
    frames_.clear();
    Eigen::Affine3f identity;
    identity = Eigen::Translation3f(0,0,0);
    transforms_.push_back(identity);
    frames_.push_back(0);
    while(true)
    {
      string line; getline(f, line);
      if(line.empty()) break;
      std::istringstream oss(line);
      float frame;
      float tx, ty, tz, q1, q2, q3, q4;
      if(first) cout << "Line: " << line << endl;
      oss >> frame;
      oss >> tx;
      oss >> ty;
      oss >> tz;
      oss >> q1;
      oss >> q2;
      oss >> q3;
      oss >> q4;
      Eigen::Affine3f transf;
      transf.matrix()(0,3) = tx;
      transf.matrix()(1,3) = ty;
      transf.matrix()(2,3) = tz;
      //Get rotation component by quaternion black magic
      Eigen::Vector4f q; q << q1,q2,q3,q4;
      q *= sqrt(2.0 / q.squaredNorm());
      cout << "q = " << q << endl;
      if(q.squaredNorm() < 1E-5)
      {
        transf = Eigen::Translation3f(0,0,0);
        continue;
      }
      Eigen::Matrix4f Q = q * q.transpose();
      cout << "Q = " << endl << Q << endl;
      transf.matrix()(0,0) = 1 - Q(1,1) - Q(2,2);
      transf.matrix()(0,1) = Q(0,1) - Q(2,3);
      transf.matrix()(0,2) = Q(0,2) + Q(1,3);
      transf.matrix()(1,0) = Q(0,1) + Q(2,3);
      transf.matrix()(1,1) = 1 - Q(0,0) - Q(2,2);
      transf.matrix()(1,2) = Q(1,2) - Q(0,3);
      transf.matrix()(2,0) = Q(0,2) - Q(1,3);
      transf.matrix()(2,1) = Q(1,2) + Q(0,3);
      transf.matrix()(2,2) = 1 - Q(0,0) - Q(1,1);
      transf.matrix()(3,0) = 0;
      transf.matrix()(3,1) = 0;
      transf.matrix()(3,2) = 0;
      transf.matrix()(3,3) = 1;
      //We index Right,Down,Forward; they index Forward,Left,Up
      Eigen::Matrix4f R;
      R <<
        0, -1,  0,  0,
        0,  0, -1,  0,
        1,  0,  0,  0,
        0,  0,  0,  1;
      transf.matrix() = R * transf.matrix() * R.inverse();
      transforms_.push_back(transf); 
      frames_.push_back(frame);
    }
    f.close();
  }
}
