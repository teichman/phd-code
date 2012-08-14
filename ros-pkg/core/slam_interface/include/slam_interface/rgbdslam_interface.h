/*
 * =====================================================================================
 *
 *       Filename:  rgbdslam_interface.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  08/02/2012 01:46:53 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#ifndef __SFM3D_RGBDSLAM_INTERFACE_H__
#define __SFM3D_RGBDSLAM_INTERFACE_H__

#include <slam_interface/types.h>

namespace slam_interface
{
  class RgbdSlamInterface
  {
  public:
    typedef boost::shared_ptr<RgbdSlamInterface> Ptr;
    typedef boost::shared_ptr<const RgbdSlamInterface> ConstPtr;
    //! Reads a trajectory_estimate file, outputs Eigen::Affine3fs
    RgbdSlamInterface(const string &traj_file);
    ~RgbdSlamInterface();
    //! For any frame, get the corresponding camera transform
    Eigen::Affine3f 
    getTransform(size_t frame) const;
    //! Get all transforms at once
    void
    getTransforms(vector<Eigen::Affine3f> &transforms) const;
    //! See how many frames I have
    size_t
    size() const;
  protected:
    //! Loads transforms from a file
    void
    loadTransformTxtFile(const string &txtfile);

    vector<int> frames_;
    vector<Eigen::Affine3f> transforms_;

  };
}

#endif//__SFM3D_RGBDSLAM_INTERFACE_H__
