/*
 * =====================================================================================
 *
 *       Filename:  test_transforms.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  09/05/2012 01:46:31 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <xpl_calibration/object_matching_calibrator.h>

using namespace std;

int main(int argc, char **argv)
{
  for(size_t i = 0; i < 100; i++)
  {
    double rx = (double)rand()/(double)RAND_MAX*M_PI/4; 
    double ry = (double)rand()/(double)RAND_MAX*M_PI/4; 
    double rz = (double)rand()/(double)RAND_MAX*M_PI/4; 
    double tx = (double)rand()/(double)RAND_MAX; 
    double ty = (double)rand()/(double)RAND_MAX; 
    double tz = (double)rand()/(double)RAND_MAX; 
    Eigen::Affine3f trans = generateTransform(rx,ry,rz,tx,ty,tz);
    double rx_new, ry_new, rz_new, tx_new, ty_new, tz_new;
    generateXYZYPR(trans, rx_new, ry_new, rz_new, tx_new, ty_new, tz_new);
    cout << "Old: " << rx << "," << ry << "," << rz << endl;
    cout << "New: " << rx_new << "," << ry_new << "," << rz_new << endl;
    assert(fabs(rx_new-rx) < 0.001);
    assert(fabs(ry_new-ry) < 0.001);
    assert(fabs(rz_new-rz) < 0.001);
    assert(fabs(tx_new-tx) < 0.001);
    assert(fabs(ty_new-ty) < 0.001);
    assert(fabs(tz_new-tz) < 0.001);
    cout << "tested i = " << i << endl;
  }
}
