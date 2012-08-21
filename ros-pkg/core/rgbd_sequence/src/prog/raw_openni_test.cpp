#include <XnCppWrapper.h>
#include <math.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  XnStatus nRetVal = XN_STATUS_OK;
  xn::Context context;
  nRetVal = context.Init();
  xn::DepthGenerator dgen;
  nRetVal = dgen.Create(context);
  xn::ImageGenerator igen;
  nRetVal = igen.Create(context);
  
  XnMapOutputMode outputMode;
  outputMode.nXRes = 640;
  outputMode.nYRes = 480;
  outputMode.nFPS = 30;
  nRetVal = dgen.SetMapOutputMode(outputMode);
  nRetVal = igen.SetMapOutputMode(outputMode);
  nRetVal = context.StartGeneratingAll();

  XnDepthPixel maxdepth = dgen.GetDeviceMaxDepth();
  cout << "Max depth (mm): " << maxdepth << endl;
  cout << "Frame sync supported: " << dgen.IsCapabilitySupported("XN_CAPABILITY_FRAME_SYNC") << endl;
  XnUInt64 zpd;
  XnDouble zpps;
  XnDouble lddis;
  XnUInt64 shadow_value;
  XnUInt64 no_sample_value;
  XnFieldOfView fov;
  dgen.GetIntProperty("ZPD", zpd);
  dgen.GetRealProperty("ZPPS", zpps);
  dgen.GetRealProperty("LDDIS", lddis);
  dgen.GetIntProperty("ShadowValue", shadow_value);
  dgen.GetIntProperty("NoSampleValue", no_sample_value);
  dgen.GetFieldOfView(fov);
  cout << "== Depth Generator ==" << endl;
  cout << "zpd, focal length? (mm?): " << zpd << endl;
  cout << "zpps, pixel size at 640 x 480? (mm / pixel?): " << zpps * 2 << endl;
  cout << "Focal length in pixels at 640 x 480?: " << (double)zpd / (zpps * 2) << endl;
  cout << "lddis, baseline: " << lddis << endl;
  cout << "shadow_value: " << shadow_value << endl;
  cout << "no_sample_value: " << no_sample_value << endl;
  cout << "Vertical field of view (radians): " << fov.fVFOV << endl;
  cout << "Horizontal field of view (radians): " << fov.fHFOV << endl;
  cout << "Vertical field of view (degrees): " << fov.fVFOV * 180.0 / M_PI << endl;
  cout << "Horizontal field of view (degrees): " << fov.fHFOV * 180.0 / M_PI << endl;
  cout << "Focal length (x) in pixels using GetFieldOfView: " << outputMode.nXRes / (2.0 * tan(fov.fHFOV / 2.0)) << endl;
  cout << "Focal length (y) in pixels using GetFieldOfView: " << outputMode.nYRes / (2.0 * tan(fov.fVFOV / 2.0)) << endl;
  cout << endl;


  {
    XnUInt64 zpd;
    XnDouble zpps;
    XnDouble lddis;
    XnUInt64 shadow_value;
    XnUInt64 no_sample_value;
    XnFieldOfView fov;

    // These apparently don't work for the image generator.
    igen.GetIntProperty("ZPD", zpd);
    igen.GetRealProperty("ZPPS", zpps);
    igen.GetRealProperty("LDDIS", lddis);
    igen.GetIntProperty("ShadowValue", shadow_value);
    igen.GetIntProperty("NoSampleValue", no_sample_value);
    //igen.GetFieldOfView(fov);  // What?  This doesn't exist for cameras?  What the hell?
    cout << "== Image Generator ==" << endl;
    cout << "zpd, focal length? (mm?): " << zpd << endl;
    cout << "zpps, pixel size at 640 x 480? (mm / pixel?): " << zpps << endl;
    cout << "Focal length in pixels at 640 x 480?: " << (double)zpd / (zpps * 2) << endl;
    cout << "lddis, baseline: " << lddis << endl;
    cout << "shadow_value: " << shadow_value << endl;
    cout << "no_sample_value: " << no_sample_value << endl;
    cout << endl;
    // cout << "Vertical field of view (radians): " << fov.fVFOV << endl;
    // cout << "Horizontal field of view (radians): " << fov.fHFOV << endl;
    // cout << "Vertical field of view (degrees): " << fov.fVFOV * 180.0 / M_PI << endl;
    // cout << "Horizontal field of view (degrees): " << fov.fHFOV * 180.0 / M_PI << endl;
    // cout << "Focal length (x) in pixels using GetFieldOfView: " << outputMode.nXRes / (2.0 * tan(fov.fHFOV / 2.0)) << endl;
    // cout << "Focal length (y) in pixels using GetFieldOfView: " << outputMode.nYRes / (2.0 * tan(fov.fVFOV / 2.0)) << endl;
  }

  
  while(true) {
    nRetVal = context.WaitOneUpdateAll(dgen);
    if(nRetVal != XN_STATUS_OK) {
      printf("Failed updating data: %s\n", xnGetStatusString(nRetVal));
      continue;
    }

    xn::DepthMetaData md;
    dgen.GetMetaData(md);
    XnDepthPixel zres = md.ZRes();
    const XnDepthPixel* depthmap = md.Data();
    int width = md.GetUnderlying()->pMap->Res.X;
    int height = md.GetUnderlying()->pMap->Res.Y;
    cout << "Width: " << width << ", height: " << height;
    cout << ", zres: " << zres << ", depth value: " << depthmap[width * height / 2] << endl;
  }

  context.Shutdown();
  return 0;
}
