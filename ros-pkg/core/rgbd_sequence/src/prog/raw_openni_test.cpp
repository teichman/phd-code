#include <XnCppWrapper.h>
#include <math.h>
#include <iostream>
#include <ros/assert.h>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
namespace bpo = boost::program_options;  

int main(int argc, char** argv)
{
  // -- Parse args.
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("register", "Register depth to rgb data")
    ("visualize", "Show depth")
    ;

  bpo::positional_options_description p;
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);

  // -- Set up production chain.
  XnStatus retval = XN_STATUS_OK;
  xn::Context context;
  retval = context.Init(); ROS_ASSERT(retval == XN_STATUS_OK);
  xn::DepthGenerator dgen;
  retval = dgen.Create(context); ROS_ASSERT(retval == XN_STATUS_OK);
  xn::ImageGenerator igen;
  retval = igen.Create(context); ROS_ASSERT(retval == XN_STATUS_OK);
  
  XnMapOutputMode output_mode;
  output_mode.nXRes = 640;
  output_mode.nYRes = 480;
  output_mode.nFPS = 30;
  retval = dgen.SetMapOutputMode(output_mode); ROS_ASSERT(retval == XN_STATUS_OK);
  retval = igen.SetMapOutputMode(output_mode); ROS_ASSERT(retval == XN_STATUS_OK);
  // Hardware depth registration.
  // https://groups.google.com/forum/?fromgroups=#!topic/openni-dev/5rP0mdPBeq0
  if(opts.count("register")) {
    cout << "Registering depth and rgb data." << endl;
    retval = dgen.SetIntProperty("RegistrationType", 1); ROS_ASSERT(retval == XN_STATUS_OK);  
    retval = dgen.GetAlternativeViewPointCap().SetViewPoint(igen); ROS_ASSERT(retval == XN_STATUS_OK);
  }
  else
    cout << "Leaving depth and rgb unregistered." << endl;
  
  // Synchronize output.
  retval = dgen.GetFrameSyncCap().FrameSyncWith(igen); ROS_ASSERT(retval == XN_STATUS_OK);
  // Start.
  retval = context.StartGeneratingAll(); ROS_ASSERT(retval == XN_STATUS_OK);

  cout << "Depth is synced with image: " << dgen.GetFrameSyncCap().IsFrameSyncedWith(igen) << endl;
  cout << "Image is synced with depth: " << igen.GetFrameSyncCap().IsFrameSyncedWith(dgen) << endl;

  // -- Read off params.
  XnDepthPixel maxdepth = dgen.GetDeviceMaxDepth();
  cout << "Max depth (mm): " << maxdepth << endl;
  cout << "Frame sync supported: " << dgen.IsCapabilitySupported("XN_CAPABILITY_FRAME_SYNC") << endl;
  cout << endl;
  
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
  cout << "Focal length (x) in pixels using GetFieldOfView: " << output_mode.nXRes / (2.0 * tan(fov.fHFOV / 2.0)) << endl;
  cout << "Focal length (y) in pixels using GetFieldOfView: " << output_mode.nYRes / (2.0 * tan(fov.fVFOV / 2.0)) << endl;
  cout << endl;


  cout << "== Image Generator ==" << endl;
  {
    XnUInt64 zpd;
    XnDouble zpps;
    XnDouble lddis;
    XnUInt64 shadow_value;
    XnUInt64 no_sample_value;
    //XnFieldOfView fov;

    // These apparently don't work for the image generator.
    igen.GetIntProperty("ZPD", zpd);
    igen.GetRealProperty("ZPPS", zpps);
    igen.GetRealProperty("LDDIS", lddis);
    igen.GetIntProperty("ShadowValue", shadow_value);
    igen.GetIntProperty("NoSampleValue", no_sample_value);
    //igen.GetFieldOfView(fov);  // What?  This doesn't exist for cameras?  What the hell?
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
    // cout << "Focal length (x) in pixels using GetFieldOfView: " << output_mode.nXRes / (2.0 * tan(fov.fHFOV / 2.0)) << endl;
    // cout << "Focal length (y) in pixels using GetFieldOfView: " << output_mode.nYRes / (2.0 * tan(fov.fVFOV / 2.0)) << endl;
  }

  if(!opts.count("visualize"))
    return 0;

  cv::Mat1f dimg(cv::Size(output_mode.nXRes, output_mode.nYRes), 0);
  while(true) {
    retval = context.WaitOneUpdateAll(dgen);
    if(retval != XN_STATUS_OK) {
      printf("Failed updating data: %s\n", xnGetStatusString(retval));
      continue;
    }

    xn::DepthMetaData md;
    dgen.GetMetaData(md);
    XnDepthPixel zres = md.ZRes();
    const XnDepthPixel* depthmap = md.Data();
    int width = md.GetUnderlying()->pMap->Res.X;
    int height = md.GetUnderlying()->pMap->Res.Y;
    cout << "Width: " << width << ", height: " << height;
    cout << ", zres: " << zres << ", depth value: " << md(width/2, height/2) << endl;

    for(int y = 0; y < dimg.rows; ++y) { 
      for(int x = 0; x < dimg.cols; ++x) {
	dimg(y, x) = md(x, y) / 3000.0;  // md is in meters.
	if(dimg(y, x) > 1.0)
	  dimg(y, x) = 0;
      }
    }

    cv::imshow("Depth Image", dimg);
    cv::waitKey(2);
  }

  context.Shutdown();
  return 0;
}
