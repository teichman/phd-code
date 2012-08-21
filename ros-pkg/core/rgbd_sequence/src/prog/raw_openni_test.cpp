#include <XnCppWrapper.h>
#include <math.h>
#include <iostream>
#include <ros/assert.h>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <timer/timer.h>
#include <pcl/io/openni_camera/openni_image_yuv_422.h>

using namespace std;
namespace bpo = boost::program_options;  


cv::Mat3b oniToCV(const openni_wrapper::Image& oni)
{
  cv::Mat3b img(oni.getHeight(), oni.getWidth());
  uchar data[img.rows * img.cols * 3];
  oni.fillRGB(img.cols, img.rows, data);
  int i = 0;
  for(int y = 0; y < img.rows; ++y) {
    for(int x = 0; x < img.cols; ++x, i+=3) {
      img(y, x)[0] = data[i+2];
      img(y, x)[1] = data[i+1];
      img(y, x)[2] = data[i];
    }
  }
    
  return img;
}

int main(int argc, char** argv)
{
  // -- Parse args.
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("register", "Register depth to rgb data")
    ("write", "Save data as pngs")
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
  retval = igen.SetIntProperty("InputFormat", 5);  // Uncompressed YUV?  PCL openni_device_primesense.cpp:62.
  retval = igen.SetPixelFormat(XN_PIXEL_FORMAT_YUV422); ROS_ASSERT(retval == XN_STATUS_OK);
  // Hardware depth registration.
  // https://groups.google.com/forum/?fromgroups=#!topic/openni-dev/5rP0mdPBeq0
  if(opts.count("register")) {
    cout << "Registering depth and rgb data." << endl;
    retval = dgen.SetIntProperty("RegistrationType", 1); ROS_ASSERT(retval == XN_STATUS_OK);  
    retval = dgen.GetAlternativeViewPointCap().SetViewPoint(igen); ROS_ASSERT(retval == XN_STATUS_OK);
    //retval = igen.GetAlternativeViewPointCap().SetViewPoint(dgen); ROS_ASSERT(retval == XN_STATUS_OK);  // This fails.
  }
  else
    cout << "Leaving depth and rgb unregistered." << endl;
  
  // Synchronize output.
  retval = dgen.GetFrameSyncCap().FrameSyncWith(igen); ROS_ASSERT(retval == XN_STATUS_OK);
  // Start.
  retval = context.StartGeneratingAll();
  if(retval != XN_STATUS_OK) {
    printf("OpenNI failed: %s\n", xnGetStatusString(retval));
  }
  ROS_ASSERT(retval == XN_STATUS_OK);

  // Recording test.
  // xn::Recorder recorder;
  // retval = recorder.Create(context); ROS_ASSERT(retval == XN_STATUS_OK);
  // retval = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, "recording.oni"); ROS_ASSERT(retval == XN_STATUS_OK);
  // //retval = recorder.AddNodeToRecording(dgen, XN_CODEC_16Z_EMB_TABLES); ROS_ASSERT(retval == XN_STATUS_OK);  // Fails.
  // retval = recorder.AddNodeToRecording(igen);
  
  XnDepthPixel maxdepth = dgen.GetDeviceMaxDepth();
  cout << "Max depth (mm): " << maxdepth << endl;
  cout << "Depth supports XN_CAPABILITY_FRAME_SYNC: " << dgen.IsCapabilitySupported(XN_CAPABILITY_FRAME_SYNC) << endl;
  cout << "Image supports XN_CAPABILITY_FRAME_SYNC: " << igen.IsCapabilitySupported(XN_CAPABILITY_FRAME_SYNC) << endl;
  cout << "Depth supports XN_CAPABILITY_ALTERNATIVE_VIEW_POINT: " << dgen.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT) << endl;
  cout << "Image supports XN_CAPABILITY_ALTERNATIVE_VIEW_POINT: " << igen.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT) << endl;
  cout << "Depth is synced with image: " << dgen.GetFrameSyncCap().IsFrameSyncedWith(igen) << endl;
  cout << "Image is synced with depth: " << igen.GetFrameSyncCap().IsFrameSyncedWith(dgen) << endl;
  cout << endl;
  
  // xn::DepthPrivateData* pDepthPrivate = (xn::DepthPrivateData*)dgen.pPrivateData;
  // XnDouble fXToZ = pDepthPrivate->GetRealWorldXtoZ();
  // XnDouble fYToZ = pDepthPrivate->GetRealWorldYtoZ();
  // cout << "fXToZ: " << fXToZ << endl;
  // cout << "fYToZ: " << fYToZ << endl;
    
  // -- Read off params.
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


  HighResTimer hrt;
  hrt.start();
  double iters = 0;
  double mean_fps = std::numeric_limits<double>::quiet_NaN();
  double prev_ts = 0;
  cv::Mat1b dimg(cv::Size(output_mode.nXRes, output_mode.nYRes), 0);
  //cv::Mat3b cimg(cv::Size(output_mode.nXRes, output_mode.nYRes), cv::Vec3b(0, 0, 0));
  while(true) {
    //retval = context.WaitOneUpdateAll(igen);
    //retval = context.WaitAndUpdateAll();  // FrameSync does not work properly with this....
    retval = context.WaitNoneUpdateAll();
    if(retval != XN_STATUS_OK) {
      printf("Failed updating data: %s\n", xnGetStatusString(retval));
      continue;
    }

    xn::DepthMetaData dmd;
    dgen.GetMetaData(dmd);
    boost::shared_ptr<xn::ImageMetaData> pimd(new xn::ImageMetaData);
    xn::ImageMetaData& imd = *pimd;
    igen.GetMetaData(imd);
    ROS_ASSERT(imd.PixelFormat() == XN_PIXEL_FORMAT_YUV422);
    
    XnDepthPixel zres = dmd.ZRes();
    int width = dmd.GetUnderlying()->pMap->Res.X;
    int height = dmd.GetUnderlying()->pMap->Res.Y;
    ROS_ASSERT(width == (int)imd.GetUnderlying()->pMap->Res.X);
    ROS_ASSERT(height == (int)imd.GetUnderlying()->pMap->Res.Y);
    double depth_ts = dmd.Timestamp() * 1e-6;
    double image_ts = imd.Timestamp() * 1e-6;
    if(depth_ts == prev_ts)
      continue;
    if(fabs(depth_ts - image_ts) > 0.003)
      continue;

    if(depth_ts - prev_ts > 0.04)
      ROS_WARN("Dropping frames!");
    
    cout << "Width: " << width << ", height: " << height;
    cout << ", zres: " << zres << ", depth_ts: " << depth_ts;
    cout << ", depth_ts - image_ts: " << depth_ts - image_ts;
    cout << ", ts - prev_ts: " << depth_ts - prev_ts;
    cout << ", mean fps: " << mean_fps << ", depth value: " << dmd(width/2, height/2) << endl;
    prev_ts = depth_ts;
    
    for(int y = 0; y < dimg.rows; ++y) { 
      for(int x = 0; x < dimg.cols; ++x) {
    	dimg(y, x) = 255.0 * dmd(x, y) / 5000.0;  // dmd is in millimeters.
    	if(dimg(y, x) > 255.0)
    	  dimg(y, x) = 0;
      }
    }
    cv::imshow("Depth Image", dimg);

    openni_wrapper::ImageYUV422 owimg(pimd);
    cv::Mat3b cimg = oniToCV(owimg);
    cv::imshow("Color Image", cimg);

    if(opts.count("write")) {
      {
	ostringstream oss;
	oss << "depth" << setw(5) << setfill('0') << iters << ".png";
	cv::imwrite(oss.str(), dimg);
      }
      {
	ostringstream oss;
	oss << "rgb" << setw(5) << setfill('0') << iters << ".png";
	cv::imwrite(oss.str(), cimg);
      }
    }
    
    char key = cv::waitKey(2);
    if(key == 'q')
      break;
    ++iters;
    mean_fps = iters / hrt.getSeconds();
  }

  context.Shutdown();
  return 0;
}
