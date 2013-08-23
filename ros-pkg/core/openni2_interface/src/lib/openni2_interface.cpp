#include <openni2_interface/openni2_interface.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <signal.h>

using namespace std;
using namespace openni;

#define SAMPLE_READ_WAIT_TIMEOUT 2000  // ms
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

bool g_int = false;

void sigint(int none) {
  cout << "OpenNI2Interface caught user signal.  Shutting down..." << endl;
  g_int = true;
}

OpenNI2Interface::OpenNI2Interface() :
  sync_(0.1),
  terminating_(false)
{
  signal(SIGINT, sigint);
}

OpenNI2Interface::~OpenNI2Interface()
{
  cout << "Destroying OpenNI2Interface." << endl;
  depth_stream_.stop();
  depth_stream_.destroy();
  color_stream_.stop();
  color_stream_.destroy();
  device_.close();
  OpenNI::shutdown();
}

void OpenNI2Interface::run()
{
  ROS_ASSERT(handler_);
  int rv = connect();
  ROS_ASSERT(rv == 0);

  VideoStream* streams[] = { &color_stream_, &depth_stream_ };
  
  while(!terminating_ && !g_int) {
    // int changedStreamDummy;
    // VideoStream* pStream = &depth_stream_;
    // Status rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
    // if (rc != STATUS_OK)
    // {
    //   printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
    //   continue;
    // }

    int idx = -1;
    OpenNI::waitForAnyStream(streams, 2, &idx, SAMPLE_READ_WAIT_TIMEOUT);
    if(idx == 0)
      processColor();
    else if(idx == 1)
      processDepth();
    else
      cout << "Did not get stream data..." << endl;
  }

}

void OpenNI2Interface::processColor()
{
  openni::VideoFrameRef color_frame;
  Status rc = color_stream_.readFrame(&color_frame);
  ROS_ASSERT(rc == STATUS_OK);
  cout << "Got new color frame with ts: " << color_frame.getTimestamp() * 1e-6 << endl;
  sync_.addT0(color_frame, color_frame.getTimestamp() * 1e-6);
  processSynchronized();
}

void OpenNI2Interface::processDepth()
{
  openni::VideoFrameRef depth_frame;
  Status rc = depth_stream_.readFrame(&depth_frame);
  ROS_ASSERT(rc == STATUS_OK);
  cout << "Got new depth frame with ts: " << depth_frame.getTimestamp() * 1e-6 << endl;
  sync_.addT1(depth_frame, depth_frame.getTimestamp() * 1e-6);
  processSynchronized();
}

void OpenNI2Interface::processSynchronized()
{
  if(sync_.updated_) {
    cout << "New sync'd frame is available!  dt: " << sync_.ts0_ - sync_.ts1_ << endl;
    handler_->rgbdCallback(sync_.current0_, sync_.current1_);
    sync_.updated_ = false;
  }
}

int OpenNI2Interface::connect()
{
  Status rc = OpenNI::initialize();
  if(rc != STATUS_OK) {
    printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
    return 1;
  }

  rc = device_.open(ANY_DEVICE);
  if (rc != STATUS_OK)
  {
    printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
    return 2;
  }

  device_.setDepthColorSyncEnabled(true);

  if (device_.getSensorInfo(SENSOR_DEPTH) != NULL)
  {
    rc = depth_stream_.create(device_, SENSOR_DEPTH);
    if (rc != STATUS_OK)
    {
      printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
      return 3;
    }

    const Array<VideoMode>& dmodes = device_.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes();
    cout << "Depth modes: " << endl;
    for(int i = 0; i < dmodes.getSize(); ++i) {
      cout << "  " << dmodes[i].getResolutionX() << " " << dmodes[i].getResolutionY() << " " << dmodes[i].getPixelFormat() << " " << dmodes[i].getFps() << endl;
      if(dmodes[i].getResolutionX() == IMAGE_WIDTH && dmodes[i].getResolutionY() == IMAGE_HEIGHT &&
         dmodes[i].getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM && dmodes[i].getFps() == 30)
      {
        rc = depth_stream_.setVideoMode(dmodes[i]);
        cout << "Set." << endl;
        ROS_ASSERT(rc == STATUS_OK);
      }
    }

  }

  rc = color_stream_.create(device_, openni::SENSOR_COLOR);
  if (rc == openni::STATUS_OK)
  {
    
    const Array<VideoMode>& cmodes = device_.getSensorInfo(SENSOR_COLOR)->getSupportedVideoModes();
    cout << "Color modes: " << endl;
    for(int i = 0; i < cmodes.getSize(); ++i) {
      cout << "  " << cmodes[i].getResolutionX() << " " << cmodes[i].getResolutionY() << " " << cmodes[i].getPixelFormat() << " " << cmodes[i].getFps() << endl;
      if(cmodes[i].getResolutionX() == IMAGE_WIDTH && cmodes[i].getResolutionY() == IMAGE_HEIGHT &&
         cmodes[i].getPixelFormat() == openni::PIXEL_FORMAT_RGB888 && cmodes[i].getFps() == 30)
      {
        rc = color_stream_.setVideoMode(cmodes[i]);
        cout << "Set." << endl;
        ROS_ASSERT(rc == STATUS_OK);
      }
    }

    rc = color_stream_.start();
    if (rc != openni::STATUS_OK)
    {
      printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
      color_stream_.destroy();
    }
  }
  else
  {
    printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
  }

  rc = depth_stream_.start();
  if (rc != STATUS_OK)
  {
    printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
    return 4;
  }
  
  if (!depth_stream_.isValid() || !color_stream_.isValid())
  {
    printf("SimpleViewer: No valid streams. Exiting\n");
    openni::OpenNI::shutdown();
    return 2;
  }
 
  device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  
  VideoMode depth_video_mode = depth_stream_.getVideoMode();
  VideoMode color_video_mode = color_stream_.getVideoMode();
  
  int depth_width = depth_video_mode.getResolutionX();
  int depth_height = depth_video_mode.getResolutionY();
  int color_width = color_video_mode.getResolutionX();
  int color_height = color_video_mode.getResolutionY();

  ROS_ASSERT(depth_width == color_width);
  ROS_ASSERT(depth_height == color_height);

  ROS_DEBUG_STREAM("Connected to OpenNI RGBD sensor with "
                   << depth_width << " x " << depth_height << " resolution.");
  ROS_DEBUG_STREAM("Depth pixel format: " << depth_video_mode.getPixelFormat());
  ROS_DEBUG_STREAM("Color pixel format: " << color_video_mode.getPixelFormat());
  ROS_DEBUG_STREAM("Device registration mode: " << device_.getImageRegistrationMode());

// typedef enum
// {
// 	// Depth
// 	PIXEL_FORMAT_DEPTH_1_MM = 100,
// 	PIXEL_FORMAT_DEPTH_100_UM = 101,
// 	PIXEL_FORMAT_SHIFT_9_2 = 102,
// 	PIXEL_FORMAT_SHIFT_9_3 = 103,

// 	// Color
// 	PIXEL_FORMAT_RGB888 = 200,
// 	PIXEL_FORMAT_YUV422 = 201,
// 	PIXEL_FORMAT_GRAY8 = 202,
// 	PIXEL_FORMAT_GRAY16 = 203,
// 	PIXEL_FORMAT_JPEG = 204,
// 	PIXEL_FORMAT_YUYV = 205,
// } PixelFormat;

  return 0;
}

