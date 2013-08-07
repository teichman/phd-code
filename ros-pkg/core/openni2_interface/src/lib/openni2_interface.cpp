#include <openni2_interface/openni2_interface.h>
#include <ros/assert.h>
#include <ros/console.h>

using namespace std;
using namespace openni;

#define SAMPLE_READ_WAIT_TIMEOUT 2000  // ms

OpenNI2Interface::OpenNI2Interface() :
  sync_(0.1)
{
}

OpenNI2Interface::~OpenNI2Interface()
{
  depth_stream_.stop();
  depth_stream_.destroy();
  device_.close();
  OpenNI::shutdown();
}

void OpenNI2Interface::run()
{
  ROS_ASSERT(handler_);
  int rv = connect();
  ROS_ASSERT(rv == 0);

  VideoStream* streams[] = { &color_stream_, &depth_stream_ };
  
  while(true) {
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

  if (device_.getSensorInfo(SENSOR_DEPTH) != NULL)
  {
    rc = depth_stream_.create(device_, SENSOR_DEPTH);
    if (rc != STATUS_OK)
    {
      printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
      return 3;
    }
  }

  rc = depth_stream_.start();
  if (rc != STATUS_OK)
  {
    printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
    return 4;
  }

  rc = color_stream_.create(device_, openni::SENSOR_COLOR);
  if (rc == openni::STATUS_OK)
  {
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

  if (!depth_stream_.isValid() || !color_stream_.isValid())
  {
    printf("SimpleViewer: No valid streams. Exiting\n");
    openni::OpenNI::shutdown();
    return 2;
  }
  
  device_.setDepthColorSyncEnabled(true);

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
