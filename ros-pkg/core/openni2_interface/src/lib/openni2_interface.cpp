#include <openni2_interface/openni2_interface.h>
#include <ros/assert.h>
#include <ros/console.h>

using namespace std;
using namespace openni;

#define SAMPLE_READ_WAIT_TIMEOUT 2000  // ms

OpenNI2Interface::OpenNI2Interface()
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
    int changedStreamDummy;
    VideoStream* pStream = &depth_stream_;
    Status rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
    if (rc != STATUS_OK)
    {
      printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
      continue;
    }

    int idx;
    OpenNI::waitForAnyStream(streams, 2, &idx);
    if(idx == 0)
      processColor();
    else
      processDepth();
    
    //handler_->rgbdCallback(color_frame_, depth_frame_);
  }

}

void OpenNI2Interface::processColor()
{
  cout << "processColor" << endl;
  Status rc = color_stream_.readFrame(&color_frame_);
  ROS_ASSERT(rc == STATUS_OK);
}

void OpenNI2Interface::processDepth()
{
  cout << "processDepth" << endl;
  Status rc = depth_stream_.readFrame(&depth_frame_);
  ROS_ASSERT(rc == STATUS_OK);
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
  
  return 0;
}

