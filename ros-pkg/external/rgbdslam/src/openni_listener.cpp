/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


//Documentation see header file
#include "pcl/ros/conversions.h"
#include <pcl/io/io.h>
#include "pcl/common/transform.h"
#include "pcl_ros/transforms.h"
#include "openni_listener.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <cv.h>
#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include "node.h"
#include "misc.h"
//#include <image_geometry/pinhole_camera_model.h>
#include "pcl/ros/for_each_type.h"

//For rosbag reading
#include <rosbag/view.h>
#include <boost/foreach.hpp>


#include "parameter_server.h"
//for comparison with ground truth from mocap and movable cameras on robots
#include <tf/transform_listener.h>

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;      




OpenNIListener::OpenNIListener(ros::NodeHandle nh, GraphManager* graph_mgr)
: graph_mgr_(graph_mgr),
  stereo_sync_(NULL), kinect_sync_(NULL), no_cloud_sync_(NULL),
  visua_sub_(NULL), depth_sub_(NULL), cloud_sub_(NULL),
  depth_mono8_img_(cv::Mat()),
  save_bag_file(false),
  pause_(ParameterServer::instance()->get<bool>("start_paused")),
  getOneFrame_(false),
  first_frame_(true),
  tflistener_(new tf::TransformListener(nh)),
  data_id_(0)
{
  ParameterServer* params = ParameterServer::instance();
  int q = params->get<int>("subscriber_queue_size");
  std::string bagfile_name = params->get<std::string>("bagfile_name");
  std::string visua_tpc = params->get<std::string>("topic_image_mono");
  std::string depth_tpc = params->get<std::string>("topic_image_depth");
  std::string cinfo_tpc = params->get<std::string>("camera_info_topic");

  if(bagfile_name.empty()){
    std::string cloud_tpc = params->get<std::string>("topic_points");
    std::string widev_tpc = params->get<std::string>("wide_topic");
    std::string widec_tpc = params->get<std::string>("wide_cloud_topic");

    //All information from Kinect
    if(!visua_tpc.empty() && !depth_tpc.empty() && !cloud_tpc.empty())
    {   
        visua_sub_ = new image_sub_type(nh, visua_tpc, q);
        depth_sub_ = new image_sub_type (nh, depth_tpc, q);
        cloud_sub_ = new pc_sub_type (nh, cloud_tpc, q);  
        kinect_sync_ = new message_filters::Synchronizer<KinectSyncPolicy>(KinectSyncPolicy(q),  *visua_sub_, *depth_sub_, *cloud_sub_),
        kinect_sync_->registerCallback(boost::bind(&OpenNIListener::kinectCallback, this, _1, _2, _3));
        ROS_INFO_STREAM("Listening to " << visua_tpc << ", " << depth_tpc << " and " << cloud_tpc);
    } 
    //No cloud, but visual image and depth
    else if(!visua_tpc.empty() && !depth_tpc.empty() && !cinfo_tpc.empty() && cloud_tpc.empty())
    {   
        visua_sub_ = new image_sub_type(nh, visua_tpc, q);
        depth_sub_ = new image_sub_type(nh, depth_tpc, q);
        cinfo_sub_ = new cinfo_sub_type(nh, cinfo_tpc, q);
        no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *visua_sub_, *depth_sub_, *cinfo_sub_);
        no_cloud_sync_->registerCallback(boost::bind(&OpenNIListener::noCloudCallback, this, _1, _2, _3));
        ROS_INFO_STREAM("Listening to " << visua_tpc << " and " << depth_tpc);
    } 

    //All information from stereo                                               
    if(!widec_tpc.empty() && !widec_tpc.empty())
    {   
      visua_sub_ = new image_sub_type(nh, widev_tpc, q);
      cloud_sub_ = new pc_sub_type(nh, widec_tpc, q);
      stereo_sync_ = new message_filters::Synchronizer<StereoSyncPolicy>(StereoSyncPolicy(q), *visua_sub_, *cloud_sub_);
      stereo_sync_->registerCallback(boost::bind(&OpenNIListener::stereoCallback, this, _1, _2));
      ROS_INFO_STREAM("Listening to " << widev_tpc << " and " << widec_tpc );
    } 

    detector_ = createDetector(params->get<std::string>("feature_detector_type"));
    extractor_ = createDescriptorExtractor(params->get<std::string>("feature_extractor_type"));

    if(params->get<bool>("concurrent_node_construction")){
      ROS_INFO("Threads used by QThreadPool on this Computer %i. Will increase this by one, b/c the QtRos Thread is very lightweight", QThread::idealThreadCount());
      QThreadPool::globalInstance()->setMaxThreadCount(QThread::idealThreadCount()*2+2);
    }

  } 
  else //Bagfile given
  {

    tf_pub_ = nh.advertise<tf::tfMessage>("/tf", 10);
    //All information from Kinect
    if(!visua_tpc.empty() && !depth_tpc.empty() && !cinfo_tpc.empty())
    {   
      // Set up fake subscribers to capture images
      depth_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
      rgb_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
      cam_info_sub_ = new BagSubscriber<sensor_msgs::CameraInfo>();
      no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *rgb_img_sub_, *depth_img_sub_, *cam_info_sub_);
      no_cloud_sync_->registerCallback(boost::bind(&OpenNIListener::noCloudCallback, this, _1, _2, _3));
      ROS_INFO_STREAM("Listening to " << visua_tpc << " and " << depth_tpc);
    } 


    detector_ = createDetector(params->get<std::string>("feature_detector_type"));
    extractor_ = createDescriptorExtractor(params->get<std::string>("feature_extractor_type"));
    QtConcurrent::run(this, &OpenNIListener::loadBag, bagfile_name);

    if(params->get<bool>("concurrent_node_construction")){
      ROS_INFO("Threads used by QThreadPool on this Computer %i. Will increase this by two, b/c the QtRos and loadBag threads are lightweight", QThread::idealThreadCount());
      QThreadPool::globalInstance()->setMaxThreadCount(QThread::idealThreadCount()+2);
    }
  }
}

 
//! Load data from bag file
/**This function reads the sensor input from a bagfile specified in the parameter bagfile_name.
 * It is meant for offline processing of each frame */
void OpenNIListener::loadBag(const std::string &filename)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  ROS_INFO("Loading Bagfile %s", filename.c_str());
  { //bag will be destructed after this block (hopefully frees memory for the optimizer)
    rosbag::Bag bag;
    try{
      bag.open(filename, rosbag::bagmode::Read);
    } catch(rosbag::BagIOException ex) {
      ROS_FATAL("Opening Bagfile %s failed: %s Quitting!", filename.c_str(), ex.what());
      ros::shutdown();
      return;
    }
    ROS_INFO("Opened Bagfile %s", filename.c_str());

    ParameterServer* params = ParameterServer::instance();
    std::string visua_tpc = params->get<std::string>("topic_image_mono");
    std::string depth_tpc = params->get<std::string>("topic_image_depth");
    std::string cinfo_tpc = params->get<std::string>("camera_info_topic");
    std::string tf_tpc = std::string("/tf");

    // Image topics to load for bagfiles
    std::vector<std::string> topics;
    topics.push_back(visua_tpc);
    topics.push_back(depth_tpc);
    topics.push_back(cinfo_tpc);
    topics.push_back(tf_tpc);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
   // int lc=0; 
    // Simulate sending of the messages in the bagfile
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
    //  if(lc++ > 1000) break;
      do{ 
        usleep(150);
        if(!ros::ok()) return;
      } while(pause_);

      if (m.getTopic() == visua_tpc || ("/" + m.getTopic() == visua_tpc))
      {
        sensor_msgs::Image::ConstPtr rgb_img = m.instantiate<sensor_msgs::Image>();
        if (rgb_img) rgb_img_sub_->newMessage(rgb_img);
        ROS_DEBUG("Found Message of %s", visua_tpc.c_str());
      }
      
      if (m.getTopic() == depth_tpc || ("/" + m.getTopic() == depth_tpc))
      {
        sensor_msgs::Image::ConstPtr depth_img = m.instantiate<sensor_msgs::Image>();
        if (depth_img) depth_img_sub_->newMessage(depth_img);
        ROS_DEBUG("Found Message of %s", depth_tpc.c_str());
      }
      if (m.getTopic() == cinfo_tpc || ("/" + m.getTopic() == cinfo_tpc))
      {
        sensor_msgs::CameraInfo::ConstPtr cam_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (cam_info) cam_info_sub_->newMessage(cam_info);
        ROS_DEBUG("Found Message of %s", cinfo_tpc.c_str());
      }
      if (m.getTopic() == tf_tpc|| ("/" + m.getTopic() == tf_tpc)){
        tf::tfMessage::ConstPtr tf_msg = m.instantiate<tf::tfMessage>();
        if (tf_msg) {
          //prevents missing callerid warning
          boost::shared_ptr<std::map<std::string, std::string> > msg_header_map = tf_msg->__connection_header;
          (*msg_header_map)["callerid"] = "rgbdslam";
          tf_pub_.publish(tf_msg);
          ROS_DEBUG("Found Message of %s", tf_tpc.c_str());
        }
      }
    }
    ROS_INFO("Finished processing of Bagfile");
    bag.close();
  }
  do{ 
    if(!future_.isFinished()){
      graph_mgr_->finishUp();
      future_.waitForFinished(); //Wait if GraphManager ist still computing. 
    }
    usleep(1000000); //give it a chance to receive further messages
    if(!ros::ok()) return;
    ROS_WARN("Waiting for processing to finish.");
  } while(graph_mgr_->isBusy());

  if(ParameterServer::instance()->get<bool>("batch_processing")){
    graph_mgr_->saveTrajectory(QString(filename.c_str()) + "before_optimization");
    graph_mgr_->optimizeGraph(20);
    ROS_INFO("Finished with 1st optimization");
    graph_mgr_->saveTrajectory(QString(filename.c_str()) + "after1_optimization");
    graph_mgr_->sanityCheck(100.0); //no trajectory is larger than a 100m
    graph_mgr_->pruneEdgesWithErrorAbove(1.5);
    graph_mgr_->optimizeGraph(20);
    ROS_INFO("Finished with 2nd optimization");
    graph_mgr_->saveTrajectory(QString(filename.c_str()) + "after2_optimization");
    if(ParameterServer::instance()->get<bool>("store_pointclouds")){
      graph_mgr_->saveIndividualClouds(QString(filename.c_str()), false);//not threaded
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    if(!ParameterServer::instance()->get<bool>("use_gui")){
      Q_EMIT bagFinished();
      ros::shutdown();
      usleep(10000000);//10sec to allow all threads to finish (don't know how much is required)
      if(ros::ok()){
        ROS_ERROR("Should have shutdown meanwhile");
        exit(1);
      }
    }
  }
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void OpenNIListener::stereoCallback(const sensor_msgs::ImageConstPtr& visual_img_msg, const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    ROS_INFO("Received data from stereo cam");
    ROS_INFO_ONCE("First RGBD-Data Received");
    if(++data_id_ % ParameterServer::instance()->get<int>("data_skip_step") != 0){ 
      ROS_INFO_THROTTLE(1, "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_);
      return;
    };

    //calculate depthMask
    cv::Mat_<uchar> depth_img(visual_img_msg->height, visual_img_msg->width);
    float value;
    unsigned int pt_ctr = 0;
    for(unsigned int y = 0; y < visual_img_msg->height; y++){
        for(unsigned int x = 0; x < visual_img_msg->width; x++){
            const uchar* value_ch_ptr = &(point_cloud->data[pt_ctr]);
            value = *((const float*)value_ch_ptr);
            pt_ctr += point_cloud->point_step;
            if(value != value){//isnan
                depth_img(y,x) = 0;
            }else{
                depth_img(y,x) = (char)(value*100.0); //Full white at 2.55 meter
            }
        }
    }

    //Get images into OpenCV format
    //sensor_msgs::CvBridge bridge;
    cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg, "mono8")->image;
    if(visual_img.rows != depth_img.rows ||
       visual_img.cols != depth_img.cols ||
       point_cloud->width != (uint32_t) visual_img.cols ||
       point_cloud->height != (uint32_t) visual_img.rows){
      ROS_ERROR("PointCloud, depth and visual image differ in size! Ignoring Data");
      return;
    }

    if (bagfile_mutex.tryLock() && save_bag_file){
       // todo: make the names dynamic
       bag.write("/wide_stereo/points2", ros::Time::now(), point_cloud);
       bag.write("/wide_stereo/left/image_mono", ros::Time::now(), visual_img_msg);
       ROS_INFO_STREAM("Wrote to bagfile " << bag.getFileName());
       bagfile_mutex.unlock();
       if(pause_) return;
    }
    pointcloud_type::Ptr pc_col(new pointcloud_type());//will belong to node
    if(ParameterServer::instance()->get<bool>("use_gui")){
      Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
      Q_EMIT newDepthImage (cvMat2QImage(depth_img,1));//overwrites last cvMat2QImage
    }
    pcl::fromROSMsg(*point_cloud,*pc_col);
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    cameraCallback(visual_img, pc_col, depth_img);
}

OpenNIListener::~OpenNIListener(){
  delete tflistener_;
}

void OpenNIListener::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                      const sensor_msgs::ImageConstPtr& depth_img_msg,
                                      const sensor_msgs::CameraInfoConstPtr& cam_info_msg) 
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  ROS_INFO_ONCE("First RGBD-Data Received");
  ROS_DEBUG("Received data from kinect");
  ParameterServer* ps = ParameterServer::instance();

  // If only a subset of frames are used, skip computations but visualize if gui is running
  if(++data_id_ % ps->get<int>("data_skip_step") != 0){ 
    ROS_INFO_THROTTLE(1, "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_);
    if(ps->get<bool>("use_gui")){//Show the image, even if not using it
      //sensor_msgs::CvBridge bridge;
      cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
      cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
      if(visual_img.rows != depth_float_img.rows || 
         visual_img.cols != depth_float_img.cols){
        ROS_ERROR("depth and visual image differ in size! Ignoring Data");
        return;
      }
      depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask
      Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
      Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
    }
    return;
  }


  //Convert images to OpenCV format
  //sensor_msgs::CvBridge bridge;
  //cv::Mat depth_float_img = bridge.imgMsgToCv(depth_img_msg);
  //cv::Mat visual_img =  bridge.imgMsgToCv(visual_img_msg);
  cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
  cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
  if(visual_img.rows != depth_float_img.rows || 
     visual_img.cols != depth_float_img.cols){
    ROS_ERROR("depth and visual image differ in size! Ignoring Data");
    return;
  }

  depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask

  if(asyncFrameDrop(depth_img_msg->header.stamp, visual_img_msg->header.stamp)) 
    return;

  if (bagfile_mutex.tryLock() && save_bag_file){
     // todo: make the names dynamic
     bag.write("/camera/rgb/image_mono", ros::Time::now(), visual_img_msg);
     bag.write("/camera/depth/image", ros::Time::now(), depth_img_msg);
     ROS_INFO_STREAM("Wrote to bagfile " << bag.getFileName());
     bagfile_mutex.unlock();
     if(pause_) return;
  }

  if(ps->get<bool>("use_gui")){
    Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
    Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
  }
  if(ps->get<bool>("store_pointclouds") || 
     (ps->get<bool>("use_glwidget") && ps->get<bool>("use_gui")) ||
     ps->get<bool>("use_icp")){
    pointcloud_type::Ptr pc_col(createXYZRGBPointCloud(depth_img_msg, visual_img_msg, cam_info_msg));
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    cameraCallback(visual_img, pc_col, depth_mono8_img_);

  }
  else {
    //Headless run without pointcloud storage
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    noCloudCameraCallback(visual_img, depth_float_img, depth_mono8_img_, depth_img_msg->header, cam_info_msg);
  }
}


void OpenNIListener::kinectCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,//got to be mono?
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,   
                                     const sensor_msgs::PointCloud2ConstPtr& point_cloud) 
{
  /// \callgraph
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  ROS_DEBUG("Received data from kinect");
  ROS_INFO_ONCE("First RGBD-Data Received");

  //Get images into OpenCV format
  //sensor_msgs::CvBridge bridge;
  //cv::Mat depth_float_img = bridge.imgMsgToCv(depth_img_msg);
  //cv::Mat visual_img =  bridge.imgMsgToCv(visual_img_msg);
  cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
  cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
  if(visual_img.rows != depth_float_img.rows || 
     visual_img.cols != depth_float_img.cols ||

     point_cloud->width != (uint32_t) visual_img.cols ||
     point_cloud->height != (uint32_t) visual_img.rows){
    ROS_ERROR("PointCloud, depth and visual image differ in size! Ignoring Data");
    return;
  }
  depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask

  if(asyncFrameDrop(depth_img_msg->header.stamp, visual_img_msg->header.stamp)) 
    return;

  if (bagfile_mutex.tryLock() && save_bag_file){
     // todo: make the names dynamic
     bag.write("/camera/rgb/points", ros::Time::now(), point_cloud);
     bag.write("/camera/rgb/image_mono", ros::Time::now(), visual_img_msg);
     bag.write("/camera/depth/image", ros::Time::now(), depth_img_msg);
     ROS_INFO_STREAM("Wrote to bagfile " << bag.getFileName());
     bagfile_mutex.unlock();
     if(pause_) return;
  }
  if(ParameterServer::instance()->get<bool>("use_gui")){
    Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
    Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
  }

  pointcloud_type::Ptr pc_col(new pointcloud_type());//will belong to node
  pcl::fromROSMsg(*point_cloud,*pc_col);
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  cameraCallback(visual_img, pc_col, depth_mono8_img_);
}




void OpenNIListener::cameraCallback(cv::Mat visual_img, 
                                    pointcloud_type::Ptr point_cloud, 
                                    cv::Mat depth_mono8_img)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  ROS_WARN_COND(point_cloud ==NULL, "Nullpointer for pointcloud");
  if(getOneFrame_) { getOneFrame_ = false; }//if getOneFrame_ is set, unset it and skip check for  pause
  else if(pause_ && !save_bag_file) { return; }//Visualization and nothing else

  cv::Mat gray_img; 
  if(visual_img.type() == CV_8UC3){ cvtColor(visual_img, gray_img, CV_RGB2GRAY); } 
  else { gray_img = visual_img; }

  //######### Main Work: create new node ##############################################################
  Q_EMIT setGUIStatus("Computing Keypoints and Features");
  Node* node_ptr = new Node(gray_img, detector_, extractor_, point_cloud, depth_mono8_img);

  retrieveTransformations(point_cloud->header, node_ptr);
  callProcessing(visual_img, node_ptr);
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}






void OpenNIListener::noCloudCameraCallback(cv::Mat visual_img, 
                                           cv::Mat depth, 
                                           cv::Mat depth_mono8_img,
                                           std_msgs::Header depth_header,
                                           const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  if(getOneFrame_) { //if getOneFrame_ is set, unset it and skip check for  pause
      getOneFrame_ = false;
  } else if(pause_ && !save_bag_file) { //Visualization and nothing else
    return; 
  }
  cv::Mat gray_img; 
  if(visual_img.type() == CV_8UC3){
    cvtColor(visual_img, gray_img, CV_RGB2GRAY);
  } else {
    gray_img = visual_img;
  }

  //######### Main Work: create new node ##############################################################
  Q_EMIT setGUIStatus("Computing Keypoints and Features");
  Node* node_ptr = new Node(gray_img, depth, depth_mono8_img, cam_info, depth_header, detector_, extractor_);

  retrieveTransformations(depth_header, node_ptr);//Retrieve the transform between the lens and the base-link at capturing time;

  callProcessing(visual_img, node_ptr);
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

//Call function either regularly or as background thread
void OpenNIListener::callProcessing(cv::Mat visual_img, Node* node_ptr)
{
  std::clock_t parallel_wait_time=std::clock();
  if(!future_.isFinished()){
    graph_mgr_->finishUp();
    future_.waitForFinished(); //Wait if GraphManager ist still computing. 
  }
  ROS_INFO_STREAM_NAMED("timings", "waiting time: "<< ( std::clock() - parallel_wait_time ) / (double)CLOCKS_PER_SEC  <<"sec"); 

  //update for visualization of the feature flow
  visualization_img_ = visual_img; //No copy
  visualization_depth_mono8_img_ = depth_mono8_img_;//No copy
  //With this define, processNode runs in the background and after finishing visualizes the results
  //Thus another Callback can be started in the meantime, to create a new node in parallel
  if(ParameterServer::instance()->get<bool>("concurrent_node_construction")) {
    ROS_DEBUG("Processing Node in parallel to the construction of the next node");
    if(ParameterServer::instance()->get<bool>("use_gui")){
      //visual_img points to the data received in the callback - which might be deallocated after the callback returns. 
      //This will happen before visualization if processNode is running in background, therefore the data needs to be cloned
      visualization_img_ = visual_img.clone();
      visualization_depth_mono8_img_ = depth_mono8_img_.clone();
    }
    future_ = QtConcurrent::run(this, &OpenNIListener::processNode, node_ptr); 
  }
  else { //Non-concurrent
    processNode(node_ptr);//regular function call
  }
}

void OpenNIListener::processNode(Node* new_node)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  Q_EMIT setGUIStatus("Adding Node to Graph");
  bool has_been_added = graph_mgr_->addNode(new_node);

  //######### Visualization code  #############################################
  if(ParameterServer::instance()->get<bool>("use_gui")){
    if(has_been_added){
      if(ParameterServer::instance()->get<bool>("visualize_mono_depth_overlay")){
        cv::Mat feature_img = cv::Mat( visualization_img_.rows, visualization_img_.cols, CV_8UC1); 
        graph_mgr_->drawFeatureFlow(feature_img);
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_,visualization_depth_mono8_img_, feature_img, 2)); //show registration
      } else {
        graph_mgr_->drawFeatureFlow(visualization_img_, cv::Scalar(255,0,0), cv::Scalar(0,128,0) );
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_, 2)); //show registration
      }
    } else {
      if(ParameterServer::instance()->get<bool>("visualize_mono_depth_overlay")){
        cv::Mat feature_img = cv::Mat( visualization_img_.rows, visualization_img_.cols, CV_8UC1); 
        cv::drawKeypoints(feature_img, new_node->feature_locations_2d_, feature_img, cv::Scalar(155), 5);
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_,visualization_depth_mono8_img_, feature_img, 2)); //show registration
      } else {
        cv::drawKeypoints(visualization_img_, new_node->feature_locations_2d_, visualization_img_, cv::Scalar(155, 0,0), 5);
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_, 2)); //show registration
      }
    }
  }
  if(!has_been_added) delete new_node;
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}


void OpenNIListener::toggleBagRecording(){
  bagfile_mutex.lock();
  save_bag_file = !save_bag_file;
  // save bag
  if (save_bag_file)
  {
    time_t rawtime; 
    struct tm * timeinfo;
    char buffer [80];

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    // create a nice name
    strftime (buffer,80,"kinect_%Y-%m-%d-%H-%M-%S.bag",timeinfo);

    bag.open(buffer, rosbag::bagmode::Write);
    ROS_INFO_STREAM("Opened bagfile " << bag.getFileName());
  } else {
    ROS_INFO_STREAM("Closing bagfile " << bag.getFileName());
    bag.close();
  }
  bagfile_mutex.unlock();
}

void OpenNIListener::togglePause(){
  pause_ = !pause_;
  ROS_INFO("Pause toggled to: %s", pause_? "true":"false");
  if(pause_) Q_EMIT setGUIStatus("Processing Thread Stopped");
  else Q_EMIT setGUIStatus("Processing Thread Running");
}
void OpenNIListener::getOneFrame(){
  getOneFrame_=true;
}
/// Create a QImage from image. The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
QImage OpenNIListener::cvMat2QImage(const cv::Mat& channel1, const cv::Mat& channel2, const cv::Mat& channel3, unsigned int idx){
  if(rgba_buffers_.size() <= idx){
      rgba_buffers_.resize(idx+1);
  }
  if(channel2.rows != channel1.rows || channel2.cols != channel1.cols ||
     channel3.rows != channel1.rows || channel3.cols != channel1.cols){
     ROS_ERROR("Image channels to be combined differ in size");
  }
  if(channel1.rows != rgba_buffers_[idx].rows || channel1.cols != rgba_buffers_[idx].cols){
    ROS_DEBUG("Created new rgba_buffer with index %i", idx);
    rgba_buffers_[idx] = cv::Mat( channel1.rows, channel1.cols, CV_8UC4); 
    //printMatrixInfo(rgba_buffers_[idx]);
  }
  cv::Mat mono_channel1 = channel1; 
  if(channel1.type() != CV_8UC1) {
    mono_channel1 = cv::Mat( channel1.rows, channel1.cols, CV_8UC1); 
    cv::cvtColor(channel1, mono_channel1, CV_RGB2GRAY);
  }
  std::vector<cv::Mat> input;
  cv::Mat alpha( channel1.rows, channel1.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  input.push_back(mono_channel1);
  input.push_back(channel2);
  input.push_back(channel3);
  input.push_back(alpha);
  cv::merge(input, rgba_buffers_[idx]);
  //printMatrixInfo(rgba_buffers_[idx]);
  return QImage((unsigned char *)(rgba_buffers_[idx].data), 
                rgba_buffers_[idx].cols, rgba_buffers_[idx].rows, 
                rgba_buffers_[idx].step, QImage::Format_RGB32 );
}

/// Create a QImage from image. The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
QImage OpenNIListener::cvMat2QImage(const cv::Mat& image, unsigned int idx){
  ROS_DEBUG_STREAM("Converting Matrix of type " << openCVCode2String(image.type()) << " to RGBA");
  if(rgba_buffers_.size() <= idx){
      rgba_buffers_.resize(idx+1);
  }
  ROS_DEBUG_STREAM("Target Matrix is of type " << openCVCode2String(rgba_buffers_[idx].type()));
  if(image.rows != rgba_buffers_[idx].rows || image.cols != rgba_buffers_[idx].cols){
    ROS_DEBUG("Created new rgba_buffer with index %i", idx);
    rgba_buffers_[idx] = cv::Mat( image.rows, image.cols, CV_8UC4); 
    //printMatrixInfo(rgba_buffers_[idx], "for QImage Buffering");
  }
  cv::Mat alpha( image.rows, image.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  cv::Mat in[] = { image, alpha };
  // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
  // rgba[2] -> bgr[0], rgba[3] -> alpha[0]

  if(image.type() == CV_8UC1){
    int from_to[] = { 0,0,  0,1,  0,2,  1,3 };
    mixChannels( in , 2, &rgba_buffers_[idx], 1, from_to, 4 );
  } else {
    int from_to[] = { 2,0,  1,1,  0,2,  3,3 }; //BGR+A -> RGBA
    mixChannels( in , 2, &rgba_buffers_[idx], 1, from_to, 4 );
  }
  //printMatrixInfo(rgba_buffers_[idx], "for QImage Buffering");
  return QImage((unsigned char *)(rgba_buffers_[idx].data), 
                rgba_buffers_[idx].cols, rgba_buffers_[idx].rows, 
                rgba_buffers_[idx].step, QImage::Format_RGB32 );
}


//Retrieve the transform between the lens and the base-link at capturing time
void OpenNIListener::retrieveTransformations(std_msgs::Header depth_header, Node* node_ptr)
{
  std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");
  std::string depth_frame_id = depth_header.frame_id;
  ros::Time depth_time = depth_header.stamp;
  tf::StampedTransform base2points;

  try{
    tflistener_->waitForTransform(base_frame, depth_frame_id, depth_time, ros::Duration(0.1));
    tflistener_->lookupTransform(base_frame, depth_frame_id, depth_time, base2points);
  }
  catch (tf::TransformException ex){
    ROS_WARN("%s",ex.what());
    ROS_WARN("Using Standard kinect /openni_camera -> /openni_rgb_optical_frame as transformation");
    //emulate the transformation from kinect openni_camera frame to openni_rgb_optical_frame
    base2points.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    base2points.setOrigin(tf::Point(0,-0.04,0));
  }
  node_ptr->setBase2PointsTransform(base2points);

  std::string gt_frame = ParameterServer::instance()->get<std::string>("ground_truth_frame_name");
  if(!gt_frame.empty()){ 
    //Retrieve the ground truth data. For the first frame it will be
    //set as origin. the rest will be used to compare
    tf::StampedTransform ground_truth_transform;
    try{
      tflistener_->waitForTransform(gt_frame, "/openni_camera", depth_time, ros::Duration(0.1));
      tflistener_->lookupTransform(gt_frame, "/openni_camera", depth_time, ground_truth_transform);
      tf::StampedTransform b2p;
      //HACK to comply with Jürgen Sturm's Ground truth, though I can't manage here to get the full transform from tf
      b2p.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
      b2p.setOrigin(tf::Point(0,-0.04,0));
      ground_truth_transform *= b2p;
    }
    catch (tf::TransformException ex){
      ROS_WARN("%s - Using Identity for Ground Truth",ex.what());
      ground_truth_transform = tf::StampedTransform(tf::Transform::getIdentity(), depth_time, "/missing_ground_truth", "/openni_camera");
    }
    node_ptr->setGroundTruthTransform(ground_truth_transform);
  }
  // End: Fill in Transformation -----------------------------------------------------------------------
}
