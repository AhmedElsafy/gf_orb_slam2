#include "SlamData.h"


#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include "../include/System.h"
#include "../include/MapPublisher.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/TransformStamped.h"

#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

//
//#include "path_smoothing_ros/cubic_spline_interpolator.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/publisher.h>

//#include <SlamData.h>
#include "ImageGrabber.h"


#include <mutex>

#include "../../../../include/Converter.h"
#include "../../../../include/MapPoint.h"
#include "../../../../include/FrameDrawer.h"
#include "../../../../include/MapDrawer.h"
#include "../../../../include/System.h"



int main(int argc, char **argv){

  ORB_SLAM2::System SLAM("/home/spiri/catkin_gf_orb_slam2/src/gf_orb_slam2/../ORB_Data/ORBvoc.bin" , "/home/spiri/catkin_gf_orb_slam2/src/gf_orb_slam2/Examples/ROS/GF_ORB_SLAM2/launch/Spiri_lowRes_Stereo.yaml" ,ORB_SLAM2::System::STEREO,false);

  SLAM.SetConstrPerFrame(1600);
  ros::init(argc, argv, "Stereo");
  ros::start();
  ros::NodeHandle nh;
  ORB_SLAM2::SlamData SLAMda(&SLAM, &nh, true);
  std::cout<<SLAMda.MavAlt<<std::endl;
  
  ORB_SLAM2::ImageGrabber igb(&SLAM, &SLAMda);
  igb.mpSLAMDATA->SetAlt(12.0);
  
  ros::Subscriber MavAltsub;
  MavAltsub = nh.subscribe("/mavros/altitude", 1000, &ORB_SLAM2::ImageGrabber::GrabAlt, &igb);
  
  while(ros::ok())
    ros::spin();
  
  
  
  return 0;




}
