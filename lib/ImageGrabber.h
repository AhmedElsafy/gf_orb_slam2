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

#include "SlamData.h"

using namespace std;

// #define MAP_PUBLISH

#define FRAME_WITH_INFO_PUBLISH

namespace ORB_SLAM2{
	class ImageGrabber
	{
	public:
	    ImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::SlamData* pSLAMDATA);

	    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
	    
	    //void GrabOdom(const nav_msgs::Odometry::ConstPtr& msg);

	    //void GrabPath(const nav_msgs::Path::ConstPtr& msg);

	    void GrabImu(const sensor_msgs::ImuConstPtr& msgImu);
	    
	    void GrabAlt(const mavros_msgs::AltitudeConstPtr& MavAlt); 

	    ORB_SLAM2::System* mpSLAM;
	    ORB_SLAM2::SlamData* mpSLAMDATA;
	    bool do_rectify;
	    cv::Mat M1l,M2l,M1r,M2r;
	    
	    double timeStamp;
	    cv::Mat Tmat;

	    ros::Publisher mpCameraPosePublisher, mpCameraPoseInIMUPublisher;
	    //    ros::Publisher mpDensePathPub;
	    
	#ifdef MAP_PUBLISH
	    size_t mnMapRefreshCounter;
	    ORB_SLAM2::MapPublisher* mpMapPub;
	#endif

	#ifdef FRAME_WITH_INFO_PUBLISH
	    ros::Publisher mpFrameWithInfoPublisher;
	#endif
	    
	};


}





