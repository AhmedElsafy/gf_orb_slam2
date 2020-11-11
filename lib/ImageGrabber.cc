
# include "ImageGrabber.h"

namespace ORB_SLAM2{
	ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::SlamData* pSLAMDATA)
	{
	    mpSLAM = pSLAM;
	    mpSLAMDATA = pSLAMDATA;
	#ifdef MAP_PUBLISH
	    mnMapRefreshCounter = 0;
	#endif
	}

	void ImageGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msgImu)
	{
	   sensor_msgs::Imu ImuMsg = *msgImu;    
	   geometry_msgs::Quaternion imuQuat = ImuMsg.orientation;
	   mpSLAMDATA->SetOrientationImu(imuQuat);

	}


	void ImageGrabber::GrabAlt(const mavros_msgs::AltitudeConstPtr& msgAlt)
	{
	    mavros_msgs::Altitude MavAlt = *msgAlt;
	    float Alt = MavAlt.relative;
	    mpSLAMDATA->SetAlt(Alt);
	}


	void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
	{
        std::cout<<"Grabbing images"<<std::endl;  

	    double latency_trans = ros::Time::now().toSec() - msgLeft->header.stamp.toSec();
	    ROS_INFO("ORB-SLAM Initial Latency: %.03f sec", ros::Time::now().toSec() - msgLeft->header.stamp.toSec());

	    // Copy the ros image message to cv::Mat.
	    cv_bridge::CvImageConstPtr cv_ptrLeft;
	    try
	    {
		    cv_ptrLeft = cv_bridge::toCvShare(msgLeft, "mono8");
	    }
	    catch (cv_bridge::Exception& e)
	    {
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return;
	    }

	    cv_bridge::CvImageConstPtr cv_ptrRight;
	    try
	    {
		    cv_ptrRight = cv_bridge::toCvShare(msgRight, "mono8");
	    }
	    catch (cv_bridge::Exception& e)
	    {
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return;
	    }

	    ROS_INFO("ORB-SLAM Image Transmision Latency: %.03f sec", ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec());

	    cv::Mat pose;
	 
		pose = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
	    

	    if (pose.empty())
	    {
		    mpSLAMDATA->SetResettingState(true);
		    mpSLAMDATA->SetResumeFromPose(mpSLAMDATA->GetLastPose());
		    mpSLAM->Reset();	
		    return;
	    }
	    
	    mpSLAMDATA->SetLastpose(pose);

	    double latency_total = ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec();
	    ROS_INFO("ORB-SLAM Tracking Latency: %.03f sec", ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec());
	    ROS_INFO("Image Transmision Latency: %.03f sec; Total Tracking Latency: %.03f sec", latency_trans, latency_total);
	    ROS_INFO("Pose Tracking Latency: %.03f sec", latency_total - latency_trans);

	    mpSLAMDATA->update(pose);

	    
	#ifdef FRAME_WITH_INFO_PUBLISH
	    if (mpSLAM != NULL && mpSLAM->mpFrameDrawer != NULL) {
		cv::Mat fr_info_cv = mpSLAM->mpFrameDrawer->DrawFrame();
		cv_bridge::CvImage out_msg;
		out_msg.header   = cv_ptrLeft->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
		out_msg.image    = fr_info_cv; // Your cv::Mat
		mpFrameWithInfoPublisher.publish(out_msg.toImageMsg());
	    }
	#endif


	#ifdef MAP_PUBLISH
	    if (mnMapRefreshCounter % 30 == 1) {
		  // publish map points
		  mpMapPub->Refresh();
	    }
	    mnMapRefreshCounter ++;
	#endif

	}
}
