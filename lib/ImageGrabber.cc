
# include "ImageGrabber.h"

namespace ORB_SLAM2{
	ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM, SlamData* pSLAMDATA) : mpSLAMDATA(pSLAMDATA),mpSLAM(pSLAM){
	    //mpSLAM = pSLAM;
	   //mpSLAMDATA = pSLAMDATA;
	#ifdef MAP_PUBLISH
	    //mnMapRefreshCounter = 0;
	#endif
	}
	
	ImageGrabber::~ImageGrabber(){
	  
	  std::cout<<"Image Grabber destroyed"<<std::endl;
	
	}
	
	ImageGrabber::ImageGrabber(){
	  
	  std::cout<<"Image Grabber default constractor"<<std::endl;
	
	}
	
	void ImageGrabber::initialize(ORB_SLAM2::System* pSLAM, SlamData* pSLAMDATA)
	{
	    mpSLAM = pSLAM;
	    mpSLAMDATA = pSLAMDATA;
	#ifdef MAP_PUBLISH
	    mnMapRefreshCounter = 0;
	#endif
	
	
	}



	void ImageGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msgImu)
	{
     std::cout<<"Inside GrabImu"<<std::endl;  
	   sensor_msgs::Imu ImuMsg = *msgImu;    
	   geometry_msgs::Quaternion imuQuat = ImuMsg.orientation;
	   std::cout<<"grabbed orientation"<<std::endl;
	   std::cout<<"imuQuat"<<imuQuat<<std::endl;
	   bool test = mpSLAMDATA->SetOrientationImu(imuQuat);
	   std::cout<<"test: "<<test<<std::endl;
	   std::cout<<"mpSLAMDATA->GetLastPose"<<std::endl;
	   //std::cout<<mpSLAMDATA->LastPose<<std::endl;  
     //mpSLAMDATA->ImuQuaternion = imuQuat;
	}


	void ImageGrabber::GrabAlt(const mavros_msgs::AltitudeConstPtr& msgAlt)
	{
	    std::cout<<"inside GrabAlt"<<std::endl;

	    mavros_msgs::Altitude MavAlt = *msgAlt;
	    float Alt = MavAlt.relative;
	    std::cout<<"Relative Altitude "<<Alt<<std::endl;
      try
      {
        mpSLAMDATA->SetAlt(Alt);
      }
      catch(runtime_error e) 
      {
        std::cout<< "Runtime error: " <<e.what()<<std::endl;
      }
	}


	void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
	{
      std::cout<<"Grabbing images"<<std::endl; 
      std::cout<<mpSLAM->mSensor<<std::endl; 

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
	    std::cout << "pose"<< std::endl;
	    std::cout << pose<<std::endl;

	    if (pose.empty())
	    {
		    mpSLAMDATA->SetResettingState(true);
		    mpSLAMDATA->SetResumeFromPose(mpSLAMDATA->GetLastPose());
		    mpSLAM->Reset();
		    std::cout<<"empty pose"<<std::endl;	
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
