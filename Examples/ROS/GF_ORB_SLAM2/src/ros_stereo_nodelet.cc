#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "ImageGrabber.h"
#include "SlamData.h"
#include <image_transport/image_transport.h>



namespace ORB_SLAM2{
using namespace message_filters::sync_policies;


    class StereoNodelet : public nodelet::Nodelet
    {
        boost::shared_ptr<image_transport::ImageTransport> it_;
        
        bool do_viz;
        
        
        //image_transport::SubscriberFilter left_sub, right_sub;
        
        ros::Subscriber Imusub;
        ros::Subscriber MavAltsub;
        
        image_transport::SubscriberFilter                               sub_l_image_, sub_r_image_;
        typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
        typedef message_filters::Synchronizer<ApproximatePolicy>        ApproximateSync;
        boost::shared_ptr<ApproximateSync>                              approximate_sync_;
        
        //ORB_SLAM2::System NSLAMSYS;
        ORB_SLAM2::System* NSLAM;
        ORB_SLAM2::SlamData* NSLAMDATA;
        ImageGrabber* Nigb;
        ORB_SLAM2::System SLAMSYSTest;
        ORB_SLAM2::SlamData SLAMDATA;
        ImageGrabber igb; 

        //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        //message_filters::Synchronizer<sync_pol> sync(sync_pol(2), left_sub, right_sub);


        
        ros::Subscriber sub;
        
        virtual void onInit();


    };


    void StereoNodelet::onInit()
    {
    
      ROS_INFO("ORBSLAM Stereo nodelet initiated");
      ros::NodeHandle& nh         = getNodeHandle();
      ros::NodeHandle& private_nh = getPrivateNodeHandle();
      it_.reset(new image_transport::ImageTransport(nh));
      
      std::string left_topic;
      nh.param<std::string>("left_topic", left_topic, "/stereo/left/image_rect");
      nh.getParam("/left_topic",left_topic);
      //left_sub.subscribe(*it_, left_topic, 1);
      
      std::string right_topic;
      nh.param<std::string>("right_topic", right_topic, "/stereo/right/image_rect");
      nh.getParam("/right_topic",right_topic);
      //left_sub.subscribe(*it_, right_topic, 1);
      
      message_filters::Subscriber<sensor_msgs::Image> left_sub(nh,"/stereo/left/image_rect", 1);
      message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/stereo/right/image_rect", 1);

      std::string do_viz_str;
      nh.param<std::string>("do_viz", do_viz_str, "false");
      stringstream s2(do_viz_str); 
      s2 >> boolalpha >> do_viz;
      
      std::string vocabulary_path;
      nh.param<std::string>("vocabulary_path", vocabulary_path);
      nh.getParam("/vocabulary_path",vocabulary_path);
      std::cout<<"vocabulary_path"<<std::endl;
      std::cout<<vocabulary_path<<std::endl;
      
      std::string camera_setting_path;
      nh.param<std::string>("camera_setting_path", camera_setting_path);
      nh.getParam("/camera_setting_path",camera_setting_path);
      std::cout<<"camera_setting_path"<<std::endl;
      std::cout<<camera_setting_path<<std::endl;
      
      int budget_per_frame;
      nh.param("budget_per_frame", budget_per_frame, 1600);
      
      std::string path_to_map;
      nh.param<std::string>("path_to_map", path_to_map);
      
      
      //ORB_SLAM2::System SLAM(vocabulary_path , camera_setting_path ,ORB_SLAM2::System::STEREO,do_viz);
      //SLAM.SetConstrPerFrame(budget_per_frame);
      
      SLAMSYSTest.initialize(vocabulary_path , camera_setting_path ,ORB_SLAM2::System::STEREO,do_viz);
      SLAMSYSTest.SetConstrPerFrame(budget_per_frame);
      //NSLAMSYS = SLAM;
      
      NSLAM = &SLAMSYSTest;

            
      SLAMDATA.initialize(&SLAMSYSTest, &nh, true);
      
      NSLAMDATA = &SLAMDATA;


      std::cout<<"creating igb"<<std::endl;
      igb.initialize(NSLAM, NSLAMDATA);
      igb.mpFrameWithInfoPublisher = nh.advertise<sensor_msgs::Image>("ORB_SLAM/frame_with_info", 100);
      
      Nigb = &igb;
      std::cout<<"igb created"<<std::endl;

      Imusub = nh.subscribe("mavros/imu/data", 1000, &ImageGrabber::GrabImu, &igb);
      MavAltsub = nh.subscribe("/mavros/altitude", 1000, &ImageGrabber::GrabAlt, &igb);

      
      
      image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
      sub_l_image_.subscribe(*it_, "/stereo/left/image_rect", 1, hints);
      sub_r_image_.subscribe(*it_, "/stereo/right/image_rect", 1, hints);


      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(2), sub_l_image_, sub_r_image_));
      approximate_sync_->registerCallback(boost::bind(&ImageGrabber::GrabStereo, Nigb, _1, _2));
      
      //sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

      

      std::cout<<"mpFrameWithInfoPublisher created"<<std::endl;  
      
      


      
      
    }



    /*
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
    double latency_trans = ros::Time::now().toSec() - msgLeft->header.stamp.toSec();
    // ROS_INFO("ORB-SLAM Initial Latency: %.03f sec", ros::Time::now().toSec() - msgLeft->header.stamp.toSec());
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
    // ROS_INFO("ORB-SLAM Image Transmision Latency: %.03f sec", ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec());
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
        // ROS_INFO("ORB-SLAM Tracking Latency: %.03f sec", ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec());
        // ROS_INFO("Image Transmision Latency: %.03f sec; Total Tracking Latency: %.03f sec", latency_trans, latency_total);
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
    }*/

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ORB_SLAM2::StereoNodelet, nodelet::Nodelet)
