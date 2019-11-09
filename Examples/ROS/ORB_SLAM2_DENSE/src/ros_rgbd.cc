/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "System.h"
#include "ORB_SLAM2_DENSE/utils/message_utils.h"
#include "ORB_SLAM2_DENSE/utils/tic_toc.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2_DENSE::MessageUtils *pMsgUtils);

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

private:

    ORB_SLAM2::System* mpSLAM;

    ORB_SLAM2_DENSE::MessageUtils *mpMsgUtils_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
//    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::Time::init();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    tf::TransformListener listener;
    
    // get parameters
    bool use_rviz;
    private_nh.param("use_rviz", use_rviz, true);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,!use_rviz);

    // publish necessary message of SLAM
    ORB_SLAM2_DENSE::MessageUtils msgUtils(listener, &SLAM);

    ImageGrabber igb(&SLAM, &msgUtils);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
    SLAM.save();

    ros::shutdown();

    return 0;
}


ImageGrabber::ImageGrabber(ORB_SLAM2::System *pSLAM, ORB_SLAM2_DENSE::MessageUtils *pMsgUtils) :
mpSLAM(pSLAM), mpMsgUtils_(pMsgUtils)
{

}


void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    ROS_DEBUG("---");
    
    ORB_SLAM2_DENSE::TicToc tic_toc;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    double cv_bridge_time = tic_toc.toc();

//    while(mpSLAM->mpPointCloudMapping->mbLoopBusy || mpSLAM->mpPointCloudMapping->cloudbusy)
//    {
//        cout<<"";
//    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    double tracking_time = tic_toc.toc();
    
    mpMsgUtils_->publishOdometry();
    mpMsgUtils_->publishFrame();
    mpMsgUtils_->publishPointCloud();

    double total_time = tic_toc.toc();
    
    ROS_DEBUG("TIME NOW: %f", ros::Time::now().toSec());
    ROS_DEBUG("CvBridge time cost: %f s[%f%%]", cv_bridge_time, (cv_bridge_time/total_time)*100);
    ROS_DEBUG("Tracking time cost: %f s[%f%%]", tracking_time-cv_bridge_time, ((tracking_time-cv_bridge_time)/total_time)*100);
    ROS_DEBUG("Publish time cost: %f s[%f%%]", total_time-tracking_time, ((total_time-tracking_time)/total_time)*100);
    ROS_DEBUG("Total time: %f s", total_time);
}


