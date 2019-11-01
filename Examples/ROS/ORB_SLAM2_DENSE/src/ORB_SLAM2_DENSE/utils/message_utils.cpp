//
// Created by yonghui on 19-10-30.
//

// ROS
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ORB_SLAM2_DENSE/utils/message_utils.h"

namespace ORB_SLAM2_DENSE
{
    MessageUtils::MessageUtils(ORB_SLAM2::System *pSystem) :
    private_nh_("~"), mpSystem_(pSystem)
    {
        // initalize publisher and subscriber
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);
        frame_pub_ = nh_.advertise<sensor_msgs::Image>("slam_frame", 10);
        pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>("cloud", 10);
        pcl2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud2", 10);

        // get parameters
        private_nh_.param("use_odom_pub", use_odom_pub_, true);
        private_nh_.param("use_tf", use_tf_, true);
        private_nh_.param("use_frame_pub", use_frame_pub_, true);
        private_nh_.param("use_pcl_pub", use_pcl_pub_, true);
        private_nh_.param("global_frame", global_frame_, string("odom"));
        private_nh_.param("odom_frame", base_frame_, string("base_link"));

        // get slam thread pointer
        mpTracker_ = mpSystem_->GetTracker();
        mpFrameDrawer_ = mpSystem_->GetFrameDrawer();
        mpMapDrawer_ = mpSystem_->GetMapDrawer();
        mpPclMapper_ = mpSystem_->GetPointCloudMapper();
    }


    void MessageUtils::PublishOdometry()
    {
        if (!use_odom_pub_)
            return;

        // camera pose
        cv::Mat Twc = mpTracker_->mCurrentFrame.mTcw.clone();
        if (mpTracker_->mState==ORB_SLAM2::Tracking::LOST)
        {
            ROS_WARN("ORB_SLAM2 has lost tracking.");
            return;
        }
        if (mpTracker_->mState!=ORB_SLAM2::Tracking::OK)
            return;
        if (cv::determinant(Twc) < 1e-4)
        {
            ROS_WARN("ORB_SLAM is tracking but the pose is ill state.");
            ROS_WARN_STREAM ("Abnormal pose\n" << Twc);
            return;
        }

        Twc = Twc.inv();

        // publish odom message
        nav_msgs::Odometry odom_msgs;
        odom_msgs.header.stamp = ros::Time::now();
        odom_msgs.header.frame_id = global_frame_;
        odom_msgs.pose.pose.position.x = Twc.at<float>(0,3);
        odom_msgs.pose.pose.position.y = Twc.at<float>(1,3);
        odom_msgs.pose.pose.position.z = Twc.at<float>(2,3);
        tf::Matrix3x3 Rwc
        (
                Twc.at<float>(0,0), Twc.at<float>(0,1), Twc.at<float>(0,2),
                Twc.at<float>(1,0), Twc.at<float>(1,1), Twc.at<float>(2,2),
                Twc.at<float>(2,0), Twc.at<float>(2,1), Twc.at<float>(2,2)
        );
        tf::Quaternion q;
        Rwc.getRotation(q);
        odom_msgs.pose.pose.orientation.x = q.x();
        odom_msgs.pose.pose.orientation.y = q.y();
        odom_msgs.pose.pose.orientation.z = q.z();
        odom_msgs.pose.pose.orientation.w = q.w();
        odom_pub_.publish(odom_msgs);

        // boardcast tf transform
        if (use_tf_)
        {
            tf::Transform trans(q, tf::Vector3(odom_msgs.pose.pose.position.x, odom_msgs.pose.pose.position.y, odom_msgs.pose.pose.position.z));
            tf::StampedTransform stamped_trans(trans, odom_msgs.header.stamp, global_frame_, base_frame_);
            transform_.sendTransform(stamped_trans);
        }
    }


    void MessageUtils::PublishFrame()
    {
        if (!use_frame_pub_)
            return;
        // draw current frame
        cv::Mat frame_mat = mpFrameDrawer_->DrawFrame();

        // publish
        std_msgs::Header h;
        h.stamp = ros::Time::now();
        h.frame_id = base_frame_;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage(h, sensor_msgs::image_encodings::BGR8, frame_mat));
        frame_pub_.publish(cv_ptr->toImageMsg());
    }


    void MessageUtils::PublishPointCloud()
    {
        if (!use_pcl_pub_ || !mpPclMapper_->isPointCloudMapUpdated())
            return;
    
        // sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 pcl2_msgs;
        // pcl map change, update it
        pcl_map_ = mpPclMapper_->GetPointCloudMap();
        pcl_map_.width = pcl_map_.size();  //! Occassionally it will conflict, force to equal
        ROS_WARN("Height: %d, Width: %d, Size: %d", pcl_map_.height, pcl_map_.width, pcl_map_.size());
        ROS_WARN("Point cloud update!");
        
        pcl::toROSMsg(pcl_map_, pcl2_msgs);
        ROS_WARN("PCL Message height: %d, width: %d, size: %d", pcl2_msgs.height, pcl2_msgs.width, pcl2_msgs.data.size());
        pcl2_msgs.header.stamp = ros::Time::now();
        pcl2_msgs.header.frame_id = global_frame_;
        pcl2_pub_.publish(pcl2_msgs);
        mpPclMapper_->setPointCloudMapUpdatedFlag(false);
        
        // sensor_msgs::PointCloud
        sensor_msgs::PointCloud pcl_msgs;
        sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msgs, pcl_msgs);
        pcl_msgs.header.stamp = ros::Time::now();
        pcl_msgs.header.frame_id = global_frame_;
        pcl_pub_.publish(pcl_msgs);
    }
}