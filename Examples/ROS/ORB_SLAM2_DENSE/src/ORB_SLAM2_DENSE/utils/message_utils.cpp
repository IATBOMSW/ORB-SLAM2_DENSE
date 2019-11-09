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
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "ORB_SLAM2_DENSE/utils/message_utils.h"

namespace ORB_SLAM2_DENSE
{
    MessageUtils::MessageUtils(tf::TransformListener &listener, ORB_SLAM2::System *pSystem) :
    private_nh_("~"), listener_(listener), mpSystem_(pSystem), pcl_map_(new PointCloudMapping::PointCloud()),
    pcl_plane_(new PointCloudMapping::PointCloud())
    {
        // initial plane coefficients ( xy plane)
        plane_coeffs_ << 0.0, 0.0, 1.0, 0.0;
        last_plane_coeffs_ << 0.0, 0.0, 1.0, 0.0;
        
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
//        private_nh_.param("use_plane_segment", use_plane_segment_, true);
//        private_nh_.param("segment_min_z", min_z_, -0.5);
//        private_nh_.param("segment_max_z", max_z_,  0.5);
//        private_nh_.param("plane_dist_thres", plane_dist_thres_, 0.2);
        //get frame id
        private_nh_.param("map_frame", map_frame_, std::string("map"));
        private_nh_.param("odom_frame", odom_frame_, std::string("odom"));
        private_nh_.param("footprint_frame", footprint_frame_, std::string("camera_footprint"));
        private_nh_.param("optical_frame", optical_frame_, std::string("camera_optical"));

        // get slam thread pointer
        mpTracker_ = mpSystem_->GetTracker();
        mpFrameDrawer_ = mpSystem_->GetFrameDrawer();
        mpMapDrawer_ = mpSystem_->GetMapDrawer();
        mpPclMapper_ = mpSystem_->GetPointCloudMapper();
    }


    void MessageUtils::publishOdometry()
    {
        if (!use_odom_pub_)
            return;
        
        // odom_frame<--optical_frame, this is estimated by ORB-SLAM2
        cv::Mat matTcw = mpTracker_->mCurrentFrame.mTcw.clone();
        if (mpTracker_->mState==ORB_SLAM2::Tracking::LOST)
        {
            ROS_WARN("ORB_SLAM2 has lost tracking.");
            return;
        }
        if (mpTracker_->mState!=ORB_SLAM2::Tracking::OK)
            return;
        if (cv::determinant(matTcw) < 1e-4)
        {
            ROS_WARN("ORB_SLAM is tracking but the pose is ill state.");
            ROS_WARN_STREAM ("Abnormal pose\n" << matTcw);
            return;
        }
        // convert to tf class
        tf::Matrix3x3 Rcw
        (
                matTcw.at<float>(0,0), matTcw.at<float>(0,1), matTcw.at<float>(0,2),
                matTcw.at<float>(1,0), matTcw.at<float>(1,1), matTcw.at<float>(2,2),
                matTcw.at<float>(2,0), matTcw.at<float>(2,1), matTcw.at<float>(2,2)
        );
        tf::Vector3 tcw(matTcw.at<float>(0,3), matTcw.at<float>(1,3), matTcw.at<float>(2,3));
        tf::Stamped<tf::Transform> Tcw(tf::Transform(Rcw, tcw), ros::Time::now(), optical_frame_);
        
        // footprint_frame<--optical_frame, static transform
        tf::Stamped<tf::Transform> Tbc(tf::Transform(), ros::Time::now(), optical_frame_);
        Tbc.setIdentity();
        if ( !getTransformedPose(Tbc, footprint_frame_) )
            return;
        
        // visual odom transform to footprint odom
        tf::Stamped<tf::Transform> Twb(Tbc*Tcw.inverse()*Tbc.inverse(), ros::Time::now(), odom_frame_);
        Twb.setRotation(Twb.getRotation().normalized());  //! necessary, otherwise Rviz will complain unnormalized

        // publish odom message
        nav_msgs::Odometry odom_msgs;
        odom_msgs.header.stamp = ros::Time::now();
        odom_msgs.header.frame_id = odom_frame_;
        odom_msgs.child_frame_id = footprint_frame_;
        tf::poseTFToMsg(Twb, odom_msgs.pose.pose);
        odom_pub_.publish(odom_msgs);

        // boardcast tf transform
        if (use_tf_)
        {
            // odom<--footprint
            tf::StampedTransform stamped_trans(Twb, odom_msgs.header.stamp, odom_frame_, footprint_frame_);
            broadcaster_.sendTransform(stamped_trans);
            
            // map<--odom
            Eigen::Matrix4d mTmo;
//            ROS_WARN_STREAM("Input plane coeffinicent: " << plane_coeffs_.transpose());
            mpPclMapper_->getPlaneTransformMatrix(Eigen::Vector4d(0,0,1,0), plane_coeffs_, mTmo);
//            ROS_WARN_STREAM("map<--odom transform:\n" << mTmo);
            Eigen::Matrix3d mRmo = mTmo.block(0,0,3,3);
            double tz = mTmo(2,3);
            tf::Matrix3x3 Rmo;
            tf::matrixEigenToTF(mRmo, Rmo);
            tf::StampedTransform trans(tf::Transform(Rmo, tf::Vector3(0, 0, tz)), ros::Time::now(), map_frame_, odom_frame_);
            broadcaster_.sendTransform(trans);
        }
        
        // update SLAM PointCloudMapping thread extrinsic matrix
        Eigen::Matrix3d matRbc;
        Eigen::Vector3d mattbc;
        tf::matrixTFToEigen(Tbc.getBasis(), matRbc);
        tf::vectorTFToEigen(Tbc.getOrigin(), mattbc);
        mpPclMapper_->updateTbc(matRbc, mattbc);
        
        // debug log
        Eigen::Isometry3d ematTwb;
        tf::poseTFToEigen(Twb, ematTwb);
        ROS_DEBUG_STREAM("SLAM output Twc: \n" << matTcw.inv());
        ROS_DEBUG_STREAM("Footprint output Twb: \n" << ematTwb.matrix());
    }


    void MessageUtils::publishFrame()
    {
        if (!use_frame_pub_)
            return;
        // draw current frame
        cv::Mat frame_mat = mpFrameDrawer_->DrawFrame();

        // publish
        std_msgs::Header h;
        h.stamp = ros::Time::now();
        h.frame_id = optical_frame_;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage(h, sensor_msgs::image_encodings::BGR8, frame_mat));
        frame_pub_.publish(cv_ptr->toImageMsg());
    }
    
    
    void MessageUtils::publishPointCloud()
    {
        if (!use_pcl_pub_ || !mpPclMapper_->getPointCloudMapUpdatedFlag())
            return;

        // get Tbc, Matrix4d
        Eigen::Matrix4d Tbc = Eigen::Matrix4d::Identity();
        if (!getTransformedPose(Tbc, footprint_frame_, optical_frame_))
            return;

        // update point cloud
        if (mpPclMapper_->getUsePlanSegmentationFlag())
        {
//            mpPclMapper_->getGlobalCloud(pcl_map_);
            mpPclMapper_->getObstacleCloud(pcl_map_);
            mpPclMapper_->getPlaneCloud(pcl_plane_);
            mpPclMapper_->getPlaneCoeffs(plane_coeffs_);
        }
        else
        {
            mpPclMapper_->getGlobalCloud(pcl_map_);
        }

        // transform point cloud: footprint_base<--optical_base
        pcl_map_->width = pcl_map_->size();  //! Occassionally it will conflict, force to equal
//        ROS_WARN("Height: %d, Width: %d, Size: %d", pcl_map_->height, pcl_map_->width, pcl_map_->size());
//        ROS_WARN("Point cloud update!");

        // segment plane
        if ( mpPclMapper_->getUsePlanSegmentationFlag() )
        {
            // Broadcast transform map<--odom
            Eigen::Matrix4d mTmo;
            mpPclMapper_->getPlaneTransformMatrix(Eigen::Vector4d(0,0,1,0), plane_coeffs_, mTmo);
//            ROS_WARN_STREAM("map<--odom transform:\n" << mTmo);
            if (use_tf_)
            {
                Eigen::Matrix3d mRmo = mTmo.block(0,0,3,3);
                double tz = mTmo(2,3);
                tf::Matrix3x3 Rmo;
                tf::matrixEigenToTF(mRmo, Rmo);
                tf::StampedTransform trans(tf::Transform(Rmo, tf::Vector3(0, 0, tz)), ros::Time::now(), map_frame_, odom_frame_);
                broadcaster_.sendTransform(trans);
            }

            // transform point cloud to the plane
            pcl::transformPointCloud(*pcl_map_, *pcl_map_, mTmo);
        }
        else if (use_tf_)
        {
            tf::Transform Tmo;
            Tmo.setIdentity();
            tf::StampedTransform trans(Tmo, ros::Time::now(), map_frame_, odom_frame_);
            broadcaster_.sendTransform(trans);
        }

        // sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 pcl2_msgs;
        pcl::toROSMsg(*pcl_map_, pcl2_msgs);
//        ROS_WARN("PCL Message height: %d, width: %d, size: %d", pcl2_msgs.height, pcl2_msgs.width, pcl2_msgs.data.size());
        pcl2_msgs.header.stamp = ros::Time::now();
        pcl2_msgs.header.frame_id = map_frame_;
        pcl2_pub_.publish(pcl2_msgs);
        mpPclMapper_->setPointCloudMapUpdatedFlag(false);

        // sensor_msgs::PointCloud
        sensor_msgs::PointCloud pcl_msgs;
        sensor_msgs::convertPointCloud2ToPointCloud(pcl2_msgs, pcl_msgs);
        pcl_msgs.header.stamp = ros::Time::now();
        pcl_msgs.header.frame_id = map_frame_;
        pcl_pub_.publish(pcl_msgs);
    }
    
    
    bool MessageUtils::getTransformedPose(tf::Stamped<tf::Transform> &output_pose, const string &target_frame, const double &timeout)
    {
        std::string source_frame = output_pose.frame_id_;
        try
        {
            if ( !listener_.waitForTransform(target_frame, source_frame, ros::Time::now(), ros::Duration(timeout)) )
            {
                ROS_ERROR("Wait transform timeout between: [%s]<--[%s], timeout %f s",
                        target_frame.c_str(), source_frame.c_str(), timeout);
                return false;
            }
            listener_.transformPose(target_frame, output_pose, output_pose);
        }
        catch (tf::TransformException &e)
        {
            ROS_ERROR("Fail to find the transform between: [%s]<--[%s]: %s",
                      footprint_frame_.c_str(), optical_frame_.c_str(), e.what());
            return false;
        }
        return true;
    }


    bool MessageUtils::getTransformedPose(Eigen::Matrix4d &output_mat, const string &target_frame, const string &source_frame, const double &timeout)
    {
        Eigen::Isometry3d output_matT(output_mat);
        tf::Transform initTfPose;
        tf::poseEigenToTF(output_matT, initTfPose);
        tf::Stamped<tf::Transform> To(initTfPose, ros::Time::now(), source_frame);
        
        // call override function
        if (!getTransformedPose(To, target_frame, timeout))
        {
            return false;
        }
        
        tf::poseTFToEigen(To, output_matT);
        output_mat = output_matT.matrix();
        return true;
    }
}