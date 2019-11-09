//
// Created by yonghui on 19-10-30.
//

#ifndef ORB_SLAM2_MESSAGE_UTILS_H
#define ORB_SLAM2_MESSAGE_UTILS_H

// ORB_SLAM2
#include "System.h"
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "pointcloudmapping.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// STL
#include <string>
#include <memory>

namespace ORB_SLAM2
{
    class System;
    class Tracking;
    class FrameDrawer;
    class MapDrawer;
}
class PointCloudMapping;


namespace ORB_SLAM2_DENSE
{
    class MessageUtils
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        MessageUtils(tf::TransformListener &listener, ORB_SLAM2::System *pSystem);

        void publishOdometry();

        void publishFrame();

        void publishPointCloud();

    protected:
        bool getTransformedPose(tf::Stamped<tf::Transform> &output_pose, const string &target_frame, const double &timeout=1.0);
        
        bool getTransformedPose(Eigen::Matrix4d &output_mat, const string &target_frame, const string &source_frame, const double &timeout=1.0);

//        bool planeSACSegmentation(PointCloudMapping::PointCloud::Ptr &pcl_map,
//                                  PointCloudMapping::PointCloud::Ptr &pcl_plane, Eigen::Vector4d &plane_coeffs);
        
        // node handler
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // tf
        tf::TransformListener &listener_;
        tf::TransformBroadcaster broadcaster_;

        // pub and sub
        ros::Publisher odom_pub_;
        ros::Publisher frame_pub_;
        ros::Publisher pcl_pub_;
        ros::Publisher pcl2_pub_;

        // parameters
        bool use_odom_pub_;
        bool use_tf_;
        bool use_frame_pub_;
        bool use_pcl_pub_;
//        bool use_plane_segment_;
//        double min_z_;
//        double max_z_;
//        double plane_dist_thres_;

        // frame id
        std::string map_frame_;
        std::string odom_frame_;
        std::string footprint_frame_;
        std::string optical_frame_;

        // ORB_SLAM2 pointer
        ORB_SLAM2::System *mpSystem_;
        ORB_SLAM2::Tracking *mpTracker_;
        ORB_SLAM2::FrameDrawer *mpFrameDrawer_;
        ORB_SLAM2::MapDrawer *mpMapDrawer_;
        std::shared_ptr<PointCloudMapping> mpPclMapper_;

        // point cloud map
        PointCloudMapping::PointCloud::Ptr pcl_map_;
        PointCloudMapping::PointCloud::Ptr pcl_plane_;
        Eigen::Vector4d plane_coeffs_;
        Eigen::Vector4d last_plane_coeffs_;
    };
}

#endif //ORB_SLAM2_MESSAGE_UTILS_H
