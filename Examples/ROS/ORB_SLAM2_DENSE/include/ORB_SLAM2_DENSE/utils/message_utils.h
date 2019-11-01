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
#include <tf/transform_broadcaster.h>

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
        MessageUtils(ORB_SLAM2::System *pSystem);

        void PublishOdometry();

        void PublishFrame();

        void PublishPointCloud();

    protected:
        // pub and sub
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        tf::TransformBroadcaster transform_;
        ros::Publisher odom_pub_;
        ros::Publisher frame_pub_;
        ros::Publisher pcl_pub_;
        ros::Publisher pcl2_pub_;

        // paramters
        bool use_odom_pub_;
        bool use_tf_;
        bool use_frame_pub_;
        bool use_pcl_pub_;
        std::string global_frame_;
        std::string base_frame_;

        // ORB_SLAM2 pointer
        ORB_SLAM2::System *mpSystem_;
        ORB_SLAM2::Tracking *mpTracker_;
        ORB_SLAM2::FrameDrawer *mpFrameDrawer_;
        ORB_SLAM2::MapDrawer *mpMapDrawer_;
        std::shared_ptr<PointCloudMapping> mpPclMapper_;

        // point cloud map
        PointCloudMapping::PointCloud pcl_map_;
    };
}

#endif //ORB_SLAM2_MESSAGE_UTILS_H
