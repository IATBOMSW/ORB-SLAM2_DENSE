//
// Created by yonghui on 19-10-31.
//

#ifndef ORB_SLAM2_DENSE_TIC_TOC_H
#define ORB_SLAM2_DENSE_TIC_TOC_H

#include <ros/time.h>

namespace ORB_SLAM2_DENSE
{
    class TicToc
    {
    public:
        TicToc()
        {
            tic();
        }

        void tic()
        {
            t_start = ros::Time::now();
        }

        double toc()
        {
            ros::Time t_end = ros::Time::now();
            return t_end.toSec() - t_start.toSec();
        }

    protected:
        ros::Time t_start;

    };
}

#endif //ORB_SLAM2_DENSE_TIC_TOC_H
