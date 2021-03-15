#pragma once

/*
 * Copyright (C) 2019-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <cav_driver_utils/driver_wrapper/driver_wrapper.h>
#include <sensor_msgs/PointCloud2.h>

class VelodyneLidarDriverWrapper : public cav::DriverWrapper
{

public:
    VelodyneLidarDriverWrapper(int argc, char **argv, const std::string &name = "velodyne_lidar_driver_wrapper");
    virtual ~VelodyneLidarDriverWrapper();

private:

    ros::Subscriber point_cloud_sub_;
    ros::Time last_update_time_;
    double point_cloud_timeout_;

    /**
     * @brief Callback for handling point cloud message
     */
    void point_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

    /**
     * @brief Check lidar data output timeout
     */
    void checkLidarTimeout();

    //cav::DriverWrapper members
    virtual void initialize();
    virtual void pre_spin();
    virtual void post_spin();
    virtual void shutdown();

};
