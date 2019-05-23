/*
 * Copyright (C) 2019 LEIDOS.
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

#include "velodyne_lidar_driver_wrapper.h"

VelodyneLidarDriverWrapper::VelodyneLidarDriverWrapper(int argc, char **argv, const std::string &name) : DriverWrapper (argc, argv, name), point_cloud_timeout_(0.3) {}

VelodyneLidarDriverWrapper::~VelodyneLidarDriverWrapper() {}

void VelodyneLidarDriverWrapper::initialize() {

    // Set driver type
    status_.lidar   = true;

    // Initilize the subscriber for point cloud data
    point_cloud_sub_ = nh_->subscribe("/velodyne_points", 1, &VelodyneLidarDriverWrapper::point_cloud_cb, this);

    private_nh_->param<double>("point_cloud_timeout", point_cloud_timeout_, 0.3);

}

void VelodyneLidarDriverWrapper::pre_spin()
{
    checkLidarTimeout();
}

void VelodyneLidarDriverWrapper::post_spin() {}

void VelodyneLidarDriverWrapper::shutdown() {}

void VelodyneLidarDriverWrapper::point_cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    last_update_time_ = ros::Time::now();
    status_.status = cav_msgs::DriverStatus::OPERATIONAL;
}

void VelodyneLidarDriverWrapper::checkLidarTimeout()
{
    if(last_update_time_.isZero() ||
       ros::Time::now() - last_update_time_ > ros::Duration(point_cloud_timeout_))
    {
        status_.status = cav_msgs::DriverStatus::OFF;
    }
}
