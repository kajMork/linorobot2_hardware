// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "odometry.h"
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>

Odometry::Odometry():
    x_pos_(0.0),
    y_pos_(0.0),
    heading_(0.0)
{
    odom_msg_.header.frame_id = micro_ros_string_utilities_set(odom_msg_.header.frame_id, "odom");
    odom_msg_.child_frame_id = micro_ros_string_utilities_set(odom_msg_.child_frame_id, "base_footprint");
}

void Odometry::update_pose(double pose[2], double q[4])
{
    double q0 = q[0];
    double q1 = q[1];
    double q2 = q[2];
    double q3 = q[3];
    double yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));

    double percent_pose = 0.2;
    double percent_heading = 0.2;
    x_pos_ = (x_pos_ * (1 - percent_pose)) + (pose[0] * (percent_pose));
    y_pos_ = (y_pos_ * (1 - percent_pose)) + (pose[1] * (percent_pose));
    heading_ = (heading_ * (1 - percent_heading)) + (yaw * (percent_heading));

}

void Odometry::update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
{
    float delta_heading = angular_vel_z * vel_dt; //radians
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; //m
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading*(1/1.5);

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    float q[4];
    euler_to_quat(0, 0, heading_, q);

    //robot's position in x,y, and z
    odom_msg_.pose.pose.position.x = x_pos_;
    odom_msg_.pose.pose.position.y = y_pos_;
    odom_msg_.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    odom_msg_.pose.pose.orientation.x = (double) q[1];
    odom_msg_.pose.pose.orientation.y = (double) q[2];
    odom_msg_.pose.pose.orientation.z = (double) q[3];
    odom_msg_.pose.pose.orientation.w = (double) q[0];

    odom_msg_.pose.covariance[0] = 0.01;
    odom_msg_.pose.covariance[7] = 0.01;
    odom_msg_.pose.covariance[35] = 0.01;

    //linear speed from encoders
    odom_msg_.twist.twist.linear.x = linear_vel_x;
    odom_msg_.twist.twist.linear.y = linear_vel_y;
    odom_msg_.twist.twist.linear.z = 0.0;

    //angular speed from encoders
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;
    odom_msg_.twist.twist.angular.z = 0.0;

    odom_msg_.twist.covariance[0] = 0.001;
    odom_msg_.twist.covariance[7] = 0.001;
    odom_msg_.twist.covariance[35] = 0.001;
}

nav_msgs__msg__Odometry Odometry::getData()
{
    return odom_msg_;
} 

const void Odometry::euler_to_quat(float roll, float pitch, float yaw, float* q) 
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}