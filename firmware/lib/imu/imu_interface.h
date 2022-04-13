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

#ifndef IMU_INTERFACE
#define IMU_INTERFACE

#include <sensor_msgs/msg/imu.h>

class IMUInterface
{
    protected:
        sensor_msgs__msg__Imu imu_msg_;
        const float g_to_accel_ = 9.81;
        const float mgauss_to_utesla_ = 0.1;
        const float utesla_to_tesla_ = 0.000001;

        float accel_cov_ = 0.00001;
        float gyro_cov_ = 0.00001;
        const int sample_size_ = 10;

        geometry_msgs__msg__Vector3 gyro_cal_;

        void calibrateGyro()
        {
            geometry_msgs__msg__Vector3 gyro;

            for(int i=0; i<sample_size_; i++)
            {
                updateSensor();
                gyro = readGyroscope();
                gyro_cal_.x += gyro.x;
                gyro_cal_.y += gyro.y;
                gyro_cal_.z += gyro.z;

                delay(50);
            }

            gyro_cal_.x = gyro_cal_.x / (float)sample_size_;
            gyro_cal_.y = gyro_cal_.y / (float)sample_size_;
            gyro_cal_.z = gyro_cal_.z / (float)sample_size_;
        }

        geometry_msgs__msg__Vector3 linear_cal;

        void calibrateLinear()
        {
            geometry_msgs__msg__Vector3 linear;

            for(int i=0; i<sample_size_; i++)
            {
                updateSensor();
                linear = readAccelerometer();
                linear_cal.x += linear.x;
                linear_cal.y += linear.y;
                linear_cal.z += linear.z;

                delay(50);
            }

            linear_cal.x = (linear_cal.x / (float)sample_size_);
            linear_cal.y = (linear_cal.y / (float)sample_size_);
            linear_cal.z = linear_cal.z / (float)sample_size_;
        }

   
    public:
        IMUInterface()
        {
            imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");
        }

        virtual void updateSensor() = 0;
        virtual geometry_msgs__msg__Vector3 readAccelerometer() = 0;
        virtual geometry_msgs__msg__Vector3 readGyroscope() = 0;
        virtual bool startSensor() = 0;

        bool init()
        {
            bool sensor_ok = startSensor();
            if(sensor_ok)
                calibrateGyro();
                calibrateLinear();

            return sensor_ok;
        }

        sensor_msgs__msg__Imu getData()
        {
            imu_msg_.angular_velocity = readGyroscope();
            //calibration offset
            /*
            imu_msg_.angular_velocity.x -= gyro_cal_.x; 
            imu_msg_.angular_velocity.y -= gyro_cal_.y; 
            imu_msg_.angular_velocity.z -= gyro_cal_.z;
            */
           
            //Convert to radians 
            imu_msg_.angular_velocity.x = (imu_msg_.angular_velocity.x - gyro_cal_.x)*PI/180; 
            imu_msg_.angular_velocity.y = (imu_msg_.angular_velocity.y - gyro_cal_.y)*PI/180; 
            imu_msg_.angular_velocity.z = (imu_msg_.angular_velocity.z - gyro_cal_.z)*PI/180;

            
            //Threshold
            if (imu_msg_.angular_velocity.x < 0.004 && imu_msg_.angular_velocity.x > -0.004)
            {
                imu_msg_.angular_velocity.x = 0;
            }
            if (imu_msg_.angular_velocity.y < 0.004 && imu_msg_.angular_velocity.y > -0.004)
            {
                imu_msg_.angular_velocity.y = 0;
            } 
            if (imu_msg_.angular_velocity.z < 0.004 && imu_msg_.angular_velocity.z > -0.004)
            {
                imu_msg_.angular_velocity.z = 0;
            } 
            

            //imu_msg_.angular_velocity.x -= 1.3; 
            //imu_msg_.angular_velocity.y -= 0.9; 
            //imu_msg_.angular_velocity.z -= 1.3; 
       
            imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[8] = gyro_cov_;
            
            imu_msg_.linear_acceleration = readAccelerometer();
            //calibration offset
            imu_msg_.linear_acceleration.x -= linear_cal.x;
            imu_msg_.linear_acceleration.y -= linear_cal.y;
            imu_msg_.linear_acceleration.z -= linear_cal.z;
            imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

            //Threshold            
            if (imu_msg_.linear_acceleration.x < 0.04 && imu_msg_.linear_acceleration.x > -0.04)
            {
                imu_msg_.linear_acceleration.x = 0;
            }
            if (imu_msg_.linear_acceleration.y < 0.06 && imu_msg_.linear_acceleration.y > -0.06)
            {
                imu_msg_.linear_acceleration.y = 0;
            } 
            if (imu_msg_.linear_acceleration.z < 1 && imu_msg_.linear_acceleration.z > -1)
            {
                imu_msg_.linear_acceleration.z = 0;
            } 
            return imu_msg_;
        }
};

#endif
