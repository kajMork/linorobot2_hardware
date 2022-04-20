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
#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <OPT3101.h>
#include <Wire.h>
#include <string>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/range.h>

#include "config.h"

#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include <SPI.h>



#define SS_M4 23
#define SS_M3 22
#define SS_M2 21
#define SS_M1 20

#define ENABLE_MOTORS 8

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)

void check_for_reset ()
{
    if (Serial.available()) {
        if (Serial.read() == 'R')
        {
            CPU_RESTART;
        }
    }
}

void flashLED(int n_times)
{
    check_for_reset();
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        check_for_reset();
        delay(150);
        check_for_reset();
        digitalWrite(LED_PIN, LOW);
        check_for_reset();
        delay(150);
        check_for_reset();
    }
    check_for_reset();
    delay(1000);
}

void rclErrorLoop() 
{
    while(true)
    {
        flashLED(2);
    }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;
rcl_publisher_t left_range_publisher;
rcl_publisher_t middle_range_publisher;
rcl_publisher_t right_range_publisher;
rcl_subscription_t pose_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__PoseWithCovarianceStamped pose_msg;
sensor_msgs__msg__Range m_ir_range_msg;
sensor_msgs__msg__Range l_ir_range_msg;
sensor_msgs__msg__Range r_ir_range_msg;

//geometry_msgs__msg__Vector3 gyroCalib;
//geometry_msgs__msg__Vector3 linearCalib;

rclc_executor_t executor;
rclc_executor_t executor_2;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
bool micro_ros_init_successful = false;
float distances[3];

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);


Odometry odometry;
IMU imu;
OPT3101 IR_sensor;

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void twistCallback(const void * msgin) 
{
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

void correct_pose_callback(const void * msgin)
{
    double percent_pose = 0.8;
    double pose[2] = {pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y};
    double orientation[4] = {pose_msg.pose.pose.orientation.w, pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z};

    odometry.update_pose(pose, orientation);
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}
void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        //digitalWrite(LED_PIN, HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );

    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1, 
        current_rpm2, 
        current_rpm3, 
        current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
}

void irSampling()
{
    for (size_t i = 0; i < 3; i++)
    {
        while(IR_sensor.isSampleDone())
        {
            IR_sensor.readOutputRegs();

            distances[IR_sensor.channelUsed] = IR_sensor.distanceMillimeters;

            IR_sensor.nextChannel();
            IR_sensor.startSample();
            break;
        }
    }
}
 
void publishData()
{
    odom_msg = odometry.getData();
    imu.updateSensor();
    imu_msg = imu.getData();
    
    irSampling();
    
    l_ir_range_msg.header.frame_id = micro_ros_string_utilities_set(l_ir_range_msg.header.frame_id , "left_tri_ir_range_frame");
    l_ir_range_msg.radiation_type = 1;
    l_ir_range_msg.field_of_view = 0.872664626;
    l_ir_range_msg.min_range = 0.0;
    l_ir_range_msg.max_range = 0.6; //1;
    l_ir_range_msg.range = distances[0]/1000;

    m_ir_range_msg.header.frame_id = micro_ros_string_utilities_set(m_ir_range_msg.header.frame_id , "tri_ir_range_frame");
    m_ir_range_msg.radiation_type = 1;
    m_ir_range_msg.field_of_view = 1.04719755;
    m_ir_range_msg.min_range = 0.0;
    m_ir_range_msg.max_range = 0.6;
    m_ir_range_msg.range = distances[1]/1000;

    r_ir_range_msg.header.frame_id = micro_ros_string_utilities_set(r_ir_range_msg.header.frame_id , "right_tri_ir_range_frame");
    r_ir_range_msg.radiation_type = 1;
    r_ir_range_msg.field_of_view = 0.872664626;
    r_ir_range_msg.min_range = 0.0;
    r_ir_range_msg.max_range = 0.6; //1;
    r_ir_range_msg.range = distances[2]/1000;

    /*imu_msg.angular_velocity.x -= gyroCalib.x;
    imu_msg.angular_velocity.y -= gyroCalib.y;
    imu_msg.angular_velocity.z -= gyroCalib.z;

    imu_msg.linear_acceleration.x -= linearCalib.x;
    imu_msg.linear_acceleration.y -= linearCalib.y;
    imu_msg.linear_acceleration.z -= linearCalib.z; */
    
    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    m_ir_range_msg.header.stamp.sec = time_stamp.tv_sec;
    m_ir_range_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    l_ir_range_msg.header.stamp.sec = time_stamp.tv_sec;
    l_ir_range_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    r_ir_range_msg.header.stamp.sec = time_stamp.tv_sec;
    r_ir_range_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    RCSOFTCHECK(rcl_publish(&left_range_publisher, &l_ir_range_msg, NULL));
    RCSOFTCHECK(rcl_publish(&middle_range_publisher, &m_ir_range_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_range_publisher, &r_ir_range_msg, NULL));
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       moveBase();
       publishData();
    }
}

void createEntities()
{
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"
    ));
    // create left IR range publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &left_range_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/tri_ir_sensor/left/range"));
    // create right IR range publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &right_range_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/tri_ir_sensor/right/range"));
    // create middle IR range publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &middle_range_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/tri_ir_sensor/middle/range"));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_best_effort( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_best_effort( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));
    RCCHECK(rclc_subscription_init_best_effort( 
        &pose_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped),
        "pose"
    ));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 10, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));
    executor_2 = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_2, &support.context, 10, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor_2, 
        &pose_subscriber, 
        &pose_msg, 
        &correct_pose_callback, 
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);
    micro_ros_init_successful = true;
}

void destroyEntities()
{
    digitalWrite(LED_PIN, LOW);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&left_range_publisher, &node);
    rcl_publisher_fini(&middle_range_publisher, &node);
    rcl_publisher_fini(&right_range_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_subscription_fini(&pose_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_executor_fini(&executor_2);
    rclc_support_fini(&support);

    micro_ros_init_successful = false;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}


void setup() 
{   
    
    unsigned int configWord;
    // put your setup code here, to run once:
    pinMode(SS_M4, OUTPUT); digitalWrite(SS_M4, HIGH);  // HIGH = not selected
    pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, HIGH);
    pinMode(SS_M2, OUTPUT); digitalWrite(SS_M2, HIGH);
    pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, HIGH);

    pinMode(ENABLE_MOTORS, OUTPUT);  digitalWrite(ENABLE_MOTORS, HIGH);   // HIGH = disabled

    configWord = 0b0000010000001100;
  
    Wire.begin();

    IR_sensor.init();
    IR_sensor.setFrameTiming(256);
    IR_sensor.setChannel(0);
    IR_sensor.setBrightness(OPT3101Brightness::High);
    IR_sensor.startSample();

    SPI.begin();
    SPI.setBitOrder(LSBFIRST);
    SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high

    // Motor 1
    digitalWrite(SS_M1, LOW);
    SPI.transfer(lowByte(configWord));
    SPI.transfer(highByte(configWord));
    digitalWrite(SS_M1, HIGH); 
    // Motor 2
    digitalWrite(SS_M2, LOW);
    SPI.transfer(lowByte(configWord));
    SPI.transfer(highByte(configWord));
    digitalWrite(SS_M2, HIGH);
    // Motor 3
    digitalWrite(SS_M3, LOW);
    SPI.transfer(lowByte(configWord));
    SPI.transfer(highByte(configWord));
    digitalWrite(SS_M3, HIGH);
    // Motor 4
    digitalWrite(SS_M4, LOW);
    SPI.transfer(lowByte(configWord));
    SPI.transfer(highByte(configWord));
    digitalWrite(SS_M4, HIGH);
    pinMode(LED_PIN, OUTPUT);

    bool imu_ok = imu.init();
    if(!imu_ok)
    {
        while(1)
        {
            flashLED(3);
        }
    }

    
    micro_ros_init_successful = false;
    set_microros_transports();
    createEntities();
}

//geometry_msgs__msg__Vector3 gyroCalib = imu.calibrateGyro();
//geometry_msgs__msg__Vector3 linearCalib = imu.calibrateLinear();

void loop() 
{
    digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
    static unsigned long prev_connect_test_time;
    // check if the agent got disconnected at 10Hz
    /*if(millis() - prev_connect_test_time >= 1000)
    {
        prev_connect_test_time = millis();
        // check if the agent is connected
        if(RMW_RET_OK == rmw_uros_ping_agent(10, 2))
        {
            // reconnect if agent got disconnected or haven't at all
            if (!micro_ros_init_successful) 
            {
                createEntities();
            } 
        } 
        else if(micro_ros_init_successful)
        {
            // stop the robot when the agent got disconnected
            fullStop();+
            // clean up micro-ROS components
            destroyEntities();
        }
    }
    
    if(micro_ros_init_successful)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    */
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
    rclc_executor_spin_some(&executor_2, RCL_MS_TO_NS(20));
}

