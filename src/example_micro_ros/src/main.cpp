#include "Wire.h"
#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <Kinematics.h>
#include <MPU6050_light.h>
#include <PidController.h>

// 引入microros和wifi相关的库
#include <WiFi.h>
#include <geometry_msgs/msg/twist.h> //角速度线速度
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h> //引入字符串内存分配工具
#include <nav_msgs/msg/odometry.h>                // 里程计
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

// 声明一些相关的结构体对象
rcl_allocator_t allocator; // 内存分配器
rclc_support_t support;    // 存储时钟, 内存分配
rclc_executor_t executor;  // 执行器, 用于管理订阅和计时器回调
rcl_node_t node;
rcl_subscription_t sub_cmd_vel; // 订阅者
geometry_msgs__msg__Twist msg_cmd_vel;

rcl_publisher_t pub_odom;
nav_msgs__msg__Odometry msg_odom;
rcl_timer_t timer;

Esp32McpwmMotor motor;
Esp32PcntEncoder encoders[2];
PidController pid[2];
Kinematics kinematics;
// Rotate
float target_linear_speed = 20.0; // mm/s
float target_angular_speed = 0.1; // rad/s
float out_left_speed = 0.0;
float out_right_speed = 0.0;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // 里程计设置
    odom_t odom = kinematics.get_odom();
    int64_t stamp = rmw_uros_epoch_millis();
    msg_odom.header.stamp.sec = static_cast<int32_t>(stamp / 1000);             // 秒部分
    msg_odom.header.stamp.nanosec = static_cast<int32_t>((stamp % 1000) * 1e6); // 纳秒部分
    msg_odom.pose.pose.position.x = odom.x;
    msg_odom.pose.pose.position.y = odom.y;
    msg_odom.pose.pose.orientation.w = cos(odom.angle * 0.5);
    msg_odom.pose.pose.orientation.x = 0;
    msg_odom.pose.pose.orientation.y = 0;
    msg_odom.pose.pose.orientation.z = sin(odom.angle * 0.5);
    msg_odom.twist.twist.linear.x = odom.linear_speed;
    msg_odom.twist.twist.angular.z = odom.angular_speed;
    // 里程计发布
    if (rcl_publish(&pub_odom, &msg_odom, NULL) != RCL_RET_OK)
    {
        Serial.println("error; odom pub failed!");
    }
}

void twist_callback(const void *msg_in)
{
    // 将收到的消息指针转换成 geometry_msgs__msg__Twist 类型的指针
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    target_linear_speed = msg->linear.x * 1000; // m/s to mm/s
    target_angular_speed = msg->angular.z;
    kinematics.kinematics_inverse(target_linear_speed, target_angular_speed, &out_left_speed, &out_right_speed);
    Serial.printf("OUT:left_speed=%f, right_speed=%f\n", out_left_speed, out_right_speed);
    pid[0].update_target(out_left_speed);
    pid[1].update_target(out_right_speed);
};

// 单独创建一个task运行 micro-ros 相当于一个线程
void microros_task(void *args)
{
    // 设置传输协议并等待完成
    IPAddress agent_ip;
    agent_ip.fromString("10.0.0.34");
    set_microros_wifi_transports(const_cast<char *>("fishros"), const_cast<char *>("88888888"), agent_ip, 8888);
    delay(2000);
    // 初始化内存分配
    allocator = rcl_get_default_allocator();
    // 初始化支持
    rclc_support_init(&support, 0, NULL, &allocator);
    // 初始化节点
    rclc_node_init_default(&node, "fishbot_motion_control", "", &support);
    // 初始化执行器
    unsigned int num_handles = 2;
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    // 初始化订阅者
    rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                       "/cmd_vel");
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &twist_callback, ON_NEW_DATA);
    // 初始化msg
    msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, "odom");
    msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, "base_footprint");
    // 初始化发布者
    rclc_publisher_init_best_effort(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
    rclc_executor_add_timer(&executor, &timer);
    // 初始化时间同步
    while (!rmw_uros_epoch_synchronized())
    {
        rmw_uros_sync_session(1000);
        delay(10);
    }
    rclc_executor_spin(&executor); // 循环执行
}

#define TRIG 27
#define ECHO 21

MPU6050 mpu(Wire);

unsigned long time_now = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(2, OUTPUT);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    Wire.begin(18, 19); // SDA, SCL

    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0)
    {
    } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true); // gyro and accelero
    Serial.println("MPU6050 Done!\n");

    motor.attachMotor(0, 22, 23); // dir, cdir
    motor.attachMotor(1, 12, 13); // dir, cdir
    encoders[0].init(0, 32, 33);  // A, B
    encoders[1].init(1, 26, 25);  // A, B

    // pid controller
    pid[0].update_pid(0.625, 0.125, 0.00);
    pid[1].update_pid(0.625, 0.125, 0.00);
    pid[0].out_limit(-100, 100);
    pid[1].out_limit(-100, 100);
    // 初始化运动学参数
    /* calibrated: rotation test under-reported
        // Your two measurements together answer it: wheel_distance is too large. Both directions came up short of a full turn:
        // - CW turn: yaw ended +14.26° — sweeping clockwise (negative) it only accumulated ~345.7° of the 360°.
        // - CCW turn: yaw ended −16.03° — sweeping counter-clockwise (positive) it only accumulated ~344.0°.
        // The intuition: firmware computes yaw as (right wheel travel − left wheel travel) ÷ wheel_distance.
        // If the wheel_distance it divides by is bigger than the real track, every rotation comes out too small — exactly your symptom,
        // symmetric in both directions (which also confirms it's geometry, not an encoder problem).
        // The correction: average under-report factor = (345.74/360 + 343.97/360) / 2 ≈ 0.958, so:
        // new wheel_distance = 175 × 0.958 ≈ 167.6 mm
        //
        // Round 2 with 167.6: still under-reported, CW short 0.051 rad (2.9°), CCW short 0.095 rad (5.4°).
        // Average residual = (0.051 + 0.095)/2 = 0.073 rad/turn → factor (2π − 0.073)/2π ≈ 0.9884
        // new wheel_distance = 167.6 × 0.9884 ≈ 165.7 mm (expect ~±1.3° per turn residual after this)
    */
    kinematics.set_wheel_distance(165.7); // mm
    // where 10 laps ticks 19766
    // 67mm is the wheel diameter
    // distance per tick = 3.141593*67/1976 = 0.10657556 mm
    kinematics.set_motor_param(0, 0.10657556);
    kinematics.set_motor_param(1, 0.10657556);

    xTaskCreate(microros_task, "microros_task", 10240, NULL, 1, NULL);
}

void readMPU6050()
{
    // MPU IMU sensor
    mpu.update();

    if (millis() - time_now > 1000)
    { // print data every second
        Serial.print(F("TEMPERATURE: "));
        Serial.println(mpu.getTemp());
        Serial.print(F("ACCELERO  X: "));
        Serial.print(mpu.getAccX());
        Serial.print("\tY: ");
        Serial.print(mpu.getAccY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getAccZ());

        Serial.print(F("GYRO      X: "));
        Serial.print(mpu.getGyroX());
        Serial.print("\tY: ");
        Serial.print(mpu.getGyroY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getGyroZ());

        Serial.print(F("ACC ANGLE X: "));
        Serial.print(mpu.getAccAngleX());
        Serial.print("\tY: ");
        Serial.println(mpu.getAccAngleY());

        Serial.print(F("ANGLE     X: "));
        Serial.print(mpu.getAngleX());
        Serial.print("\tY: ");
        Serial.print(mpu.getAngleY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getAngleZ());
        Serial.println(F("=====================================================\n"));
        time_now = millis();
    }
}

void readLED()
{
    // LED
    digitalWrite(2, LOW);
    delay(1000);
    digitalWrite(2, HIGH);
    delay(1000);
}

void readUltrasonicSensor()
{
    // Ultrasonic Sensor
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    double duration = pulseIn(ECHO, HIGH);
    float distance = duration * 0.034 / 2; // 340 m/s = 0.034 cm/µs
    Serial.printf("Distance: %f cm\n", distance);
}

void loop()
{
    // readMPU6050();
    // readLED();
    readUltrasonicSensor();
    // Motor
    delay(10);
    kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks());
    motor.updateMotorSpeed(0, pid[0].update(kinematics.get_motor_speed(0)));
    motor.updateMotorSpeed(1, pid[1].update(kinematics.get_motor_speed(1)));
    Serial.printf("x, y, yaw=%f, %f, %f\n", kinematics.get_odom().x, kinematics.get_odom().y,
                  kinematics.get_odom().angle);
}