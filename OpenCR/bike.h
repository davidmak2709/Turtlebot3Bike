
#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>

#include <TurtleBot3.h>

#include "bike_motor_driver.h"

#define INIT_LOG_DATA "This core(v1.1.2) is compatible with TB3 Bike"
#define HARDWARE_VER "1.0.0"
#define SOFTWARE_VER "1.0.0"
#define FIRMWARE_VER "1.1.2"

#define CONTROL_MOTOR_SPEED_PERIOD          30   //hz
#define IMU_PUBLISH_PERIOD                  200  //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD    30   //hz
#define VERSION_INFORMATION_PUBLISH_PERIOD  1    //hz 

#define LEFT 0
#define RIGHT 2
#define FRONT 1

#define WHEEL_NUM                       3
#define WHEEL_RADIUS                    0.033     // meter
#define WHEEL_SEPARATION                0.16      // meter (BURGER => 0.16, WAFFLE => 0.287)
#define ROBOT_LENGTH                    0.165     // meter

#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define VELOCITY_CONSTANT_VAULE         1263.632956882  // V = r * w = r * RPM * 0.10472
                                                        //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                        // Goal RPM = V * 1263.632956882

#define CONTROL_PERIOD                  8000

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI
#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f


//callback proto
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void soundCallback(const turtlebot3_msgs::Sound& sound_msg);
void motorPowerCallback(const std_msgs::Bool& power_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

// Function prototypes
void controlMotorSpeed(void);
void publishImuMsg(void);
void publishMagMsg(void);
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros);

/****** ROS ******/
//Node handler
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

//Subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<turtlebot3_msgs::Sound> sound_sub("sound", soundCallback);
ros::Subscriber<std_msgs::Bool> motor_power_sub("motor_power", motorPowerCallback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

//Publisher
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("version_info", &version_info_msg);

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Odometry of Turtlebot3 Bike
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Turtlebot3
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

//Transform Broadcaster
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

//Declaration of sensors and diagnosis
Turtlebot3Sensor sensors;
Turtlebot3Diagnosis diagnosis;

//Battery
bool setup_end        = false;
bool init_encoder = true;

int32_t last_diff_tick[WHEEL_NUM] = {0.0, 0.0, 0.0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0, 0.0};
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0, 0.0};

uint8_t battery_state = 0;
static uint32_t tTime[4];
unsigned long prev_update_time;

float odom_pose[3];
double odom_vel[3];
