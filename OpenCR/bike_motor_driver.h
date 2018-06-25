#ifndef BIKE_MOTOR_DRIVER_H_
#define BIKE_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_PROFILE_ACCELERATION     108
#define ADDR_X_PROFILE_VELOCITY         112
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DXL_LEFT_REAR_ID                2       // ID of left rear motor
#define DXL_RIGHT_REAR_ID               1       // ID of right rear motor
#define DXL_FRONT_ID                    3       // ID of front motor
#define BAUDRATE                        1000000 // baud rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define X_POS_MIN                       0
#define X_POS_MAX                       4095
#define X_POS_CENTER                    2048

class BikeMotorDriver
{
 public:
  BikeMotorDriver();
  ~BikeMotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool setProfileAcceleration(uint8_t id, uint32_t value);
  bool setProfileVelocity(uint8_t id, uint32_t value);
  bool controlMotor(int64_t left_wheel_value, int64_t right_wheel_value, int64_t front_joint_value);
  bool readEncoder(int32_t &left_value, int32_t &right_value, int32_t &front_value);
 
 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_rear_wheel_id_, right_rear_wheel_id_;
  uint8_t front_joint_id_;
  bool flag = true;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupBulkWrite *groupBulkWrite_;
  dynamixel::GroupSyncRead *groupSyncReadEncoder_;
};

#endif // TURTLEBOT3_BIKE_MOTOR_DRIVER_H_
