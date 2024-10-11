
//--Start Includes--//

// Needed for micro_ros
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Needed for the IMU
#include "MPU6050_6Axis_MotionApps20.h"

//This is needed for the multiplexor
#include <Wire.h>

// This is the message format we will use for the IMU. There are other options though
#include <sensor_msgs/msg/imu.h> //http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html

//Needed for CanBus / ODrive 
#include <Arduino.h>
#include "ODriveCAN.h"

// See https://github.com/tonton81/FlexCAN_T4
// clone https://github.com/tonton81/FlexCAN_T4.git into /src
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"

//--End Includes--//

//In this segment we create the objects required for micro_ros
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer; //If we want to run sensor updates at different intervals, we create more than one timer

/*
//Now we created the publisher, mpu, and msg objects, numbered 0 to n-1, where n is the number of IMUs we have connected
rcl_publisher_t publisher0;
rcl_publisher_t publisher1;
rcl_publisher_t publisher2;
rcl_publisher_t publisher3;
rcl_publisher_t publisher4;

sensor_msgs__msg__Imu msg0;
sensor_msgs__msg__Imu msg1; 
sensor_msgs__msg__Imu msg2; 
sensor_msgs__msg__Imu msg3; 
sensor_msgs__msg__Imu msg4;

MPU6050 imu0;
MPU6050 imu1;
MPU6050 imu2;
MPU6050 imu3;
MPU6050 imu4;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z] //the ros2 quat message expects xyzw        quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gg;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
*/

#define LED_PIN 13 //This is for our error loop

//These are part of the error loop
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//This is required for the multiplexor
#define TCAADDR 0x70

//These states are used to help start the robot without physically disconnecting it
bool micro_ros_init_successful;

//These states are used to help start the robot without physically disconnecting it
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrv0
#define ODRV_NODE_ID_STARBOARD 19 //Becuase S is the 19th letter of the alphabet
#define ODRV_NODE_ID_PORT 16 //Because P is the 16th letter of the alphabet


//This starts the CanBus interface
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// Instantiate ODrive objects
ODriveCAN odrv19(wrap_can_intf(can_intf), ODRV_NODE_ID_STARBOARD); // Standard CAN message ID
ODriveCAN odrv16(wrap_can_intf(can_intf), ODRV_NODE_ID_PORT); // Standard CAN message ID


ODriveCAN* odrives[] = {&odrv16, &odrv19}; // Make sure all ODriveCAN instances are accounted for here

// Keep some application-specific user data for every ODrive.
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}


ODriveUserData odrv16_user_data;
ODriveUserData odrv19_user_data;
// inputs actual velocity for left and right wheel, from odrive data
//outputs linear vel in x dir in revs/s (i.e whatever units the input vels are)
float generateLinearVel(float lwvel, float rwvel){
float c1 = 1;
float c2 =1;
float gearratio = (12/40)*(12/40);

return gearratio*(WHEELRAD/2)*(c1*lwvel +c2*rwvel) 
}


//inputs actual velocity for l and r wheel, from odrive data
//outputs angular vel in z dir in rev/s (whatever units the input is)
float generateAngularVel(float lwvel, float rwvel){
float c1 = 1;
float c2 =1;
float gearratio = (12/40)*(12/40);

return gearratio*(WHEELRAD/2)*((c1*lwvel-c2*rwvel)/WHEELSEP)
}



// Called for every message that arrives on the CAN bus

void onCanMessage(const CanMsg& msg) {

  for (auto odrive: odrives) {

    onReceive(msg, *odrive);

  }

}

//im like 78.23% sure that this message does come in as rev/s, so it should be good to go like that.
void subscription_callback(const void *msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  float vel = msg->velocity.data[0];
  float vel2=msg->velocity.data[1];
   odrv16.setVelocity(
    vel
  ); 
   odrv19.setVelocity(
    vel2
  ); 
}

void setup() 
  {
  state = WAITING_AGENT;

  Wire.begin();
  Serial.begin(115200);
  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }

  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  //setupIMU();
  setupCan();
  setupODrive();
odrv16.onFeedback(onFeedback, &odrv16_user_data);
odrv16.onStatus(onHeartbeat, &odrv16_user_data);
odrv19.onFeedback(onFeedback, &odrv19_user_data);
odrv19.onStatus(onHeartbeat, &odrv19_user_data);
 while (odrv16_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {

    odrv16.clearErrors();

    delay(1);

    odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    for (int i = 0; i < 15; ++i) {

      delay(10);

      pumpEvents(can_intf);

    }

  }
   while (odrv19_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv19.clearErrors();
    delay(1);
    odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    for (int i = 0; i < 15; ++i) {

      delay(10);

      pumpEvents(can_intf);

    }

  }

}



void loop() {
  switch (state)
  {
  case WAITING_AGENT:
    if (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
    {
      state = AGENT_AVAILABLE;
    }
    break;
  case AGENT_AVAILABLE:
    if (create_entities())
    {
      state = AGENT_CONNECTED;
    }
    else
    {
      state = WAITING_AGENT;
    }
    break;
  case AGENT_CONNECTED:
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1))
    {
      state = AGENT_DISCONNECTED;
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  }
  if (state == AGENT_CONNECTED)
  {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 1));
  }
}
