// real file

/*
This code is designed for a Teensy 4.1 and will run code to connect to a micro-ros agent over the serial port.
This is the main file. Here we set up our constant definitions and variable declarations, as well as the main startup and loop functions.
*/
//--Start Includes--//

// Needed for micro_ros
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float64.h>
#include <geometry_msgs/msg/vector3.h>


// This is needed for the multiplexor
#include <Wire.h>

// Needed for CanBus / ODrive
#include <Arduino.h>
//clone https://github.com/odriverobotics/ODriveArduino
//make changes as directed in "MicroROS/Teensy research document", found in RIPLab/UXS/ROS drive folder.

//this is needed because the Arduino toolchain does not support C-style designated initializers reliably
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error
//this must be imported before flexcan, because there are some conflicts with the FlexCAN_T4 library

// See https://github.com/tonton81/FlexCAN_T4
// clone https://github.com/tonton81/FlexCAN_T4.git into libaries
#include <FlexCAN_T4.h>


//--End Includes--//

// start definitions
#define WHEELRAD 0.05715 // given that the wheel diameter is 4.5 inches, the radius is 2.25 inches or 0.05715 meters
#define WHEELSEP 0.42926 //.508 m is the outer wheel sep, 0.42926m is the inner wheel sep //.48m is original
#define GEARRATIO 11.1111

#define LED_PIN 13 // built in led
// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrv0
#define ODRV_NODE_ID_STARBOARD 19 // Becuase S is the 19th letter of the alphabet
#define ODRV_NODE_ID_PORT 16      // Because P is the 16th letter of the alphabet

//limits for the odrive trapezoidal controller
//https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.TrapezoidalTrajectory.Config
#define VEL_LIMIT 8.0            // rev/s
#define ACCEL_LIMIT 3.0          // rev/s^2
#define DECEL_LIMIT 3.0          // rev/s^2


// end definitions

// start declarations
//  In this segment we create the objects required for micro_ros
static micro_ros_utilities_memory_conf_t conf = {0};
rclc_executor_t executor;
rcl_subscription_t joint_state_subscriber;
rcl_subscription_t pos_subscriber;
rcl_subscription_t odom_flag_subscriber;
rcl_publisher_t OdomPublisher;
// rcl_publisher_t TFpublisher;
// rcl_publisher_t LeftWheelPublisher;
// rcl_publisher_t RightWheelPublisher;
rcl_publisher_t WheelPublisher;

rcl_publisher_t LeftVoltagePublisher;
rcl_publisher_t RightVoltagePublisher;



rcl_publisher_t JointPublisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_clock_t clock;
double time_now, time_old = 0.0;

rcl_timer_t timer;
// these msg are used to publish data
sensor_msgs__msg__JointState joint_state_msg; 
std_msgs__msg__Float64 pos_msg;
sensor_msgs__msg__JointState joint_publish_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Float64 odom_flag_msg;
geometry_msgs__msg__Vector3 wheel_pos_msg;
geometry_msgs__msg__Vector3 left_voltage_msg;
geometry_msgs__msg__Vector3 right_voltage_msg;

// this allows for frames to be specified in msgs, as they ask for a specific type. see below
// https://docs.vulcanexus.org/en/iron/rst/microros_documentation/user_api/user_api_utilities.html
const char *str = "odom";
rosidl_runtime_c__String odom_str = micro_ros_string_utilities_init(str);
const char *str1 = "base_link";
rosidl_runtime_c__String base_str = micro_ros_string_utilities_init(str1);
const char *str2 = "leftwheel";
rosidl_runtime_c__String left_str = micro_ros_string_utilities_init(str2);
const char *str3 = "rightwheel";
rosidl_runtime_c__String right_str = micro_ros_string_utilities_init(str3);

// This is an error function that will blink the LED if something goes wrong
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// These states are used to help start the robot without physically disconnecting it
bool micro_ros_init_successful;
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
  }
}
void blink_led(int times, int delay_ms = 500)
{
  for (int i = 0; i < times; ++i)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(delay_ms);
    digitalWrite(LED_PIN, LOW);
    delay(delay_ms);
  }
}
// This starts the CanBus interface
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_intf;

// Instantiate ODrive objects
ODriveCAN odrv19(wrap_can_intf(can_intf), ODRV_NODE_ID_STARBOARD); // Standard CAN message ID
ODriveCAN odrv16(wrap_can_intf(can_intf), ODRV_NODE_ID_PORT);      // Standard CAN message ID

ODriveCAN *odrives[] = {&odrv16, &odrv19}; // Make sure all ODriveCAN instances are accounted for here

// Keep some application-specific user data for every ODrive.
struct ODriveUserData
{
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};
ODriveUserData odrv16_user_data;
ODriveUserData odrv19_user_data;

/*odom declarations*/
double wheelc1 = 1.0;
double wheelc2 = 1.0;
double lwvel;
double rwvel;
double linvel;
double angvel;
double x;
double y;
double x_pos;
double y_pos;
double theta_pos;
double lwpos = 0;
double rwpos = 0;
double lwpos_prev = 0;
double rwpos_prev = 0;
double delta_lwpos;
double delta_rwpos;
double Davg;
double Dth;
Get_Encoder_Estimates_msg_t encoderFeedback16;
Get_Encoder_Estimates_msg_t encoderFeedback19;
int64_t time_ns_now;
int64_t time_microseconds_now;
int64_t time_ns_old;
bool first = true;

/*end odom declarations*/

void setup()
{
  state = WAITING_AGENT;
  Wire.begin();
  Serial.begin(115200);
  // Wait for up to 1 seconds for the serial port to be opened on the PC side.
  //  If no PC connects, continue anyway.
  for (int i = 0; i < 300 && !Serial; ++i)
  {
    delay(10);
  }

  set_microros_transports();
  // Synchronize time with the agent
  double timeout_ms = 100;
  rmw_uros_sync_session(timeout_ms);

  if (rmw_uros_epoch_synchronized())
  {
    // Get time in milliseconds or nanoseconds

    time_ns_now = rmw_uros_epoch_nanos();
    time_ns_old = time_ns_now;
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  setupCan();

  // odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  // odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  // odrv16.setControllerMode(2, 1);
  // odrv19.setControllerMode(2, 1);
  setupODrive();
  odrv16_user_data.last_feedback.Pos_Estimate = 0;
  odrv19_user_data.last_feedback.Pos_Estimate = 0;

}

//odrive voltage stuff
bool getPower(Get_Powers_msg_t &msg, uint16_t timeout_ms = 10); //already in can.h
Get_Powers_msg_t pwrMsg16;
Get_Powers_msg_t pwrMsg19;


// this is the main loop. this function is called repeatedly by the Teensy
// the state machine is set up here to allow micro ros to work properly
//  in the case that shutdown does not go smoothly or it takes some time to connect to the micro-ros agent on the main machine (PI)

void loop()
{

  switch (state) // checks what state we should be in
  {
  case WAITING_AGENT:
    if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) // ping until we get a response from the agent
    {
      state = AGENT_AVAILABLE;
    }
    break; // exits from the switch, effectively starting the switch over again as there is no other code in loop()
  case AGENT_AVAILABLE:
    if (create_entities()) // create_entities returns true if successful
    {
      state = AGENT_CONNECTED;
    }
    else
    {
      state = WAITING_AGENT;
    }
    break; // exits from the switch again
  case AGENT_CONNECTED:
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1))
    {
      state = AGENT_DISCONNECTED;
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities(); // if we lose connection, destroy the entities, then go back to default state
    state = WAITING_AGENT;
    break;
  } // end switch

  if (state == AGENT_CONNECTED) // at the end of every loop, check if we are connected to agent
  {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // run the executor 1 time 
} // start the loop over again
}
