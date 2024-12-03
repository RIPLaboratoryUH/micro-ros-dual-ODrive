
//To Do
//1) Keep adding tabs
//2) Remove encoderFeedback16/19
//3) Verify 

//--Start Includes--//

//Needed for microros
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//Varios message types used
#include <sensor_msgs/msg/joint_state.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>

// Wire.h is needed for the multiplexor
#include <Wire.h>

// Needed for CanBus / ODrive
#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"

//--End Includes--//

// In this segment we create the objects required for micro_ros
// 1x executor, 1x support, 1x subscriber, 1x allocator, 1x node
// If we want to run sensor updates at different intervals, we create more than one timer

static micro_ros_utilities_memory_conf_t conf = {0};

rclc_executor_t executor;

rclc_support_t support;

rcl_subscription_t subscriber;
rcl_allocator_t allocator;

rcl_node_t node;
rcl_clock_t clock;

rcl_timer_t timer;

rcl_publisher_t Odompublisher;
rcl_publisher_t TFpublisher;
rcl_publisher_t LeftWheelPublisher;
rcl_publisher_t RightWheelPublisher;
rcl_publisher_t motor16Publisher;

geometry_msgs__msg__Vector3 motor16msg;
sensor_msgs__msg__JointState msg;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
std_msgs__msg__Float32 lwpos;
std_msgs__msg__Float32 rwpos;

// this allows for frames to be specified in msgs, as they ask for a specific type. see below
//https://docs.vulcanexus.org/en/iron/rst/microros_documentation/user_api/user_api_utilities.html
const char *str = "odom";
rosidl_runtime_c__String odom_str = micro_ros_string_utilities_init(str);
const char *str1 = "base_link";
rosidl_runtime_c__String base_str = micro_ros_string_utilities_init(str1);

#define LED_PIN 13 // This is for our error loop

// These are part of the error loop
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




void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_publisher_fini(&Odompublisher, &node);
  rcl_publisher_fini(&TFpublisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&LeftWheelPublisher, &node);
  rcl_publisher_fini(&RightWheelPublisher, &node);
  rcl_publisher_fini(&motor16Publisher, &node);

  rcl_timer_fini(&timer);
  rcl_clock_fini(&clock);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
/*End ROS Enttities*/

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrv0
#define ODRV_NODE_ID_STARBOARD 19 // Becuase S is the 19th letter of the alphabet
#define ODRV_NODE_ID_PORT 16      // Because P is the 16th letter of the alphabet

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

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data)
{
  ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t &msg, void *user_data)
{
  ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

ODriveUserData odrv16_user_data;
ODriveUserData odrv19_user_data;

// Called for every message that arrives on the CAN bus

void onCanMessage(const CanMsg &msg)
{
  for (auto odrive : odrives)
  {
    onReceive(msg, *odrive);
  }
}

bool getPower(Get_Powers_msg_t &msg, uint16_t timeout_ms = 10); //already in can.h
Get_Powers_msg_t pwrMsg16;



void setupODrive()
{

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv16.onFeedback(onFeedback, &odrv16_user_data);
  odrv16.onStatus(onHeartbeat, &odrv16_user_data);

  odrv19.onFeedback(onFeedback, &odrv19_user_data);
  odrv19.onStatus(onHeartbeat, &odrv19_user_data);

  while (odrv16_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL or odrv19_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
  {
    odrv16.clearErrors();
    odrv19.clearErrors();
    delay(1);
    odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    for (int i = 0; i < 15; ++i)
    {
      delay(10);
      pumpEvents(can_intf);
    }
    break;
  }
}



Get_Encoder_Estimates_msg_t encoderFeedback16;
Get_Encoder_Estimates_msg_t encoderFeedback19;






void odomUpdate(){ 

  if (odrv16_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL or odrv19_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
  {
    odrv16.clearErrors();
    odrv19.clearErrors();
    odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  
  if(odrv16_user_data.received_feedback == true || odrv19_user_data.received_feedback == true){

  encoderFeedback16 = odrv16_user_data.last_feedback;
  encoderFeedback19 = odrv19_user_data.last_feedback;

  odrv16.request(pwrMsg16 , 1);
  
  motor16msg.x = micros();
  
  if (encoderFeedback16.Vel_Estimate < 0.0001){
    motor16msg.y = 0.0;
    }
  else {
    motor16msg.y = encoderFeedback16.Vel_Estimate;
    }
  
  if (pwrMsg16.Electrical_Power < 0.0001){
      motor16msg.z = 0.0;
    }
  else {
    motor16msg.z = pwrMsg16.Electrical_Power; 
    }

  odrv16_user_data.received_feedback = false;
  odrv19_user_data.received_feedback = false;

  }
}


// im like 78.23% sure that this message does come in as rev/s, so it should be good to go like that.
void subscription_callback(const void *msgin)
{
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
  float vel = msg->velocity.data[0];
  float vel2 = msg->velocity.data[1];
  odrv16.setVelocity(
      vel);
  odrv19.setVelocity(
      -vel2);
}


// this is the publisher timer
// every x seconds, we will generate and publish the odometry data
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{

  encoderFeedback16 = odrv16_user_data.last_feedback;
  encoderFeedback19 = odrv19_user_data.last_feedback;

  odomUpdate();
  
  RCSOFTCHECK(rcl_publish(&Odompublisher, &odom_msg, NULL));
  RCSOFTCHECK(rcl_publish(&TFpublisher, &tf_msg, NULL));
  RCSOFTCHECK(rcl_publish(&LeftWheelPublisher, &lwpos, NULL));
  RCSOFTCHECK(rcl_publish(&RightWheelPublisher, &rwpos, NULL));
  RCSOFTCHECK(rcl_publish(&motor16Publisher, &motor16msg, NULL));

}

void setup()
{
  state = WAITING_AGENT;

  //Wire.begin();
  //Serial.begin(115200);
  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 300000 && !Serial; ++i)
  {
    delay(100);
  }

  set_microros_transports();
  
  // Synchronize time with the agent
  rmw_uros_sync_session(100); //fomerly float timeout_ms = 100;

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // setupIMU();
  if (!setupCan())
  {
    while (true)
      ; // wait indefinitely
  }

  odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  odrv16.setControllerMode(2, 1);
  odrv19.setControllerMode(2, 1);
  
  setupODrive();

}

void loop()
{
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
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  }
}
