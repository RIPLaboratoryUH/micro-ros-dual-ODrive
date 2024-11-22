
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
#include <std_msgs/msg/float32.h>




// This is needed for the multiplexor
#include <Wire.h>

// Needed for CanBus / ODrive
#include <Arduino.h>
#include "ODriveCAN.h"

// See https://github.com/tonton81/FlexCAN_T4
// clone https://github.com/tonton81/FlexCAN_T4.git into /src
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"

//--End Includes--//

// In this segment we create the objects required for micro_ros
static micro_ros_utilities_memory_conf_t conf = {0};
rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_publisher_t Odompublisher;
// rcl_publisher_t TFpublisher;
rcl_publisher_t LeftWheelPublisher;
rcl_publisher_t RightWheelPublisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_clock_t clock;
float time_now, time_old = 0.0;

rcl_timer_t timer; // If we want to run sensor updates at different intervals, we create more than one timer
sensor_msgs__msg__JointState msg;

nav_msgs__msg__Odometry odom_msg;
// tf2_msgs__msg__TFMessage tf_msg;
std_msgs__msg__Float32 lwpos;
std_msgs__msg__Float32 rwpos;

// this allows for frames to be specified in msgs, as they ask for a specific type. see below
//https://docs.vulcanexus.org/en/iron/rst/microros_documentation/user_api/user_api_utilities.html
const char *str = "odom";
rosidl_runtime_c__String odom_str = micro_ros_string_utilities_init(str);
const char *str1 = "base_link";
rosidl_runtime_c__String base_str = micro_ros_string_utilities_init(str1);

#define WHEELRAD .05f
#define WHEELSEP .62f
#define GEARRATIO 11.1111f
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

bool create_entities()
{

  /*Creates all ROS Entities*/
  allocator = rcl_get_default_allocator();
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/joint_states"));
  RCCHECK(rcl_ros_clock_init(&clock, &allocator));
  RCCHECK(rclc_publisher_init_default(
      &Odompublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "/diff_drive_controller/odom"));
      RCCHECK(rclc_publisher_init_default(
      &LeftWheelPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/left_wheel_pos"));

      RCCHECK(rclc_publisher_init_default(
      &RightWheelPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/right_wheel_pos"));

// RCCHECK(rclc_publisher_init_default(
//       &TFpublisher,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
//       "/tf"));

  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // mem allo
  bool success = micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      &msg,
      conf);
  // bool success1 = micro_ros_utilities_create_message_memory(
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
  //     &tf_msg,
  //     conf);
    
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}
void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_publisher_fini(&Odompublisher, &node);
  // rcl_publisher_fini(&TFpublisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&LeftWheelPublisher, &node);
  rcl_publisher_fini(&RightWheelPublisher, &node);

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
void setupODrive()
{
  Serial.println("Starting ODriveCAN");

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv16.onFeedback(onFeedback, &odrv16_user_data);
  odrv16.onStatus(onHeartbeat, &odrv16_user_data);

  odrv19.onFeedback(onFeedback, &odrv19_user_data);
  odrv19.onStatus(onHeartbeat, &odrv19_user_data);

  Serial.println("Enabling closed loop control on odrv16...");
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
bool setupCan()
{
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

/*begin odom data generation*/
float c1 = 1.0;
float c2 = 1.0;
float lwvel = 0.0;
float rwvel = 0.0;
float linvel = 0.0;
float angvel = 0.0;
float delta_t = 0.0;
float delta_s = 0.0;
float delta_theta =0.0;
float x = 0.0;
float y = 0.0;
float x_pos = 0.0;
float y_pos = 0.0;
float theta_pos = 0.0;
Get_Encoder_Estimates_msg_t encoderFeedback16;
Get_Encoder_Estimates_msg_t encoderFeedback19;


//generates a quaternion from euler angles
// we are only concerned with the z axis rotation, which is q
double euler_to_quat(float x, float y, float z, double* q) {
    
    double c1 = cos((y*3.14/180.0)/2);
    double c2 = cos((z*3.14/180.0)/2);
    double c3 = cos((x*3.14/180.0)/2);

    double s1 = sin((y*3.14/180.0)/2);
    double s2 = sin((z*3.14/180.0)/2);
    double s3 = sin((x*3.14/180.0)/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
    return c1 * s2 * c3 - s1 * c2 * s3;
}



// inputs actual velocity for left and right wheel, from odrive data
// outputs linear vel in x dir
float generateLinearVel(float lwvel, float rwvel)
{
  return GEARRATIO * (WHEELRAD / 2) * (c1 * lwvel + c2 * rwvel);
}

// inputs actual velocity for l and r wheel, from odrive data
// outputs angular vel in z dir
float generateAngularVel(float lwvel, float rwvel)
{
  return GEARRATIO * (WHEELRAD / 2) * ((c1 * lwvel - c2 * rwvel) / WHEELSEP);
}
int64_t time_ns_now;
int64_t time_ns_old;



void odomUpdate(){ 
    rmw_uros_sync_session(100);
  if (rmw_uros_epoch_synchronized())
  {
 time_ns_now = rmw_uros_epoch_nanos();
  }else{
    return;
  }
if(odrv16_user_data.received_feedback == true || odrv19_user_data.received_feedback == true){

  encoderFeedback16 = odrv16_user_data.last_feedback;
  encoderFeedback19 = odrv19_user_data.last_feedback;
   lwvel = encoderFeedback16.Vel_Estimate; 
   rwvel = encoderFeedback19.Vel_Estimate;
   if (lwvel < 0.01){
     lwvel = 0;
   }
   if (rwvel < 0.01){
     rwvel = 0;
   }

  linvel = generateLinearVel(lwvel, rwvel);
  angvel = generateAngularVel(lwvel, rwvel);

  delta_t = (time_ns_now - time_ns_old) / 1000000000; // convert to seconds
  delta_s = linvel * delta_t; // assume no accel
  delta_theta = angvel * delta_t; // assume no accel
  x = delta_s * cos(theta_pos);
  y = delta_s * sin(theta_pos);

  x_pos += x;
  y_pos += y;
  theta_pos += delta_theta; // check 360 degree wrap
//  double* q;
//   double quatZ = euler_to_quat(0, 0, theta_pos, q);


  // fill in the message
  odom_msg.header.stamp.sec = time_ns_now / 1000000000;
  odom_msg.header.stamp.nanosec = time_ns_now % 1000000000;
  odom_msg.header.frame_id = odom_str;
  odom_msg.child_frame_id = base_str;
  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.position.z = 0;

  // odom_msg.pose.pose.orientation.x = (double)q[1];
  // odom_msg.pose.pose.orientation.y = (double)q[2];
  odom_msg.pose.pose.orientation.z =0;
  // odom_msg.pose.pose.orientation.w =(double) q[0];

  odom_msg.twist.twist.linear.x = linvel;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.z = angvel;

// //tf publisher

  // tf_msg.transforms.data[0].header.frame_id = odom_str;
  // tf_msg.transforms.data[0].child_frame_id = base_str;
  //   tf_msg.transforms.data[0].header.stamp.sec = time_ns_now / 1000000000;
  //   // tf_msg.transforms.data[0].header.stamp.nanosec = time_ns_now;
  //   tf_msg.transforms.data[0].transform.translation.x = x_pos;
  //   tf_msg.transforms.data[0].transform.translation.y = y_pos;
  //   tf_msg.transforms.data[0].transform.translation.z = 0;
  //   tf_msg.transforms.data[0].transform.rotation.z = sin(theta_pos / 2);
    



  odrv16_user_data.received_feedback = false;
  time_ns_old = time_ns_now;
}
}

/*end odom data generation*/

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

  // encoderFeedback16 = odrv16_user_data.last_feedback;
  // encoderFeedback19 = odrv19_user_data.last_feedback;
  // lwpos.data= double(encoderFeedback16.Pos_Estimate); 
  // rwpos.data =  double(encoderFeedback19.Pos_Estimate);

  odomUpdate();
  RCSOFTCHECK(rcl_publish(&Odompublisher, &odom_msg, NULL));
  // RCSOFTCHECK(rcl_publish(&TFpublisher, &tf_msg, NULL));
  RCSOFTCHECK(rcl_publish(&LeftWheelPublisher, &lwpos, NULL));
  RCSOFTCHECK(rcl_publish(&RightWheelPublisher, &rwpos, NULL));
}

void setup()
{
  state = WAITING_AGENT;

  Wire.begin();
  Serial.begin(115200);
  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 300000 && !Serial; ++i)
  {
    delay(100);
  }

  set_microros_transports();
  // Synchronize time with the agent
  float timeout_ms = 100;
   rmw_uros_sync_session(timeout_ms);

  if (rmw_uros_epoch_synchronized())
  {
    // Get time in milliseconds or nanoseconds
    
    time_ns_now = rmw_uros_epoch_nanos();
    time_ns_old = time_ns_now;
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // setupIMU();
  if (!setupCan())
  {
    while (true)
      ; // spin indefinitely
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
