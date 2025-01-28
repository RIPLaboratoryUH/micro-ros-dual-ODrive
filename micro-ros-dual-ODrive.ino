
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
// make changes as directed in lucas' micro-ros docs
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"

//--End Includes--//

// In this segment we create the objects required for micro_ros
static micro_ros_utilities_memory_conf_t conf = {0};
rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_subscription_t OdomFlagSubscriber;
rcl_publisher_t OdomPublisher;
// rcl_publisher_t TFpublisher;
rcl_publisher_t LeftWheelPublisher;
rcl_publisher_t RightWheelPublisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_clock_t clock;
float time_now, time_old = 0.0;

rcl_timer_t timer; // If we want to run sensor updates at different intervals, we create more than one timer
// these msg are used to publish data
sensor_msgs__msg__JointState msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Float32 odom_flag_msg;
// tf2_msgs__msg__TFMessage tf_msg;

// this allows for frames to be specified in msgs, as they ask for a specific type. see below
// https://docs.vulcanexus.org/en/iron/rst/microros_documentation/user_api/user_api_utilities.html
const char *str = "odom";
rosidl_runtime_c__String odom_str = micro_ros_string_utilities_init(str);
const char *str1 = "base_link";
rosidl_runtime_c__String base_str = micro_ros_string_utilities_init(str1);

#define WHEELRAD .05
#define WHEELSEP .48
#define GEARRATIO 11.1111
#define LED_PIN 13 // built in led

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
  RCCHECK(rclc_subscription_init_default(
      &OdomFlagSubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/reset_odom"));
  RCCHECK(rcl_ros_clock_init(&clock, &allocator));
  RCCHECK(rclc_publisher_init_default(
      &OdomPublisher,
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
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &OdomFlagSubscriber, &odom_flag_msg, &flag_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}
void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_publisher_fini(&OdomPublisher, &node);
  // rcl_publisher_fini(&TFpublisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&LeftWheelPublisher, &node);
  rcl_publisher_fini(&RightWheelPublisher, &node);
  rcl_subscription_fini(&OdomFlagSubscriber, &node);
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
float wheelc1 = 1.0;
float wheelc2 = 1.0;
float lwvel;
float rwvel;
float linvel;
float angvel;
float delta_t;
float delta_s;
float delta_theta;
float x;
float y;
float x_pos;
float y_pos;
float theta_pos;
float lwpos = 0;
float rwpos = 0;
float lwpos_prev = 0;
float rwpos_prev = 0;
float delta_lwpos;
float delta_rwpos;
float Davg;
float Dth;

Get_Encoder_Estimates_msg_t encoderFeedback16;
Get_Encoder_Estimates_msg_t encoderFeedback19;

// generates a quaternion from euler angles
//  we are only concerned with the z axis rotation, which is q
//  const void euler_to_quat(float x, float y, float z, double* q) {
//      float c1 = cos((y*3.14/180.0)/2);
//      float c2 = cos((z*3.14/180.0)/2);
//      float c3 = cos((x*3.14/180.0)/2);

//     float s1 = sin((y*3.14/180.0)/2);
//     float s2 = sin((z*3.14/180.0)/2);
//     float s3 = sin((x*3.14/180.0)/2);

//     q[0] = c1 * c2 * c3 - s1 * s2 * s3;
//     q[1] = s1 * s2 * c3 + c1 * c2 * s3;
//     q[2] = s1 * c2 * c3 + c1 * s2 * s3;
//     q[3] = c1 * s2 * c3 - s1 * c2 * s3;
// }

// const void euler_to_quat(float roll, float pitch, float yaw, double* q) {
//     float cy = cos(yaw * 0.5);
//     float sy = sin(yaw * 0.5);

//     q[0] = cy;
//     q[1] = 0;
//     q[2] = 0;
//     q[3] = sy;
// }

const void euler_to_quat(float roll, float pitch, float yaw, double *q)
{
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);

  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
}
// inputs actual velocity for left and right wheel, from odrive data
// outputs linear vel in x dir
float generateLinearVel(float lwvel, float rwvel)
{
  return GEARRATIO * (WHEELRAD / 2) * (wheelc1 * lwvel + wheelc2 * rwvel);
}

// inputs actual velocity for l and r wheel, from odrive data
// outputs angular vel in z dir
float generateAngularVel(float lwvel, float rwvel)
{
  return GEARRATIO * (WHEELRAD / 2) * ((wheelc1 * lwvel - wheelc2 * rwvel) / WHEELSEP);
}

int64_t time_ns_now;
int64_t time_ns_old;
bool first = true;

void odomUpdate()
{
  if (first == true)
  {
    x_pos = 0;
    y_pos = 0;
    theta_pos = 0;
    lwpos = 0;
    rwpos = 0;

    // send some command to odrive that resets pos of encoders to 0
    odrv16_user_data.last_feedback.Pos_Estimate = 0;
    odrv19_user_data.last_feedback.Pos_Estimate = 0;
    // fill in the message
    odom_msg.header.stamp.sec = 0;
    odom_msg.header.stamp.nanosec = 0;
    odom_msg.header.frame_id = odom_str;
    odom_msg.child_frame_id = base_str;
    odom_msg.pose.pose.position.x = 0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0;

    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = 0;
    odom_msg.pose.pose.orientation.w = 1;

    odom_msg.twist.twist.linear.x = 0;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.z = 0;
    first = false;
  }
  else
  { // regular update
    encoderFeedback16 = odrv16_user_data.last_feedback;
    encoderFeedback19 = odrv19_user_data.last_feedback;
    lwvel = encoderFeedback16.Vel_Estimate;
    rwvel = encoderFeedback19.Vel_Estimate;
    lwpos = encoderFeedback16.Pos_Estimate; // 0 to n where n is #turns, float value. i.e 0.5 is half a turn
    rwpos = encoderFeedback19.Pos_Estimate;
    rwpos = rwpos * -1;
    //  if (lwvel < 0.0001){
    //    lwvel = 0;
    //  }
    //  if (rwvel < 0.0001){
    //    rwvel = 0;
    //  }

    linvel = generateLinearVel(lwvel, rwvel);
    angvel = generateAngularVel(lwvel, rwvel);



    lwpos = (lwpos / GEARRATIO) * WHEELRAD * 2 * 3.14;
    rwpos = (rwpos / GEARRATIO) * WHEELRAD * 2 * 3.14;
    // maybe switch, so delta is taken before conversion to radians
    delta_lwpos = lwpos - lwpos_prev;
    delta_rwpos = rwpos - rwpos_prev;

    // // throw out noise
    // if (abs(delta_lwpos) < 0.001)
    // {
    //   delta_lwpos = 0;
    // }
    // if (abs(delta_rwpos) < 0.001)
    // {
    //   delta_rwpos = 0;
    // }


// Davg = (delta_lwpos + delta_rwpos) / 2;
// Dth = (delta_lwpos - delta_rwpos) / WHEELSEP;
// if(Davg !=0){
//   x = cos(Dth)*Davg;
//   y = -sin(Dth)*Davg;
//   x_pos+=(cos(theta_pos)*x - sin(theta_pos)*y);
//   y_pos+=(sin(theta_pos)*x + cos(theta_pos)*y);
// }
// if(Dth !=0){
//   theta_pos+=Dth;
//   theta_pos = atan2(sin(theta_pos), cos(theta_pos));

// }


//this method has the weird turn thingy

    Davg = (delta_lwpos + delta_rwpos) / 2;
    Dth = (delta_rwpos - delta_lwpos) / WHEELSEP; //
    x = Davg * cos(theta_pos+ (Dth / 2));
    y = Davg * sin(theta_pos + (Dth / 2));
    theta_pos += Dth;
    // does not work if you pass in y,x
    // theta_pos = atan2(sin(theta_pos), cos(theta_pos)); // maybe try comment out 
    // theta_pos = Myatan2(sin(theta_pos), cos(theta_pos));
    //my function has same reuslts as built in atan2



  // if (theta_pos < 0){
  //   theta_pos = theta_pos + (2*3.14);
  // }
  
    // if (theta_pos > 3.14){
    //   theta_pos = theta_pos - (2*3.14);
    // }
    // if (theta_pos < -3.14){
    //   theta_pos = theta_pos + (2*3.14);
    // }
    // delta_t = (time_ns_now - time_ns_old) / 1000000000; // convert to seconds
    // delta_s = linvel * delta_t; // assume no accel
    // delta_theta = angvel * delta_t; // assume no accel
    // x = delta_s * cos(theta_pos);
    // y = delta_s * sin(theta_pos);

    x_pos += x;
    y_pos += y;
    // theta_pos += delta_theta;
    double q[4];
    // i put a negative here as the rotation was not matching in rviz
    euler_to_quat(0, 0, theta_pos, q);

    // fill in the message
    odom_msg.header.stamp.sec = time_ns_now / 1000000000;
    odom_msg.header.stamp.nanosec = time_ns_now % 1000000000;
    odom_msg.header.frame_id = odom_str;
    odom_msg.child_frame_id = base_str;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0;

    odom_msg.pose.pose.orientation.x = (double)q[1];
    odom_msg.pose.pose.orientation.y = (double)q[2];
    odom_msg.pose.pose.orientation.z = (double)q[3];
    odom_msg.pose.pose.orientation.w = (double)q[0];

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

    time_ns_old = time_ns_now;
    lwpos_prev = lwpos;
    rwpos_prev = rwpos;
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
void flag_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  if (msg->data == 1)
  {
    first = true;
    odrv16.clearErrors();
    odrv19.clearErrors();
  }
}

// this is the publisher timer
// every x seconds, we will generate and publish the odometry data
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  rmw_uros_sync_session(100);
  if (rmw_uros_epoch_synchronized())
  {
    time_ns_now = rmw_uros_epoch_nanos();
  }
  // if there is new data from odrive and it has been 10ms since last update
  if ((odrv16_user_data.received_feedback == true && odrv19_user_data.received_feedback == true) && (time_ns_now - time_ns_old > 1000000000 / 100))
  { // do 100hz
    odomUpdate();
    RCSOFTCHECK(rcl_publish(&OdomPublisher, &odom_msg, NULL));
  }
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
