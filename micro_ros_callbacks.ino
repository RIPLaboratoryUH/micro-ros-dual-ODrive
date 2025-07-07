// this file has the callback functions for micro ros

// gets called when a message is received on the /joint_states topic
void subscription_callback(const void *msgin)
{
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
  float vel = msg->velocity.data[0];
  float vel2 = msg->velocity.data[1];
  odrv16.setVelocity(vel);
  odrv19.setVelocity(-vel2);
}
// gets called when a message is received on the /settings topic
void flag_callback(const void *msgin)
{
  const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;

  if (msg->data == 0) // if the message is 0, set odrives to idle state (sort of estop?)
  {
    odrv16.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv19.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv16.setControllerMode(0, 0);
    odrv19.setControllerMode(0, 0);
  }
  if (msg->data == 1) // if the message is 1, reset odrives and encoder position
  {
    // turn on led
    digitalWrite(LED_PIN, HIGH);
    odrv16.reset(0);
    odrv19.reset(0);
    setupODrive(); // re-setup the odrive after reset
    blink_led(5, 500);
    delay(2000); // wait for the odrive to reset
    odrv16.clearErrors();
    odrv19.clearErrors();
    odrv19_user_data.last_feedback.Pos_Estimate = 0;
    odrv16_user_data.last_feedback.Pos_Estimate = 0;
  }
  if (msg->data == 2) // if the message is 2, set odrives to  position control with trapezoidal trajectory planner
  {
    odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv16.setControllerMode(3, 5);
    odrv19.setControllerMode(3, 5);
    odrv16.setTrapezoidalVelLimit(VEL_LIMIT);
    odrv19.setTrapezoidalVelLimit(VEL_LIMIT);
    odrv16.setTrapezoidalAccelLimits(ACCEL_LIMIT, DECEL_LIMIT);
    odrv19.setTrapezoidalAccelLimits(ACCEL_LIMIT, DECEL_LIMIT);
  }
  if (msg->data == 3) // if the message is 3, set odrives to velocity control with ramped vel input
  {
    odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv16.setControllerMode(2, 2);
    odrv19.setControllerMode(2, 2);
    odrv16.setVelocity(0);
    odrv19.setVelocity(0);
  }
}
void position_callback(const void *msgin)
{
  delay(1000);
  const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;

  float pos = msg->data;
  odrv16.setPosition(pos);
  odrv19.setPosition(-pos);
}

// this is the publisher timer
// this will generate and publish the odometry data

// this is the publisher timer
// this will generate and publish the odometry data
int counter = 0;
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  time_ns_now = rmw_uros_epoch_nanos();
  // time_microseconds_now = time_ns_now / 1000;
  if (odrv16_user_data.received_feedback)
  {
    Get_Encoder_Estimates_msg_t feedback16 = odrv16_user_data.last_feedback;
    odrv16_user_data.received_feedback = false;
    lwpos = feedback16.Pos_Estimate;
    lwpos = lwpos / GEARRATIO;
    lwpos = lwpos * 2 * PI;

    wheel_pos_msg.y = lwpos;
    lwpos = 0;
    counter++;
  }
  if( odrv19_user_data.received_feedback){
    Get_Encoder_Estimates_msg_t feedback19 = odrv19_user_data.last_feedback;
    odrv19_user_data.received_feedback = false;
    rwpos = feedback19.Pos_Estimate;
    rwpos = rwpos * -1;
    rwpos = rwpos / GEARRATIO;
    rwpos = rwpos * 2 * PI;
    wheel_pos_msg.z = rwpos;
    rwpos = 0;
counter++;
  }
  if (counter >=1){

    wheel_pos_msg.x = time_ns_now; // set the timestamp for the wheel positions
    rcl_publish(&WheelPublisher, &wheel_pos_msg, NULL);
  }
  else
  {
    wheel_pos_msg.x = 67; // set the timestamp for the wheel positions
    rcl_publish(&WheelPublisher, &wheel_pos_msg, NULL);
  }
  
  counter = 0; // reset the counter
  //     odrv16.request(pwrMsg16, 1);
  // left_voltage_msg.x = time_microseconds_now;
  // left_voltage_msg.y = feedback16.Pos_Estimate;
  // left_voltage_msg.z = feedback16.Vel_Estimate;
  // rcl_publish(&LeftVoltagePublisher, &left_voltage_msg, NULL);
  //     odrv19.request(pwrMsg19,1);
  // right_voltage_msg.x = time_microseconds_now;
  // right_voltage_msg.y = feedback19.Pos_Estimate;
  // right_voltage_msg.z = feedback19.Vel_Estimate;
  // rcl_publish(&RightVoltagePublisher, &right_voltage_msg, NULL);
}
