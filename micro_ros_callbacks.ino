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
// gets called when a message is received on the /reset_odom topic
void flag_callback(const void *msgin)
{
  const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;
  if (msg->data == 1)
  {

    // turn on led
    digitalWrite(LED_PIN, HIGH);
    odrv16.reset(0);
    odrv19.reset(0);
    setupODrive(); // re-setup the odrive after reset
    digitalToggle(LED_PIN);
    delay(100);

    digitalToggle(LED_PIN);
    delay(100);

    digitalToggle(LED_PIN);
    delay(100);

    digitalToggle(LED_PIN);
    delay(100);

    digitalToggle(LED_PIN);
    delay(100);
    delay(2000); // wait for the odrive to reset

    odrv16.clearErrors();
    odrv19.clearErrors();
    odrv19_user_data.last_feedback.Pos_Estimate = 0;
    odrv16_user_data.last_feedback.Pos_Estimate = 0;
  }
}
void position_callback(const void *msgin)
{
  const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;

  float pos = msg->data;
  odrv16.setPosition(pos);
  odrv19.setPosition(-pos);
}

// this is the publisher timer
// this will generate and publish the odometry data

// this is the publisher timer
// this will generate and publish the odometry data
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{

  time_ns_now = rmw_uros_epoch_nanos();
  // print position and velocity for Serial Plotter
  if (odrv16_user_data.received_feedback)
  {
    Get_Encoder_Estimates_msg_t feedback = odrv16_user_data.last_feedback;
    odrv16_user_data.received_feedback = false;
    lwpos = feedback.Pos_Estimate;
    lwpos = lwpos / GEARRATIO;
    lwpos = lwpos * 2 * PI;

    left_wheel_msg.data = lwpos;
    lwpos = 0;

    rcl_publish(&LeftWheelPublisher, &left_wheel_msg, NULL);

    odrv16.request(pwrMsg16, 1);
    left_voltage_msg.x = time_ns_now;
    left_voltage_msg.y = pwrMsg16.Electrical_Power;
    left_voltage_msg.z = feedback.Vel_Estimate;
    rcl_publish(&LeftVoltagePublisher, &left_voltage_msg, NULL);
  }
  if (odrv19_user_data.received_feedback)
  {
    Get_Encoder_Estimates_msg_t feedback = odrv19_user_data.last_feedback;
    odrv19_user_data.received_feedback = false;
    rwpos = feedback.Pos_Estimate;
    rwpos = rwpos * -1;
    rwpos = rwpos / GEARRATIO;
    rwpos = rwpos * 2 * PI;
    right_wheel_msg.data = rwpos;
    rwpos = 0;

    rcl_publish(&RightWheelPublisher, &right_wheel_msg, NULL);
    odrv19.request(pwrMsg19,1);
    right_voltage_msg.x = time_ns_now;
    right_voltage_msg.y = pwrMsg19.Electrical_Power;
    right_voltage_msg.z = feedback.Vel_Estimate;
    rcl_publish(&RightVoltagePublisher, &right_voltage_msg, NULL);
  }
}