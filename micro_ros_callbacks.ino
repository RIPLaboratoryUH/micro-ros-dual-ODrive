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
    first = true;
    odrv16.clearErrors();
    odrv19.clearErrors();
  }
}

int i = 0;

// this is the publisher timer
// this will generate and publish the odometry data
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  //       digitalToggle(LED_PIN);
  // rmw_uros_sync_session(100);
  // if (rmw_uros_epoch_synchronized())
  // {
  //   time_ns_now = rmw_uros_epoch_nanos();
  // }
  // if there is new data from odrive and it has been 10ms since last update
  // if ((odrv16_user_data.received_feedback == true || odrv19_user_data.received_feedback == true)) // try polling more often
  // { // do 100000hz (i think)
  // //switch led
  //   // RCSOFTCHECK(rcl_publish(&OdomPublisher, &odom_msg, NULL));
  // }
  //  if (odrv19_user_data.received_feedback) {
 
 
 /*
  static bool publish_right = true;

  if (publish_right) {
    rwpos = odrv19_user_data.last_feedback.Pos_Estimate;
    rwpos = rwpos * -1;
    RCSOFTCHECK(rcl_publish(&RightWheelPublisher, &rwpos, NULL));
  } else {
    lwpos = odrv16_user_data.last_feedback.Pos_Estimate;
    RCSOFTCHECK(rcl_publish(&LeftWheelPublisher, &lwpos, NULL));
  }

  publish_right = !publish_right; // Toggle between left and right
*/

 // print position and velocity for Serial Plotter
  if (odrv16_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrv16_user_data.last_feedback;
    odrv16_user_data.received_feedback = false;
    lwpos = feedback.Pos_Estimate;
    // lwpos = lwpos / GEARRATIO;
    left_wheel_msg.data = lwpos;
    lwpos = 0;

    rcl_publish(&LeftWheelPublisher, &left_wheel_msg, NULL);
  }

  // if(odrv16_user_data.received_feedback) {
  //   i++;
  //   left_wheel_msg.data = i;
  //   rcl_publish(&LeftWheelPublisher, &left_wheel_msg, NULL);

  // }



  if (odrv19_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrv19_user_data.last_feedback;
    odrv19_user_data.received_feedback = false;
    rwpos = feedback.Pos_Estimate;
    rwpos = rwpos * -1;
    // rwpos= rwpos / GEARRATIO;
    right_wheel_msg.data = rwpos;
    rwpos = 0;

    rcl_publish(&RightWheelPublisher, &right_wheel_msg, NULL);
  }
   
    // lwpos = odrv16_user_data.last_feedback.Pos_Estimate;
    // lwpos++;
    // RCSOFTCHECK(rcl_publish(&LeftWheelPublisher, &lwpos, NULL));
    // rwpos++;
    // RCSOFTCHECK(rcl_publish(&RightWheelPublisher, &rwpos, NULL));

  // RCSOFTCHECK(rcl_publish(&TFpublisher, &tf_msg, NULL));
}