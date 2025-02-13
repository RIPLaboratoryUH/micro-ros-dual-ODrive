// this file has the callback functions for micro ros

//gets called when a message is received on the /joint_states topic
void subscription_callback(const void *msgin)
{
  const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
  float vel = msg->velocity.data[0];
  float vel2 = msg->velocity.data[1];
  odrv16.setVelocity(vel);
  odrv19.setVelocity(-vel2);
}
//gets called when a message is received on the /reset_odom topic
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
// this will generate and publish the odometry data
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  rmw_uros_sync_session(100);
  if (rmw_uros_epoch_synchronized())
  {
    time_ns_now = rmw_uros_epoch_nanos();
  }
  // if there is new data from odrive and it has been 10ms since last update
  if ((odrv16_user_data.received_feedback == true && odrv19_user_data.received_feedback == true) && (time_ns_now - time_ns_old > (1000000 / 100))) // try polling more often
  { // do 100000hz (i think)
    odomUpdate();
    RCSOFTCHECK(rcl_publish(&OdomPublisher, &odom_msg, NULL));
  }
  // RCSOFTCHECK(rcl_publish(&TFpublisher, &tf_msg, NULL));
  RCSOFTCHECK(rcl_publish(&LeftWheelPublisher, &lwpos, NULL));
  RCSOFTCHECK(rcl_publish(&RightWheelPublisher, &rwpos, NULL));
}