bool create_entities()
{
  
  /*Creates all ROS Entities*/
  allocator = rcl_get_default_allocator();
 //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),  
      "/diff_drive_controller/odom"));

//mem allo
bool success = micro_ros_utilities_create_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
  &msg,
  conf
);
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  /*End ROS Enttities*/
  




  /*Creates all IMU publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher0,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "micro_ros_arduino_imu0_publisher"));
*/

  
  return true;
}
