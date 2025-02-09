bool create_entities()
{

  /*Creates all ROS Entities*/
  allocator = rcl_get_default_allocator();
  
  //Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  //Create allocator
  RCCHECK(rcl_ros_clock_init(&clock, &allocator));

  // create subscribers
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/joint_states"));
  
  //create publishers
  /*
  RCCHECK(rclc_publisher_init_default(
    &Odompublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),"/diff_drive_controller/odom"));

  RCCHECK(rclc_publisher_init_default(
    &LeftWheelPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/left_wheel_pos"));

  RCCHECK(rclc_publisher_init_default(
    &RightWheelPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/right_wheel_pos"));

  RCCHECK(rclc_publisher_init_default(
    &TFpublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),"/tf")); 
  */

  RCCHECK(rclc_publisher_init_default(
    &motor16Publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),"motor_16_data"));

  RCCHECK(rclc_publisher_init_default(
      &motor19Publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),"motor_19_data"));

  //Create timer
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(10), //This value used to be called "const unsigned int timer_timeout = 10
      timer_callback));

  // mem allocation
  bool success = micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      &msg,
      conf);
      
  bool success1 = micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
      &tf_msg,
      conf);
    
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  return true;
}
