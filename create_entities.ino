// This file is used to create all the ROS entities required for the micro-ROS node
bool create_entities()
{

    /*Creates all ROS Entities*/
    allocator = rcl_get_default_allocator();
    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rcl_ros_clock_init(&clock, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_ctrl"));
    // RCCHECK(rclc_subscription_init_default(
    //     &OdomFlagSubscriber,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    //     "/reset_odom"));

    // RCCHECK(rclc_publisher_init_default(
    //     &OdomPublisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    //     "/diff_drive_controller/odom"));
    RCCHECK(rclc_publisher_init_default(
        &LeftWheelPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "/left_wheel_pos"));
    // RCCHECK(rclc_publisher_init_best_effort(
    //     &LeftWheelPublisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    //     "/wheel_poses"));

    RCCHECK(rclc_publisher_init_default(
        &RightWheelPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "/right_wheel_pos"));

        // RCCHECK(rclc_publisher_init_default(
        // &JointPublisher,
        // &node,
        // ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        // "/joints"));
    // RCCHECK(rclc_publisher_init_default(
    //       &TFpublisher,
    //       &node,
    //       ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    //       "/tf"));

    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // memory allocation
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
    // RCCHECK(rclc_executor_add_subscription(&executor, &OdomFlagSubscriber, &odom_flag_msg, &flag_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    left_wheel_msg.data = 0;
    right_wheel_msg.data = 0;
    return true;
}
