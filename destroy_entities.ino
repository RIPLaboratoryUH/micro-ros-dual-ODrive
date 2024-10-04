void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  /*
  //Make sure to destroy each publisher
  rcl_publisher_fini(&publisher0, &node);
  rcl_publisher_fini(&publisher1, &node);
  rcl_publisher_fini(&publisher2, &node);
  rcl_publisher_fini(&publisher3, &node);
  rcl_publisher_fini(&publisher4, &node);
  //End IMU Publishers
  */
  
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
