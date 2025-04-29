//kills entites for micro ros clean up   
void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  // rcl_publisher_fini(&OdomPublisher, &node);
  // rcl_publisher_fini(&TFpublisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&LeftWheelPublisher, &node);
  rcl_publisher_fini(&RightWheelPublisher, &node);
  // rcl_publisher_fini(&JointPublisher, &node);
  // rcl_subscription_fini(&OdomFlagSubscriber, &node);
  rcl_timer_fini(&timer);
  rcl_clock_fini(&clock);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}