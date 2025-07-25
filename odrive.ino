// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
  digitalToggle(LED_PIN); // Toggle the LED to indicate feedback received
}

//called once to set up the ODrive in setup()
void setupODrive()
{
//   while (!odrv19_user_data.received_heartbeat) {
//     digitalToggle(LED_PIN);
//     pumpEvents(can_intf);
//     delay(100);
//   }
    
    // Register callbacks for the heartbeat and encoder feedback messages
    odrv16.onFeedback(onFeedback, &odrv16_user_data);
    odrv16.onStatus(onHeartbeat, &odrv16_user_data);

    odrv19.onFeedback(onFeedback, &odrv19_user_data);
    odrv19.onStatus(onHeartbeat, &odrv19_user_data);

    //sets the controller mode to position control with  an online trapezoidal trajectory planner
    //https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode

    //sets the controller mode to velocity control with ramped vel input
    // odrv16.setControllerMode(2,2);
    // odrv19.setControllerMode(2,2);
blink_led(3, 100); // blink the LED 3 times to indicate setup is starting
    while(odrv16_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL or odrv19_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
    {

        odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv16.setControllerMode(3,5);
    odrv19.setControllerMode(3,5);
odrv16.setTrapezoidalVelLimit(VEL_LIMIT);
    odrv19.setTrapezoidalVelLimit(VEL_LIMIT);
    odrv16.setTrapezoidalAccelLimits(ACCEL_LIMIT, DECEL_LIMIT);
    odrv19.setTrapezoidalAccelLimits(ACCEL_LIMIT, DECEL_LIMIT);

        uint32_t t_start = millis();
        const uint32_t t_poll_duration = 150;
        while (millis() < t_start + t_poll_duration) {
            pumpEvents(can_intf);
        }

          while (!odrv16_user_data.received_feedback) {
        pumpEvents(can_intf);
    }
    
    
    }
blink_led(5, 100); // blink the LED 3 times to indicate setup is starting


}
