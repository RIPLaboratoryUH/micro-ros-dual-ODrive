
// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data)
{
    ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
    odrv_user_data->last_feedback = msg;
    odrv_user_data->received_feedback = true;
}

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t &msg, void *user_data)
{
    ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
    odrv_user_data->last_heartbeat = msg;
    odrv_user_data->received_heartbeat = true;
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

    // sets the controller mode to velocity control with ramped vel input
    odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv16.setControllerMode(2,2);
    odrv19.setControllerMode(2,2);

        // digitalToggle(LED_PIN);
        // delay(100);

        // digitalToggle(LED_PIN);
        // delay(100);
        
        // digitalToggle(LED_PIN);
        // delay(100);
        
        // digitalToggle(LED_PIN);
        // delay(100);
        
        // digitalToggle(LED_PIN);
        // delay(100);
    //sets the controller mode to velocity control with direct input
    // odrv16.setControllerMode(2,1);
    // odrv19.setControllerMode(2,1);

    while(odrv16_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL or odrv19_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
    {

        odrv16.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrv19.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrv16.setControllerMode(2,2);
        odrv19.setControllerMode(2,2);

        for (int i = 0; i < 15; ++i)
        {
            delay(10);
            pumpEvents(can_intf);
        }
        break;
    }
}
