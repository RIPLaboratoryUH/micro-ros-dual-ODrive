void loopODrive() {
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

  float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();
  
  float phase = t * (TWO_PI / SINE_PERIOD);

  odrv16.setPosition(
    sin(phase), // position
    cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
  );

  // print position and velocity for Serial Plotter
  if (odrv16_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrv16_user_data.last_feedback;
    odrv16_user_data.received_feedback = false;
    Serial.print("odrv0-pos:");
    Serial.print(feedback.Pos_Estimate);
    Serial.print(",");
    Serial.print("odrv0-vel:");
    Serial.println(feedback.Vel_Estimate);
  }
}
