// can bus setup and message handling

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg &msg)
{

  for (auto odrive : odrives)
  {

    onReceive(msg, *odrive);
  }
}


bool setupCan()
{
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}