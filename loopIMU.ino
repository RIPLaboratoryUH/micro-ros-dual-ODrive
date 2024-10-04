void loopIMU()
{
  /*
  tcaselect(0);  
  imu0.dmpGetCurrentFIFOPacket(fifoBuffer);
  imu0.dmpGetQuaternion(&q, fifoBuffer);
    imu0.dmpGetGravity(&gravity, &q);
    imu0.dmpGetAccel(&aa, fifoBuffer);
    imu0.dmpGetGyro(&gg, fifoBuffer);
    msg0.orientation.w = q.w;
    msg0.orientation.x = q.x;
    msg0.orientation.y = q.y;
    msg0.orientation.z = q.z;
    msg0.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg0.angular_velocity.y = (float)aa.y;
    msg0.angular_velocity.z = (float)aa.z;
    msg0.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg0.linear_acceleration.y = (float)gg.y;
    msg0.linear_acceleration.z = (float)gg.z;
    msg0.header.stamp.sec = millis() / 1000;
    RCSOFTCHECK(rcl_publish(&publisher0, &msg0, NULL)); //We just do a temperature check once per loop
    //rcl_publish(&publisher0, &msg0, NULL);

    tcaselect(1);  
    imu1.dmpGetCurrentFIFOPacket(fifoBuffer);
    imu1.dmpGetQuaternion(&q, fifoBuffer);
    imu1.dmpGetGravity(&gravity, &q);
    imu1.dmpGetAccel(&aa, fifoBuffer);
    imu1.dmpGetGyro(&gg, fifoBuffer);
    msg1.orientation.w = q.w;
    msg1.orientation.x = q.x;
    msg1.orientation.y = q.y;
    msg1.orientation.z = q.z;
    msg1.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg1.angular_velocity.y = (float)aa.y;
    msg1.angular_velocity.z = (float)aa.z;
    msg1.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg1.linear_acceleration.y = (float)gg.y;
    msg1.linear_acceleration.z = (float)gg.z;
    msg1.header.stamp.sec = millis() / 1000;
    rcl_publish(&publisher1, &msg1, NULL);

    tcaselect(2);  
    imu2.dmpGetCurrentFIFOPacket(fifoBuffer);
    imu2.dmpGetQuaternion(&q, fifoBuffer);
    imu2.dmpGetGravity(&gravity, &q);
    imu2.dmpGetAccel(&aa, fifoBuffer);
    imu2.dmpGetGyro(&gg, fifoBuffer);
    msg2.orientation.w = q.w;
    msg2.orientation.x = q.x;
    msg2.orientation.y = q.y;
    msg2.orientation.z = q.z;
    msg2.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg2.angular_velocity.y = (float)aa.y;
    msg2.angular_velocity.z = (float)aa.z;
    msg2.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg2.linear_acceleration.y = (float)gg.y;
    msg2.linear_acceleration.z = (float)gg.z;
    msg2.header.stamp.sec = millis() / 1000;
    rcl_publish(&publisher2, &msg2, NULL);

    tcaselect(3);  
    imu3.dmpGetCurrentFIFOPacket(fifoBuffer);
    imu3.dmpGetQuaternion(&q, fifoBuffer);
    imu3.dmpGetGravity(&gravity, &q);
    imu3.dmpGetAccel(&aa, fifoBuffer);
    imu3.dmpGetGyro(&gg, fifoBuffer);
    msg3.orientation.w = q.w;
    msg3.orientation.x = q.x;
    msg3.orientation.y = q.y;
    msg3.orientation.z = q.z;
    msg3.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg3.angular_velocity.y = (float)aa.y;
    msg3.angular_velocity.z = (float)aa.z;
    msg3.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg3.linear_acceleration.y = (float)gg.y;
    msg3.linear_acceleration.z = (float)gg.z;
    msg3.header.stamp.sec = millis() / 1000;
    rcl_publish(&publisher3, &msg3, NULL);

    tcaselect(4);  
    imu4.dmpGetCurrentFIFOPacket(fifoBuffer);
    imu4.dmpGetQuaternion(&q, fifoBuffer);
    imu4.dmpGetGravity(&gravity, &q);
    imu4.dmpGetAccel(&aa, fifoBuffer);
    imu4.dmpGetGyro(&gg, fifoBuffer);
    msg4.orientation.w = q.w;
    msg4.orientation.x = q.x;
    msg4.orientation.y = q.y;
    msg4.orientation.z = q.z;
    msg4.angular_velocity.x = (float)aa.x; // these might be gg not aa
    msg4.angular_velocity.y = (float)aa.y;
    msg4.angular_velocity.z = (float)aa.z;
    msg4.linear_acceleration.x = (float)gg.x; // these might be aa not gg
    msg4.linear_acceleration.y = (float)gg.y;
    msg4.linear_acceleration.z = (float)gg.z;
    msg4.header.stamp.sec = millis() / 1000;
    rcl_publish(&publisher4, &msg4, NULL);
    */
  
}
