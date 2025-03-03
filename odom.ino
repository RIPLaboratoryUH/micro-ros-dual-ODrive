// this function is called by the timer_callback function
// generates the odometry data
void odomUpdate()
{ 
    if (first == true)
    {
        x_pos = 0;
        y_pos = 0;
        theta_pos = 0;
        lwpos = 0;
        rwpos = 0;

        // send some command to odrive that resets pos of encoders to 0
        odrv16_user_data.last_feedback.Pos_Estimate = 0;
        odrv19_user_data.last_feedback.Pos_Estimate = 0;
        // fill in the message
        odom_msg.header.stamp.sec = 0;
        odom_msg.header.stamp.nanosec = 0;
        odom_msg.header.frame_id = odom_str;
        odom_msg.child_frame_id = base_str;
        odom_msg.pose.pose.position.x = 0;
        odom_msg.pose.pose.position.y = 0;
        odom_msg.pose.pose.position.z = 0;

        odom_msg.pose.pose.orientation.x = 0;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = 0;
        odom_msg.pose.pose.orientation.w = 1;

        odom_msg.twist.twist.linear.x = 0;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.linear.z = 0;
        odom_msg.twist.twist.angular.z = 0;
        first = false;
    }
    else
    { // regular update
        encoderFeedback16 = odrv16_user_data.last_feedback;
        encoderFeedback19 = odrv19_user_data.last_feedback;
        lwvel = encoderFeedback16.Vel_Estimate;
        rwvel = encoderFeedback19.Vel_Estimate;
        lwpos = encoderFeedback16.Pos_Estimate; // 0 to n where n is #turns, float value. i.e 0.5 is half a turn
        rwpos = encoderFeedback19.Pos_Estimate;
        rwpos = rwpos * -1;


        linvel = generateLinearVel(lwvel, rwvel);
        angvel = generateAngularVel(lwvel, rwvel);

        lwpos = (lwpos / GEARRATIO) * WHEELRAD * 2 * 3.14;
        rwpos = (rwpos / GEARRATIO) * WHEELRAD * 2 * 3.14;
        // maybe switch, so delta is taken before conversion to radians
        delta_lwpos = lwpos - lwpos_prev;
        delta_rwpos = rwpos - rwpos_prev;

        // // throw out noise
        // if (abs(delta_lwpos) < 0.001)
        // {
        //   delta_lwpos = 0;
        // }
        // if (abs(delta_rwpos) < 0.001)
        // {
        //   delta_rwpos = 0;
        // }

        Davg = (delta_lwpos + delta_rwpos) / 2;
        Dth = (delta_rwpos - delta_lwpos) / WHEELSEP; //
        x = Davg * cos(theta_pos + (Dth / 2));
        y = Davg * sin(theta_pos + (Dth / 2));
        theta_pos += Dth;
        // theta_pos = atan2(sin(theta_pos), cos(theta_pos));


        // if (theta_pos < 0){
        //   theta_pos = theta_pos + (2*3.14);
        // }

        // if (theta_pos > 3.14){
        //   theta_pos = theta_pos - (2*3.14);
        // }
        // if (theta_pos < -3.14){
        //   theta_pos = theta_pos + (2*3.14);
        // }


        x_pos += x;
        y_pos += y;

        double q[4];
        // i put a negative here as the rotation was not matching in rviz
        euler_to_quat(0, 0, theta_pos, q);

        // fill in the message
        odom_msg.header.stamp.sec = time_ns_now / 1000000000;
        odom_msg.header.stamp.nanosec = time_ns_now % 1000000000;
        odom_msg.header.frame_id = odom_str;
        odom_msg.child_frame_id = base_str;
        odom_msg.pose.pose.position.x = x_pos;
        odom_msg.pose.pose.position.y = y_pos;
        odom_msg.pose.pose.position.z = 0;

        odom_msg.pose.pose.orientation.x = (double)q[1];
        odom_msg.pose.pose.orientation.y = (double)q[2];
        odom_msg.pose.pose.orientation.z = (double)q[3];
        odom_msg.pose.pose.orientation.w = (double)q[0];

        odom_msg.twist.twist.linear.x = linvel;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.linear.z = 0;
        odom_msg.twist.twist.angular.z = angvel;

        // //tf publisher

        // tf_msg.transforms.data[0].header.frame_id = odom_str;
        // tf_msg.transforms.data[0].child_frame_id = base_str;
        //   tf_msg.transforms.data[0].header.stamp.sec = time_ns_now / 1000000000;
        //   // tf_msg.transforms.data[0].header.stamp.nanosec = time_ns_now;
        //   tf_msg.transforms.data[0].transform.translation.x = x_pos;
        //   tf_msg.transforms.data[0].transform.translation.y = y_pos;
        //   tf_msg.transforms.data[0].transform.translation.z = 0;
        //   tf_msg.transforms.data[0].transform.rotation.z = sin(theta_pos / 2);

        time_ns_old = time_ns_now;
        lwpos_prev = lwpos;
        rwpos_prev = rwpos;
    }
}