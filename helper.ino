const void euler_to_quat(double roll, double pitch, double yaw, double *q)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}
// inputs actual velocity for left and right wheel, from odrive data
// outputs linear vel in x dir
double generateLinearVel(double lwvel, double rwvel)
{
    return GEARRATIO * (WHEELRAD / 2) * (wheelc1 * lwvel + wheelc2 * rwvel);
}

// inputs actual velocity for l and r wheel, from odrive data
// outputs angular vel in z dir
double generateAngularVel(double lwvel, double rwvel)
{
    return GEARRATIO * (WHEELRAD / 2) * ((wheelc1 * lwvel - wheelc2 * rwvel) / WHEELSEP);
}
