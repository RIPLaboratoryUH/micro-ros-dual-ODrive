const void euler_to_quat(float roll, float pitch, float yaw, double *q)
{
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}
// inputs actual velocity for left and right wheel, from odrive data
// outputs linear vel in x dir
float generateLinearVel(float lwvel, float rwvel)
{
    return GEARRATIO * (WHEELRAD / 2) * (wheelc1 * lwvel + wheelc2 * rwvel);
}

// inputs actual velocity for l and r wheel, from odrive data
// outputs angular vel in z dir
float generateAngularVel(float lwvel, float rwvel)
{
    return GEARRATIO * (WHEELRAD / 2) * ((wheelc1 * lwvel - wheelc2 * rwvel) / WHEELSEP);
}
