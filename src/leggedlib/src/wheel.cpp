#include "wheel.h"

vector<int> wheel::forward(int vel)
{
    vector<int> speed = {vel,vel,vel,vel};
    return speed;
}
vector<int> wheel::forward_v2(int vel)
{
    vector<int> speed = {vel/3,vel,vel,vel/3};
    return speed;
}
vector<int> wheel::backward(int vel)
{
    vector<int> speed = {1023+vel,1023+vel,vel-1023,vel-1023};
    return speed;
}
vector<int> wheel::spin_cw(int vel)
{
    vector<int> speed = {vel,vel,vel-1023,vel-1023};
    return speed;

}
vector<int> wheel::spin_ccw(int vel)
{
    vector<int> speed = {1023+vel,1023+vel,vel,vel};
    return speed;
}
vector<int> wheel::roundspin_cw(int vel, double r)
{
    double a = 0.175;
    double b = 0.190;
    double theta = atan2((b/2.),(r-(a/2.))); //r = 0.4일 때 theta = 16.9
    double w = double(vel)/r;
    double v1 = ( (a/2.)/sin(theta) ) * w; //   v1/w = 0.3
    double v2 = sqrt(pow((r + a/2.), 2)+pow(b/2, 2)) * w;// v2/w = 0.496 
    vector<int> speed = {int(v2), int(v2), int(v1), int(v1)};
    return speed;
}
vector<int> wheel::roundspin_ccw(int vel, double r)
{
    double a = 0.175;
    double b = 0.190;
    double theta = atan2((b/2.),(r-(a/2.))); //r = 0.4일 때 theta = 16.9
    double w = double(vel)/r;
    double v1 = ( (a/2.)/sin(theta) ) * w; //   v1/w = 0.3
    double v2 = sqrt(pow((r + a/2.), 2)+pow(b/2, 2)) * w;// v2/w = 0.496 
    vector<int> speed = {int(v1), int(v1), int(v2), int(v2)};
    return speed;
}
vector<int> wheel::roundspin_bcw(int vel, double r)
{
    double a = 0.175;
    double b = 0.190;
    double theta = atan2((b/2.),(r-(a/2.))); //r = 0.4일 때 theta = 16.9
    double w = double(vel)/r;
    double v1 = ( (a/2.)/sin(theta) ) * w; //   v1/w = 0.3
    double v2 = sqrt(pow((r + a/2.), 2)+pow(b/2, 2)) * w;// v2/w = 0.496 
    vector<int> speed = {1023+int(v2), 1023+int(v2), int(v1)-1023, int(v1)-1023};
    return speed;
}
vector<int> wheel::roundspin_bccw(int vel, double r)
{
    double a = 0.175;
    double b = 0.190;
    double theta = atan2((b/2.),(r-(a/2.))); //r = 0.4일 때 theta = 16.9
    double w = double(vel)/r;
    double v1 = ( (a/2.)/sin(theta) ) * w; //   v1/w = 0.3
    double v2 = sqrt(pow((r + a/2.), 2)+pow(b/2, 2)) * w;// v2/w = 0.496 
    vector<int> speed = {1023+int(v1), 1023+int(v1), int(v2)-1023, int(v2)-1023};
    return speed;
}
void wheel::getodometry(double *Vx, double *Vy, double th)
{
    *Vx = key[0]*cos(th);
    *Vy = key[0]*sin(th);
}
vector<int> wheel::movingwheel_slam()
{
    double k = 10000*(10/15.);
    //double k = 10000*(10/15.)*(1/5.); //0.5->0.1
    vector<int> speed;
    if(key[0] == 0.0 && key[1] == 0.0)
        speed = stop();
    if(key[1] == 0.0)
    {
        if(key[0] > 0)
            speed = forward(key[0]*k);
        else if(key[0] < 0)
            speed = backward(-key[0]*k);
    }
    else if(key[1] != 0.0)
    {
        double r = abs(key[0]/key[1]);
        if(key[0] > 0)
        {
            if(key[1] > 0)
            {
                speed = roundspin_ccw(key[0]*k,r);
            }
            else
            {
                speed = roundspin_cw(key[0]*k, r);
            }
        }
        else if(key[0] < 0)
        {
            if(key[1] > 0)
            {
                speed = roundspin_bccw(-key[0]*k,r);
            }
            else
            {
                speed = roundspin_bcw(-key[0]*k, r);
            }
        }
        else if(key[0] == 0.0)
        {
            if(key[1] > 0)
            {
                speed = spin_ccw(key[1]*k*0.2*0.85);
            }
            else
            {
                speed = spin_cw(-key[1]*k*0.2*0.85);
            }
        }

    }
    return speed;
}
vector<int> wheel::stop()
{
    vector<int> speed = {0,0,-1023,-1023};
    return speed;
}
vector<int> wheel::rest(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, dynamixel::GroupSyncWrite groupSyncWrite)
{
    uint8_t dxl_error = 0;
    
    packetHandler->write1ByteTxRx(portHandler, DXL_W_LB, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_RB, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_RF, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_LF, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    vector<int> speed = {0,0,-1023,-1023};
    return speed;
}
vector<int> wheel::on(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, dynamixel::GroupSyncWrite groupSyncWrite)
{
    uint8_t dxl_error = 0;
    packetHandler->write1ByteTxRx(portHandler, DXL_W_LB, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_RB, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_RF, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_LF, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    vector<int> speed = {0,0,-1023,-1023};
    return speed;

}
vector<int> wheel::movingwheel(int vel)
{
    vector<int> speed;
    if(key[0] > 0)
        speed = forward(vel/2);
    else if(key[0] < 0)
        speed = backward(vel/2);
    if(key[1] > 0)
        speed = roundspin_ccw(vel/2, 0.2);
    else if(key[1] < 0)
        speed = roundspin_cw(vel/2, 0.2);
    if(key[0] == 0.0 && key[1] == 0.0)
        speed = stop();
    return speed;
}