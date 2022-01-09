#include "wheel.h"

vector<int> wheel::forward(int vel)
{
    vector<int> speed = {vel,vel,vel,vel};
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
        speed = spin_cw(vel/4);
    else if(key[1] < 0)
        speed = spin_ccw(vel/4);
    if(key[0] == 0.0 && key[1] == 0.0)
        speed = stop();
    return speed;
}