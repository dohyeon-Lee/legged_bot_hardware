#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include "group_motor_control.h"
#include "dynamixel_sdk.h"
#include "inverse_kinemetics.h"
#include <unistd.h>

double group_motor_control::fmap(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min;
}

int group_motor_control::angle(double angle_)
{
    return fmap(angle_, -M_PI/2, M_PI/2, 205, 819);
}

int group_motor_control::getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}
void group_motor_control::rest(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, dynamixel::GroupSyncWrite groupSyncWrite)
{
    uint8_t dxl_error = 0;
    packetHandler->write1ByteTxRx(portHandler, DXL_RF0, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RF1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RF2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LF0, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LF1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LF2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LB0, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LB1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LB2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RB0, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RB1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RB2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    packetHandler->write1ByteTxRx(portHandler, DXL_W_LB, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_RB, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_RF, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_LF, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
}


void group_motor_control::setting(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, dynamixel::GroupSyncWrite groupSyncWrite)
{
    uint8_t dxl_error = 0;                         

    packetHandler->write1ByteTxRx(portHandler, DXL_RF0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RF1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RF2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LF0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LF1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LF2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LB0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LB1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LB2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RB0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RB1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RB2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    packetHandler->write1ByteTxRx(portHandler, DXL_W_LB, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_RB, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_RF, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_W_LF, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

}

void group_motor_control::moving(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,dynamixel::GroupSyncWrite groupSyncWrite ,vector<vector<double>> point) //points->point
{                      
    /*uint8_t leg_position[4][3][4];
    IK ik;
    int speed = 1023;
    vector<double> leg1 = ik.IK_RF(point[0][0], point[0][1], point[0][2]);
    vector<double> leg2 = ik.IK_LF(point[1][0], point[1][1], point[1][2]);
    vector<double> leg3 = ik.IK_LB(point[2][0], point[2][1], point[2][2]);
    vector<double> leg4 = ik.IK_RB(point[3][0], point[3][1], point[3][2]);

    int position1[3] = {angle(leg1[0]),angle(-leg1[1]),angle(-leg1[2])};
    int position2[3] = {angle(leg2[0]),angle(-leg2[1]),angle(-leg2[2])};
    int position3[3] = {angle(leg3[0]),angle(-leg3[1]),angle(-leg3[2])};
    int position4[3] = {angle(leg4[0]),angle(-leg4[1]),angle(-leg4[2])};

    int speed1[3] = {speed,speed,speed};
    int speed2[3] = {speed,speed,speed};
    int speed3[3] = {speed,speed,speed};
    int speed4[3] = {speed,speed,speed};

    leg_position[0][0][0] = DXL_LOBYTE(position1[0]);
    leg_position[0][0][1] = DXL_HIBYTE(position1[0]);
    leg_position[0][0][2] = DXL_LOBYTE(speed1[0]);
    leg_position[0][0][3] = DXL_HIBYTE(speed1[0]);

    leg_position[0][1][0] = DXL_LOBYTE(position1[1]);
    leg_position[0][1][1] = DXL_HIBYTE(position1[1]);
    leg_position[0][1][2] = DXL_LOBYTE(speed1[1]);
    leg_position[0][1][3] = DXL_HIBYTE(speed1[1]);

    leg_position[0][2][0] = DXL_LOBYTE(position1[2]);
    leg_position[0][2][1] = DXL_HIBYTE(position1[2]);
    leg_position[0][2][2] = DXL_LOBYTE(speed1[2]);
    leg_position[0][2][3] = DXL_HIBYTE(speed1[2]); //one leg
    

    leg_position[1][0][0] = DXL_LOBYTE(position2[0]);
    leg_position[1][0][1] = DXL_HIBYTE(position2[0]);
    leg_position[1][0][2] = DXL_LOBYTE(speed2[0]);
    leg_position[1][0][3] = DXL_HIBYTE(speed2[0]);

    leg_position[1][1][0] = DXL_LOBYTE(position2[1]);
    leg_position[1][1][1] = DXL_HIBYTE(position2[1]);
    leg_position[1][1][2] = DXL_LOBYTE(speed2[1]);
    leg_position[1][1][3] = DXL_HIBYTE(speed2[1]);

    leg_position[1][2][0] = DXL_LOBYTE(position2[2]);
    leg_position[1][2][1] = DXL_HIBYTE(position2[2]);   
    leg_position[1][2][2] = DXL_LOBYTE(speed2[2]);
    leg_position[1][2][3] = DXL_HIBYTE(speed2[2]);//one leg  


    leg_position[2][0][0] = DXL_LOBYTE(position3[0]);
    leg_position[2][0][1] = DXL_HIBYTE(position3[0]);
    leg_position[2][0][2] = DXL_LOBYTE(speed3[0]);
    leg_position[2][0][3] = DXL_HIBYTE(speed3[0]);

    leg_position[2][1][0] = DXL_LOBYTE(position3[1]);
    leg_position[2][1][1] = DXL_HIBYTE(position3[1]);
    leg_position[2][1][2] = DXL_LOBYTE(speed3[1]);
    leg_position[2][1][3] = DXL_HIBYTE(speed3[1]);

    leg_position[2][2][0] = DXL_LOBYTE(position3[2]);
    leg_position[2][2][1] = DXL_HIBYTE(position3[2]); 
    leg_position[2][2][2] = DXL_LOBYTE(speed3[2]);
    leg_position[2][2][3] = DXL_HIBYTE(speed3[2]);//one leg

    
    leg_position[3][0][0] = DXL_LOBYTE(position4[0]);
    leg_position[3][0][1] = DXL_HIBYTE(position4[0]);
    leg_position[3][0][2] = DXL_LOBYTE(speed4[0]);
    leg_position[3][0][3] = DXL_HIBYTE(speed4[0]);

    leg_position[3][1][0] = DXL_LOBYTE(position4[1]);
    leg_position[3][1][1] = DXL_HIBYTE(position4[1]);
    leg_position[3][1][2] = DXL_LOBYTE(speed4[1]);
    leg_position[3][1][3] = DXL_HIBYTE(speed4[1]);

    leg_position[3][2][0] = DXL_LOBYTE(position4[2]);
    leg_position[3][2][1] = DXL_HIBYTE(position4[2]); 
    leg_position[3][2][2] = DXL_LOBYTE(speed4[2]);
    leg_position[3][2][3] = DXL_HIBYTE(speed4[2]);//one leg

    groupSyncWrite.addParam(DXL_RF0, leg_position[0][0]);
    groupSyncWrite.addParam(DXL_RF1, leg_position[0][1]);
    groupSyncWrite.addParam(DXL_RF2, leg_position[0][2]);
    groupSyncWrite.addParam(DXL_LF0, leg_position[1][0]);
    groupSyncWrite.addParam(DXL_LF1, leg_position[1][1]);
    groupSyncWrite.addParam(DXL_LF2, leg_position[1][2]);
    groupSyncWrite.addParam(DXL_LB0, leg_position[2][0]);
    groupSyncWrite.addParam(DXL_LB1, leg_position[2][1]);
    groupSyncWrite.addParam(DXL_LB2, leg_position[2][2]);
    groupSyncWrite.addParam(DXL_RB0, leg_position[3][0]);
    groupSyncWrite.addParam(DXL_RB1, leg_position[3][1]);
    groupSyncWrite.addParam(DXL_RB2, leg_position[3][2]);       

    groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
    */
    uint8_t leg_position[4][3][6];
    uint8_t wheel[4][6];
    IK ik;
    int speed = 1023;
    int torque = 1023;
    
    vector<double> leg1 = ik.IK_RF(point[0][0], point[0][1], point[0][2]);
    vector<double> leg2 = ik.IK_LF(point[1][0], point[1][1], point[1][2]);
    vector<double> leg3 = ik.IK_LB(point[2][0], point[2][1], point[2][2]);
    vector<double> leg4 = ik.IK_RB(point[3][0], point[3][1], point[3][2]);
    
    int position1[3] = {angle(leg1[0]),angle(-leg1[1]),angle(-leg1[2])};
    int position2[3] = {angle(leg2[0]),angle(-leg2[1]),angle(-leg2[2])};
    int position3[3] = {angle(leg3[0]),angle(-leg3[1]),angle(-leg3[2])};
    int position4[3] = {angle(leg4[0]),angle(-leg4[1]),angle(-leg4[2])};
    
    int speed1[3] = {speed,speed,speed};
    int speed2[3] = {speed,speed,speed};
    int speed3[3] = {speed,speed,speed};
    int speed4[3] = {speed,speed,speed};

    int torque1[3] = {torque,torque,torque};
    int torque2[3] = {torque,torque,torque};
    int torque3[3] = {torque,torque,torque};
    int torque4[3] = {torque,torque,torque};

    int wheelspeed[4] = {0,0,0,0};
    int wheeltorque[4] = {1023,1023,1023,1023};
    
    wheel[0][0] = DXL_LOBYTE(0);
    wheel[0][1] = DXL_HIBYTE(0);
    wheel[0][2] = DXL_LOBYTE(wheelspeed[0]);
    wheel[0][3] = DXL_HIBYTE(wheelspeed[0]);
    wheel[0][4] = DXL_LOBYTE(wheeltorque[0]);
    wheel[0][5] = DXL_HIBYTE(wheeltorque[0]);

    wheel[0][0] = DXL_LOBYTE(0);
    wheel[0][1] = DXL_HIBYTE(0);
    wheel[0][2] = DXL_LOBYTE(wheelspeed[1]);
    wheel[0][3] = DXL_HIBYTE(wheelspeed[1]);
    wheel[0][4] = DXL_LOBYTE(wheeltorque[1]);
    wheel[0][5] = DXL_HIBYTE(wheeltorque[1]);

    wheel[0][0] = DXL_LOBYTE(0);
    wheel[0][1] = DXL_HIBYTE(0);
    wheel[0][2] = DXL_LOBYTE(wheelspeed[2]);
    wheel[0][3] = DXL_HIBYTE(wheelspeed[2]);
    wheel[0][4] = DXL_LOBYTE(wheeltorque[2]);
    wheel[0][5] = DXL_HIBYTE(wheeltorque[2]);

    wheel[0][0] = DXL_LOBYTE(0);
    wheel[0][1] = DXL_HIBYTE(0);
    wheel[0][2] = DXL_LOBYTE(wheelspeed[3]);
    wheel[0][3] = DXL_HIBYTE(wheelspeed[3]);
    wheel[0][4] = DXL_LOBYTE(wheeltorque[3]);
    wheel[0][5] = DXL_HIBYTE(wheeltorque[3]);



    leg_position[0][0][0] = DXL_LOBYTE(position1[0]);
    leg_position[0][0][1] = DXL_HIBYTE(position1[0]);
    leg_position[0][0][2] = DXL_LOBYTE(speed1[0]);
    leg_position[0][0][3] = DXL_HIBYTE(speed1[0]);
    leg_position[0][0][4] = DXL_LOBYTE(torque1[0]);
    leg_position[0][0][5] = DXL_HIBYTE(torque1[0]);

    leg_position[0][1][0] = DXL_LOBYTE(position1[1]);
    leg_position[0][1][1] = DXL_HIBYTE(position1[1]);
    leg_position[0][1][2] = DXL_LOBYTE(speed1[1]);
    leg_position[0][1][3] = DXL_HIBYTE(speed1[1]);
    leg_position[0][1][4] = DXL_LOBYTE(torque1[1]);
    leg_position[0][1][5] = DXL_HIBYTE(torque1[1]);

    leg_position[0][2][0] = DXL_LOBYTE(position1[2]);
    leg_position[0][2][1] = DXL_HIBYTE(position1[2]);
    leg_position[0][2][2] = DXL_LOBYTE(speed1[2]);
    leg_position[0][2][3] = DXL_HIBYTE(speed1[2]); 
    leg_position[0][2][4] = DXL_LOBYTE(torque1[2]);
    leg_position[0][2][5] = DXL_HIBYTE(torque1[2]); //one leg



    leg_position[1][0][0] = DXL_LOBYTE(position2[0]);
    leg_position[1][0][1] = DXL_HIBYTE(position2[0]);
    leg_position[1][0][2] = DXL_LOBYTE(speed2[0]);
    leg_position[1][0][3] = DXL_HIBYTE(speed2[0]);
    leg_position[1][0][4] = DXL_LOBYTE(torque2[0]);
    leg_position[1][0][5] = DXL_HIBYTE(torque2[0]);

    leg_position[1][1][0] = DXL_LOBYTE(position2[1]);
    leg_position[1][1][1] = DXL_HIBYTE(position2[1]);
    leg_position[1][1][2] = DXL_LOBYTE(speed2[1]);
    leg_position[1][1][3] = DXL_HIBYTE(speed2[1]);
    leg_position[1][1][4] = DXL_LOBYTE(torque2[1]);
    leg_position[1][1][5] = DXL_HIBYTE(torque2[1]);

    leg_position[1][2][0] = DXL_LOBYTE(position2[2]);
    leg_position[1][2][1] = DXL_HIBYTE(position2[2]);  
    leg_position[1][2][2] = DXL_LOBYTE(speed2[2]);
    leg_position[1][2][3] = DXL_HIBYTE(speed2[2]);
    leg_position[1][2][4] = DXL_LOBYTE(torque2[2]);
    leg_position[1][2][5] = DXL_HIBYTE(torque2[2]);//one leg   



    leg_position[2][0][0] = DXL_LOBYTE(position3[0]);
    leg_position[2][0][1] = DXL_HIBYTE(position3[0]);
    leg_position[2][0][2] = DXL_LOBYTE(speed3[0]);
    leg_position[2][0][3] = DXL_HIBYTE(speed3[0]);
    leg_position[2][0][4] = DXL_LOBYTE(torque3[0]);
    leg_position[2][0][5] = DXL_HIBYTE(torque3[0]);

    leg_position[2][1][0] = DXL_LOBYTE(position3[1]);
    leg_position[2][1][1] = DXL_HIBYTE(position3[1]);
    leg_position[2][1][2] = DXL_LOBYTE(speed3[1]);
    leg_position[2][1][3] = DXL_HIBYTE(speed3[1]);
    leg_position[2][1][4] = DXL_LOBYTE(torque3[1]);
    leg_position[2][1][5] = DXL_HIBYTE(torque3[1]);

    leg_position[2][2][0] = DXL_LOBYTE(position3[2]);
    leg_position[2][2][1] = DXL_HIBYTE(position3[2]); 
    leg_position[2][2][2] = DXL_LOBYTE(speed3[2]);
    leg_position[2][2][3] = DXL_HIBYTE(speed3[2]);
    leg_position[2][2][4] = DXL_LOBYTE(torque3[2]);
    leg_position[2][2][5] = DXL_HIBYTE(torque3[2]);//one leg

    
    leg_position[3][0][0] = DXL_LOBYTE(position4[0]);
    leg_position[3][0][1] = DXL_HIBYTE(position4[0]);
    leg_position[3][0][2] = DXL_LOBYTE(speed4[0]);
    leg_position[3][0][3] = DXL_HIBYTE(speed4[0]);
    leg_position[3][0][4] = DXL_LOBYTE(torque4[0]);
    leg_position[3][0][5] = DXL_HIBYTE(torque4[0]);

    leg_position[3][1][0] = DXL_LOBYTE(position4[1]);
    leg_position[3][1][1] = DXL_HIBYTE(position4[1]);
    leg_position[3][1][2] = DXL_LOBYTE(speed4[1]);
    leg_position[3][1][3] = DXL_HIBYTE(speed4[1]);
    leg_position[3][1][4] = DXL_LOBYTE(torque4[1]);
    leg_position[3][1][5] = DXL_HIBYTE(torque4[1]);

    leg_position[3][2][0] = DXL_LOBYTE(position4[2]);
    leg_position[3][2][1] = DXL_HIBYTE(position4[2]); 
    leg_position[3][2][2] = DXL_LOBYTE(speed4[2]);
    leg_position[3][2][3] = DXL_HIBYTE(speed4[2]);
    leg_position[3][2][4] = DXL_LOBYTE(torque4[2]);
    leg_position[3][2][5] = DXL_HIBYTE(torque4[2]);//one leg

    groupSyncWrite.addParam(DXL_RF0, leg_position[0][0]);
    groupSyncWrite.addParam(DXL_RF1, leg_position[0][1]);
    groupSyncWrite.addParam(DXL_RF2, leg_position[0][2]);
    groupSyncWrite.addParam(DXL_LF0, leg_position[1][0]);
    groupSyncWrite.addParam(DXL_LF1, leg_position[1][1]);
    groupSyncWrite.addParam(DXL_LF2, leg_position[1][2]);
    groupSyncWrite.addParam(DXL_LB0, leg_position[2][0]);
    groupSyncWrite.addParam(DXL_LB1, leg_position[2][1]);
    groupSyncWrite.addParam(DXL_LB2, leg_position[2][2]);
    groupSyncWrite.addParam(DXL_RB0, leg_position[3][0]);
    groupSyncWrite.addParam(DXL_RB1, leg_position[3][1]);
    groupSyncWrite.addParam(DXL_RB2, leg_position[3][2]);       

    groupSyncWrite.addParam(DXL_W_RF, wheel[0]);
    groupSyncWrite.addParam(DXL_W_RB, wheel[1]);
    groupSyncWrite.addParam(DXL_W_LB, wheel[2]);
    groupSyncWrite.addParam(DXL_W_LF, wheel[3]);       

    groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();

}
void group_motor_control::smooth(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,dynamixel::GroupSyncWrite groupSyncWrite , vector<int> wheelspeed, vector<vector<double>> before_point, vector<vector<double>> current_point)
{
    vector<vector<double>> errorpoint = before_point;
    vector<vector<double>> point = before_point;
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 3; j++)
            errorpoint[i][j] = current_point[i][j] - before_point[i][j];

    double moving_speed = 100.0;
    for(int k = 0; k < int(moving_speed); k++)
    {
        for(int i = 0; i < 4; i++)
            for(int j = 0; j < 3; j++)
                point[i][j] = point[i][j] + double(errorpoint[i][j])/moving_speed;
        moving(portHandler, packetHandler, groupSyncWrite ,point, wheelspeed);
        usleep(100000);//100 millisecond
    }
}

void group_motor_control::moving(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,dynamixel::GroupSyncWrite groupSyncWrite ,vector<vector<double>> point, vector<int> wheelspeed) //points->point
{                      
    uint8_t leg_position[4][3][6];
    uint8_t wheel[4][6];
    IK ik;
    int speed = 1023;
    int torque = 1023;
    
    vector<double> leg1 = ik.IK_RF(point[0][0], point[0][1], point[0][2]);
    vector<double> leg2 = ik.IK_LF(point[1][0], point[1][1], point[1][2]);
    vector<double> leg3 = ik.IK_LB(point[2][0], point[2][1], point[2][2]);
    vector<double> leg4 = ik.IK_RB(point[3][0], point[3][1], point[3][2]);
    
    int position1[3] = {angle(leg1[0]),angle(-leg1[1]),angle(-leg1[2])};
    int position2[3] = {angle(leg2[0]),angle(-leg2[1]),angle(-leg2[2])};
    int position3[3] = {angle(leg3[0]),angle(-leg3[1]),angle(-leg3[2])};
    int position4[3] = {angle(leg4[0]),angle(-leg4[1]),angle(-leg4[2])};
    
    int speed1[3] = {speed,speed,speed};
    int speed2[3] = {speed,speed,speed};
    int speed3[3] = {speed,speed,speed};
    int speed4[3] = {speed,speed,speed};

    int torque1[3] = {torque,torque,torque};
    int torque2[3] = {torque,torque,torque};
    int torque3[3] = {torque,torque,torque};
    int torque4[3] = {torque,torque,torque};

    int wheeltorque[4] = {1023,1023,1023,1023};
    
    wheel[0][0] = DXL_LOBYTE(0);
    wheel[0][1] = DXL_HIBYTE(0);
    wheel[0][2] = DXL_LOBYTE(wheelspeed[0]);
    wheel[0][3] = DXL_HIBYTE(wheelspeed[0]);
    wheel[0][4] = DXL_LOBYTE(wheeltorque[0]);
    wheel[0][5] = DXL_HIBYTE(wheeltorque[0]);

    wheel[1][0] = DXL_LOBYTE(0);
    wheel[1][1] = DXL_HIBYTE(0);
    wheel[1][2] = DXL_LOBYTE(wheelspeed[1]);
    wheel[1][3] = DXL_HIBYTE(wheelspeed[1]);
    wheel[1][4] = DXL_LOBYTE(wheeltorque[1]);
    wheel[1][5] = DXL_HIBYTE(wheeltorque[1]);

    wheel[2][0] = DXL_LOBYTE(0);
    wheel[2][1] = DXL_HIBYTE(0);
    wheel[2][2] = DXL_LOBYTE(wheelspeed[2]+1023);
    wheel[2][3] = DXL_HIBYTE(wheelspeed[2]+1023);
    wheel[2][4] = DXL_LOBYTE(wheeltorque[2]);
    wheel[2][5] = DXL_HIBYTE(wheeltorque[2]);

    wheel[3][0] = DXL_LOBYTE(0);
    wheel[3][1] = DXL_HIBYTE(0);
    wheel[3][2] = DXL_LOBYTE(wheelspeed[3]+1023);
    wheel[3][3] = DXL_HIBYTE(wheelspeed[3]+1023);
    wheel[3][4] = DXL_LOBYTE(wheeltorque[3]);
    wheel[3][5] = DXL_HIBYTE(wheeltorque[3]);



    leg_position[0][0][0] = DXL_LOBYTE(position1[0]);
    leg_position[0][0][1] = DXL_HIBYTE(position1[0]);
    leg_position[0][0][2] = DXL_LOBYTE(speed1[0]);
    leg_position[0][0][3] = DXL_HIBYTE(speed1[0]);
    leg_position[0][0][4] = DXL_LOBYTE(torque1[0]);
    leg_position[0][0][5] = DXL_HIBYTE(torque1[0]);

    leg_position[0][1][0] = DXL_LOBYTE(position1[1]);
    leg_position[0][1][1] = DXL_HIBYTE(position1[1]);
    leg_position[0][1][2] = DXL_LOBYTE(speed1[1]);
    leg_position[0][1][3] = DXL_HIBYTE(speed1[1]);
    leg_position[0][1][4] = DXL_LOBYTE(torque1[1]);
    leg_position[0][1][5] = DXL_HIBYTE(torque1[1]);

    leg_position[0][2][0] = DXL_LOBYTE(position1[2]);
    leg_position[0][2][1] = DXL_HIBYTE(position1[2]);
    leg_position[0][2][2] = DXL_LOBYTE(speed1[2]);
    leg_position[0][2][3] = DXL_HIBYTE(speed1[2]); 
    leg_position[0][2][4] = DXL_LOBYTE(torque1[2]);
    leg_position[0][2][5] = DXL_HIBYTE(torque1[2]); //one leg



    leg_position[1][0][0] = DXL_LOBYTE(position2[0]);
    leg_position[1][0][1] = DXL_HIBYTE(position2[0]);
    leg_position[1][0][2] = DXL_LOBYTE(speed2[0]);
    leg_position[1][0][3] = DXL_HIBYTE(speed2[0]);
    leg_position[1][0][4] = DXL_LOBYTE(torque2[0]);
    leg_position[1][0][5] = DXL_HIBYTE(torque2[0]);

    leg_position[1][1][0] = DXL_LOBYTE(position2[1]);
    leg_position[1][1][1] = DXL_HIBYTE(position2[1]);
    leg_position[1][1][2] = DXL_LOBYTE(speed2[1]);
    leg_position[1][1][3] = DXL_HIBYTE(speed2[1]);
    leg_position[1][1][4] = DXL_LOBYTE(torque2[1]);
    leg_position[1][1][5] = DXL_HIBYTE(torque2[1]);

    leg_position[1][2][0] = DXL_LOBYTE(position2[2]);
    leg_position[1][2][1] = DXL_HIBYTE(position2[2]);  
    leg_position[1][2][2] = DXL_LOBYTE(speed2[2]);
    leg_position[1][2][3] = DXL_HIBYTE(speed2[2]);
    leg_position[1][2][4] = DXL_LOBYTE(torque2[2]);
    leg_position[1][2][5] = DXL_HIBYTE(torque2[2]);//one leg   



    leg_position[2][0][0] = DXL_LOBYTE(position3[0]);
    leg_position[2][0][1] = DXL_HIBYTE(position3[0]);
    leg_position[2][0][2] = DXL_LOBYTE(speed3[0]);
    leg_position[2][0][3] = DXL_HIBYTE(speed3[0]);
    leg_position[2][0][4] = DXL_LOBYTE(torque3[0]);
    leg_position[2][0][5] = DXL_HIBYTE(torque3[0]);

    leg_position[2][1][0] = DXL_LOBYTE(position3[1]);
    leg_position[2][1][1] = DXL_HIBYTE(position3[1]);
    leg_position[2][1][2] = DXL_LOBYTE(speed3[1]);
    leg_position[2][1][3] = DXL_HIBYTE(speed3[1]);
    leg_position[2][1][4] = DXL_LOBYTE(torque3[1]);
    leg_position[2][1][5] = DXL_HIBYTE(torque3[1]);

    leg_position[2][2][0] = DXL_LOBYTE(position3[2]);
    leg_position[2][2][1] = DXL_HIBYTE(position3[2]); 
    leg_position[2][2][2] = DXL_LOBYTE(speed3[2]);
    leg_position[2][2][3] = DXL_HIBYTE(speed3[2]);
    leg_position[2][2][4] = DXL_LOBYTE(torque3[2]);
    leg_position[2][2][5] = DXL_HIBYTE(torque3[2]);//one leg

    
    leg_position[3][0][0] = DXL_LOBYTE(position4[0]);
    leg_position[3][0][1] = DXL_HIBYTE(position4[0]);
    leg_position[3][0][2] = DXL_LOBYTE(speed4[0]);
    leg_position[3][0][3] = DXL_HIBYTE(speed4[0]);
    leg_position[3][0][4] = DXL_LOBYTE(torque4[0]);
    leg_position[3][0][5] = DXL_HIBYTE(torque4[0]);

    leg_position[3][1][0] = DXL_LOBYTE(position4[1]);
    leg_position[3][1][1] = DXL_HIBYTE(position4[1]);
    leg_position[3][1][2] = DXL_LOBYTE(speed4[1]);
    leg_position[3][1][3] = DXL_HIBYTE(speed4[1]);
    leg_position[3][1][4] = DXL_LOBYTE(torque4[1]);
    leg_position[3][1][5] = DXL_HIBYTE(torque4[1]);

    leg_position[3][2][0] = DXL_LOBYTE(position4[2]);
    leg_position[3][2][1] = DXL_HIBYTE(position4[2]); 
    leg_position[3][2][2] = DXL_LOBYTE(speed4[2]);
    leg_position[3][2][3] = DXL_HIBYTE(speed4[2]);
    leg_position[3][2][4] = DXL_LOBYTE(torque4[2]);
    leg_position[3][2][5] = DXL_HIBYTE(torque4[2]);//one leg

    groupSyncWrite.addParam(DXL_RF0, leg_position[0][0]);
    groupSyncWrite.addParam(DXL_RF1, leg_position[0][1]);
    groupSyncWrite.addParam(DXL_RF2, leg_position[0][2]);
    groupSyncWrite.addParam(DXL_LF0, leg_position[1][0]);
    groupSyncWrite.addParam(DXL_LF1, leg_position[1][1]);
    groupSyncWrite.addParam(DXL_LF2, leg_position[1][2]);
    groupSyncWrite.addParam(DXL_LB0, leg_position[2][0]);
    groupSyncWrite.addParam(DXL_LB1, leg_position[2][1]);
    groupSyncWrite.addParam(DXL_LB2, leg_position[2][2]);
    groupSyncWrite.addParam(DXL_RB0, leg_position[3][0]);
    groupSyncWrite.addParam(DXL_RB1, leg_position[3][1]);
    groupSyncWrite.addParam(DXL_RB2, leg_position[3][2]);       

    groupSyncWrite.addParam(DXL_W_RF, wheel[0]);
    groupSyncWrite.addParam(DXL_W_RB, wheel[1]);
    groupSyncWrite.addParam(DXL_W_LB, wheel[2]);
    groupSyncWrite.addParam(DXL_W_LF, wheel[3]);       

    groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();

}
