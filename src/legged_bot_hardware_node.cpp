#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h> 

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>

#include "dynamixel_sdk.h"                                 
#include "motor_control.h"
#include "inverse_kinemetics.h"
#include "group_motor_control.h"
#include "action.h"
#include "torque.h"
#include "SerialClass.h"
#include "wheel.h"

#include <string>
#include <stdexcept>
// C library headers
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <cstdio>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstdlib>
#include <csignal>
#include <sys/file.h>

using std::vector;
using std::string;
using namespace std;
int serial_port;
group_motor_control legged_bot;
torque torque;
wheel W;
IK body; 
action act;
/*about IMU ros data*/
double anglex;
double angley;

//ros callback 함수
void Callback_IMU(const geometry_msgs::Twist::ConstPtr& msg);
void Callback_KEY(const geometry_msgs::Twist::ConstPtr& msg);

//upstair 함수
void upstair(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, dynamixel::GroupSyncWrite groupSyncWrite, double l);
//dynamixel 관련 함수
void signalHandler( int signum );
//입력함수
int getch();
//balancing 함수
vector<vector<double>> balancing(double &anglex, double &angley, double &angle_x, double &angle_y, double l, IK *body, action *act);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "legged_bot_hardware");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, Callback_KEY);
  ros::Subscriber IMU = nh.subscribe("ANGLE",1000, Callback_IMU);
 
  //dynamixel setting
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);

  vector<double> normal = {0,0.3,1}; //for plane & groundslope

  int data = 0;
  double l = 0.18;
  double before_l = l;
  vector<vector<double>> point;
  vector<double> origin = {0,0,1};
  int l_state = 0;
  int sleep_time = 10000;

  vector<vector<double>> pbefore = body.plane(origin, 0.13);
  vector<vector<double>> pcurrent = body.plane(origin, 0.18);
  legged_bot.smooth(portHandler, packetHandler, groupSyncWrite, W.stop(), pbefore, pcurrent);

  /*자율주행*/
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(100.0);

  double Vx = 0;
  double Vy = 0;
  double w = 0;

  double x = 0;
  double y = 0;
  double th = 0;

  while(nh.ok())
  {
    ros::spinOnce();
    current_time = ros::Time::now();
    point = body.plane(origin, l);
    double dt = (current_time - last_time).toSec();
    legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point, W.movingwheel_slam());
    w = W.key[1];
    th = th + w*dt*0.3*1.2;
    W.getodometry(&Vx, &Vy, th);
    x = x + Vx*dt;
    y = y + Vy*dt;
    ROS_INFO("%5f, %5f, %5f",x, y, th*(180./M_PI));
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = Vx;
    odom.twist.twist.linear.y = Vy;
    odom.twist.twist.angular.z = w;

    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();

  }
  ros::spin();

  /*just keyboard control*/

  // /*balancing mode setting*/
  // legged_bot.setting(portHandler, packetHandler, groupSyncWrite);
  
  // double angle_x;
  // double angle_y;

  // //PID control
  // double PID_x;
  // double PID_y;
  // act.groundslopePID_pre_setting(); 

  // while(true)
  // {
  //   vector<double> origin = {0,0,1};
  //   point = balancing(anglex, angley, angle_x, angle_y, l, &body, &act);
  //   if(W.key[1] != 0.0)
  //   {
  //     l = 0.14;
  //     if(l != before_l)
  //     {
  //       l_state = 1;
  //       vector<vector<double>> pointbefore = body.plane(origin, before_l);
  //       vector<vector<double>> pointcurrent = body.plane(origin, l);
  //       legged_bot.smooth(portHandler, packetHandler, groupSyncWrite, W.stop(), pointbefore, pointcurrent);
  //     }
  //     else
  //       l_state = 0;
  //   }
  //   else
  //   {
  //     l = 0.18;
  //     if(l != before_l)
  //     {
  //       l_state = 1;
  //       vector<vector<double>> pointbefore = body.plane(origin, before_l);
  //       vector<vector<double>> pointcurrent = body.plane(origin, l);
  //       legged_bot.smooth(portHandler, packetHandler, groupSyncWrite, W.movingwheel(1023), pointbefore, pointcurrent);
  //     }
  //     else
  //       l_state = 0;
  //   }
  //   if(W.key[2] == 0.0)
  //   {
  //     if(l_state == 0)
  //     {//point = body.upstair1(origin,l, 0.055);
        
  //       legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point, W.movingwheel(1023));
  //     }
  //   }
  //   else
  //   {
  //     upstair(portHandler, packetHandler, groupSyncWrite,l);
  //   }
  //   usleep(sleep_time);
  //   ros::spinOnce();
  //   before_l = l;
  // }




  
//   else if (getch() == 51) //shacking mode
//   {
//     legged_bot.setting(portHandler, packetHandler, groupSyncWrite);
//     l = 0.13;
//     sleep_time = 1000;
//     while(1)
//     {   
//       for(double i = 0; i <= 0.2; i = i+0.001)
//       {
//         normal[0] = 0.2;
//         normal[1] = i;

//         point = body.plane(normal,l);
//         legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
//         usleep(sleep_time);
//       }
//       for(double i = 0; i <= 0.2; i = i+0.001)
//       {
//         normal[0] = 0.2-i;
//         normal[1] = 0.2;

//         point = body.plane(normal,l);
//         legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
//         usleep(sleep_time);
//       }
//       for(double i = 0; i <= 0.2; i = i+0.001)
//       {
//         normal[0] = -i;
//         normal[1] = 0.2;

//         point = body.plane(normal,l);
//         legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
//         usleep(sleep_time);
//       }
//       for(double i = 0; i <= 0.2; i = i+0.001)
//       {
//         normal[0] = -0.2;
//         normal[1] = 0.2-i;

//         point = body.plane(normal,l);
//         legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
//         usleep(sleep_time);
//       }
//       for(double i = 0; i <= 0.2; i = i+0.001)
//       {
//         normal[0] = -0.2;
//         normal[1] = -i;

//         point = body.plane(normal,l);
//         legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
//         usleep(sleep_time);
//       }
//       for(double i = 0; i <= 0.2; i = i+0.001)
//       {
//         normal[0] = -0.2+i;
//         normal[1] = -0.2;

//         point = body.plane(normal,l);
//         legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
//         usleep(sleep_time);
//       }
//       for(double i = 0; i <= 0.2; i = i+0.001)
//       {
//         normal[0] = i;
//         normal[1] = -0.2;

//         point = body.plane(normal,l);
//         legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
//         usleep(sleep_time);
//       }
//       for(double i = 0; i <= 0.2; i = i+0.001)
//       {
//         normal[0] = 0.2;
//         normal[1] = -0.2+i;

//         point = body.plane(normal,l);
//         legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
//         usleep(sleep_time);
//       }

//     }

//   }

  return 0;
}

void upstair(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, dynamixel::GroupSyncWrite groupSyncWrite,  double l)
{
  int sec = 10000;
  vector<double> origin = {0,0,1};
  vector<double> angle = body.make_normal_vec(10, 10);
  wheel W;
  int vel = 500;
  //수평유지
  vector<vector<double>> point0 = body.upstair1(origin,l, 0.055);
  legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point0);
  usleep(sec);

  /*RB*/
  //몸 기울이기
  vector<vector<double>> point1 = body.upstair1(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point0, point1);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 올리기
  vector<vector<double>> point2 = body.upstair2(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point1, point2);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 옮기기
  vector<vector<double>> point3 = body.upstair3(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point2, point3);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 내리기
  vector<vector<double>> point4 = body.upstair4(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point3, point4);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //몸 기울이기(수평유지)
  vector<vector<double>> point4_5 = body.upstair4(origin,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point4, point4_5);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);


  /*LB*/
  angle = body.make_normal_vec(-10, 10); //다른 방향으로 기울이기
  //몸 기울이기(다른방향)
  vector<vector<double>> point4_55 = body.upstair4(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point4_5, point4_55);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 올리기
  vector<vector<double>> point5 = body.upstair5(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point4_55, point5);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 옮기기
  vector<vector<double>> point6 = body.upstair6(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point5, point6);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 내리기
  vector<vector<double>> point7 = body.upstair7(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point6, point7);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //몸 기울이기(수평유지) & 전진이동
  vector<vector<double>> point7_5 = body.upstair7(origin,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.forward(vel), point7, point7_5);//W.forward(vel)
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(1000000);

  //앞다리 자세복구
  vector<vector<double>> point8 = body.upstair8(origin,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point7_5, point8);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);

  /*RF*/
  angle = body.make_normal_vec(10, -10); //다른 방향으로 기울이기

  //기울이기
  vector<vector<double>> point9 = body.upstair8(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point8, point9);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 올리기
  vector<vector<double>> point10 = body.upstair10(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point9, point10);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 옮기기
  vector<vector<double>> point11 = body.upstair11(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point10, point11);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 내리기
  vector<vector<double>> point12 = body.upstair12(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point11, point12);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //몸 기울이기(수평유지)
  vector<vector<double>> point12_5 = body.upstair12(origin,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point12, point12_5);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);

  /*LF*/
  angle = body.make_normal_vec(-16, -16); //다른 방향으로 기울이기

  //기울이기
  vector<vector<double>> point12_55 = body.upstair12(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point12_5, point12_55);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 올리기
  vector<vector<double>> point13 = body.upstair13(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point12_55, point13);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 옮기기
  vector<vector<double>> point14 = body.upstair14(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point13, point14);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //다리 내리기
  vector<vector<double>> point15 = body.upstair15(angle,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point14, point15);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //몸 기울이기(수평유지) & 전진
  vector<vector<double>> point15_5 = body.upstair15(origin,l+0.02, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.forward_v2(vel/2), point15, point15_5);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);
  //뒷다리 자세복구 & 전진
  vector<vector<double>> point16 = body.upstair16(origin,l, 0.055);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.forward_v2(vel/2), point15_5, point16);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
  usleep(sec);

  //제대로 서기
  vector<vector<double>> point17 = body.plane(origin,l);
  legged_bot.smooth(portHandler, packetHandler,groupSyncWrite, W.stop(), point16, point17);
  //legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
}
vector<vector<double>> balancing(double &anglex, double &angley, double &angle_x, double &angle_y, double l, IK *body, action *act)
{
  int limit_angle = 15;
  if(anglex != -1 && angley != -1 && anglex <= 180 && anglex >= -180 && angley <= 180 && angley >= -180)
  {
    angle_x = anglex;
    angle_y = angley;
  }

  std::cout << angle_x <<","<< angle_y <<std::endl;
  vector<double> goal = {0,0};
  vector<double> PID = act->groundslopePID(goal, angle_x, angle_y);
  
  if(PID[0] > 15)
    PID[0] = 15;
  else if(PID[0] < -15)
    PID[0] = -15;
  else 
    PID[0] = PID[0];

  if(PID[1] > 15)
    PID[1] = 15;
  else if(PID[1] < -15)
    PID[1] = -15;
  else 
    PID[1] = PID[1];

  vector<double> angle = body->aaa(PID[0], PID[1]);
  vector<vector<double>> point = body->groundslope(angle,l);
  return point;
}
void signalHandler( int signum ) 
{
  close(serial_port);
  exit(signum);  
}

int getch()
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

void Callback_IMU(const geometry_msgs::Twist::ConstPtr& msg)
{
  anglex = msg->linear.x;
  angley = msg->linear.y;
}
void Callback_KEY(const geometry_msgs::Twist::ConstPtr& msg)
{
  W.key[0] = msg->linear.x;
  W.key[1] = msg->angular.z;
  W.key[2] = msg->linear.y;
  //ROS_INFO("robot vx value: %f", W.key[0]);
  //ROS_INFO("robt w value : %f", W.key[1]);
}