#include <iostream>
#include <vector>
#include "inverse_kinemetics.h"
#include <numeric>

using std::vector;
//using std::inner_product;
vector<double> IK::IK_LF(double Yy, double Zz, double Xx)
{
    //about leg length
    double Ly = 0;
    double Lx = 0.03;
    double L1 = 0.095;
    double L2 = 0.095;

    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001

    double Y = Yy - LFx;
    double Z = -(Zz - LFy);
    double X = -Xx;
    double th0 = atan2(-Z,X);
    double x = (X/cos(th0)) - Lx;
    double y = Y-Ly;
    double COS = (pow(x,2)+pow(y,2)-(pow(L1,2) + pow(L2,2)))/(2*L1*L2);
    double th2 = atan2(-sqrt(1-pow(COS,2)),COS);
    double s1 = (-L2*sin(th2)*x + (L1+L2*cos(th2))*y)/(pow((L1+L2*cos(th2)),2) + pow((L2*sin(th2)),2));
    double c1 = ((L1+L2*cos(th2))*x + L2*sin(th2)*y)/(pow((L1+L2*cos(th2)),2) + pow((L1*sin(th2)),2));
    double th1 = atan2(s1, c1);

    vector<double> angles;
    angles.push_back(th0);
    angles.push_back(th1);
    angles.push_back(th2);
    angles.push_back(2);
    return angles;
}
vector<double> IK::IK_RF(double Yy, double Zz, double Xx)
{
    //about leg length
    double Ly = 0;
    double Lx = 0.03;
    double L1 = 0.095;
    double L2 = 0.095;

    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001

    double Y = Yy - RFx;
    double Z = -(Zz - RFy);
    double X = -Xx;
    double th0 = atan2(-Z,X);
    double x = (X/cos(th0)) - Lx;
    double y = Y-Ly;
    double COS = (pow(x,2)+pow(y,2)-(pow(L1,2) + pow(L2,2)))/(2*L1*L2);
    double th2 = atan2(-sqrt(1-pow(COS,2)),COS);
    double s1 = (-L2*sin(th2)*x + (L1+L2*cos(th2))*y)/(pow((L1+L2*cos(th2)),2) + pow((L2*sin(th2)),2));
    double c1 = ((L1+L2*cos(th2))*x + L2*sin(th2)*y)/(pow((L1+L2*cos(th2)),2) + pow((L1*sin(th2)),2));
    double th1 = atan2(s1, c1);

    vector<double> angles;
    angles.push_back(th0);
    angles.push_back(th1);
    angles.push_back(th2);
    angles.push_back(1);
    return angles;

}
vector<double> IK::IK_LB(double Yy, double Zz, double Xx)
{
    //about leg length
    double Ly = 0;
    double Lx = 0.03;
    double L1 = 0.095;
    double L2 = 0.095;

    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001

    double Y = Yy - LBx;
    double Z = -(Zz - LBy);
    double X = -Xx;
    double th0 = atan2(-Z,X);
    double x = (X/cos(th0)) - Lx;
    double y = Y-Ly;
    double COS = (pow(x,2)+pow(y,2)-(pow(L1,2) + pow(L2,2)))/(2*L1*L2);
    double th2 = atan2(-sqrt(1-pow(COS,2)),COS);
    double s1 = (-L2*sin(th2)*x + (L1+L2*cos(th2))*y)/(pow((L1+L2*cos(th2)),2) + pow((L2*sin(th2)),2));
    double c1 = ((L1+L2*cos(th2))*x + L2*sin(th2)*y)/(pow((L1+L2*cos(th2)),2) + pow((L1*sin(th2)),2));
    double th1 = atan2(s1, c1);

    vector<double> angles;
    angles.push_back(th0);
    angles.push_back(th1);
    angles.push_back(th2);
    angles.push_back(3);
    return angles;

}
vector<double> IK::IK_RB(double Yy, double Zz, double Xx)
{
    //about leg length
    double Ly = 0;
    double Lx = 0.03;
    double L1 = 0.095;
    double L2 = 0.095;

    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001

    double Y = Yy - RBx;
    double Z = -(Zz - RBy);
    double X = -Xx;

    double th0 = atan2(-Z,X);
    double x = (X/cos(th0)) - Lx;
    double y = Y-Ly;
    double COS = (pow(x,2)+pow(y,2)-(pow(L1,2) + pow(L2,2)))/(2*L1*L2);
    double th2 = atan2(-sqrt(1-pow(COS,2)),COS);
    double s1 = (-L2*sin(th2)*x + (L1+L2*cos(th2))*y)/(pow((L1+L2*cos(th2)),2) + pow((L2*sin(th2)),2));
    double c1 = ((L1+L2*cos(th2))*x + L2*sin(th2)*y)/(pow((L1+L2*cos(th2)),2) + pow((L1*sin(th2)),2));
    double th1 = atan2(s1, c1);

    vector<double> angles;
    angles.push_back(th0);
    angles.push_back(th1);
    angles.push_back(th2);
    angles.push_back(4);
    return angles;

}
vector<vector<double>> IK::inv_kinenmatics(double points[][3])
{
    vector<vector<double>> angle;
    angle.push_back(IK_RF(points[0][0], points[0][1], points[0][2]));
    angle.push_back(IK_LF(points[1][0], points[1][1], points[1][2]));
    angle.push_back(IK_LB(points[2][0], points[2][1], points[2][2]));
    angle.push_back(IK_RB(points[3][0], points[3][1], points[3][2]));
    return angle;
}
double IK::dot(vector<double> point1, vector<double>point2)
{
    return std::inner_product(point1.begin(), point1.end(), point2.begin(), 0.0); 
}
vector<double> IK::sub(vector<double> v1, vector<double> v2)
{
    vector<double> arr = {v1[0]-v2[0], v1[1]-v2[1], v1[2]-v2[2]}; 
    return  arr;
}

vector<double> IK::add(vector<double> v1, vector<double> v2)
{
    vector<double> arr = {v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2]}; 
    return  arr;
}

vector<double> IK::multiply(vector<double> v1, double v2)
{
    vector<double> arr = {v1[0]*v2, v1[1]*v2, v1[2]*v2}; 
    return  arr;
}

vector<vector<double>> IK::plane(vector<double> normal, double l)
{

    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001    

    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    
    vector<double> p = multiply(n,-l);

    double LFt = dot(sub(p,LF0),n)/dot(n,n);
    vector<double> LF1 = add(LF0, multiply(n,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(n,n);
    vector<double> RF1 = add(RF0, multiply(n,RFt));

    double LBt = dot(sub(p,LB0),n)/dot(n,n);
    vector<double> LB1 = add(LB0, multiply(n,LBt));

    double RBt = dot(sub(p,RB0),n)/dot(n,n);
    vector<double> RB1 = add(RB0, multiply(n,RBt));

    vector<vector<double>> points = {RF1, LF1, LB1, RB1};//발끝지점
    return points;
}

vector<vector<double>> IK::groundslope(vector<double> normal, double l)
{
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));

    vector<vector<double>> points = {RF1, LF1, LB1, RB1};
    return points;
}

vector<double> IK::aaa(double pitch, double roll)
{
    double x = cos(pitch*(M_PI/180.0))*sin(roll*(M_PI/180.0));
    double y = sin(pitch*(M_PI/180.0));
    double z = cos(pitch*(M_PI/180.0))*cos(roll*(M_PI/180.0));
    vector<double> point = {x, y, z};
    return point;
}

vector<double> IK::make_normal_vec(double pitch, double roll)
{
    double x = cos(-pitch*(M_PI/180.0))*sin(-roll*(M_PI/180.0));
    double y = sin(-pitch*(M_PI/180.0));
    double z = cos(-pitch*(M_PI/180.0))*cos(-roll*(M_PI/180.0));
    vector<double> point = {x, y, z};
    return point;
}





vector<vector<double>> IK::upstair1(vector<double> normal, double l, double a)
{
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    
    vector<vector<double>> points = {RF1, LF1, LB1, RB1};
    return points;
}

vector<vector<double>> IK::upstair2(vector<double> normal, double l, double a)
{
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    vector<double> zb = {0,0,b};

    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB2 = add(RB1, zb);

    vector<vector<double>> points = {RF1, LF1, LB1, RB2};
    return points;
}

vector<vector<double>> IK::upstair3(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.08,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    vector<double> zb = {0,0,b};

    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));

    double RBt = dot(sub(sub(p,RB0),delta_x),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB3 = add(add(RB1, zb),delta_x);


    vector<vector<double>> points = {RF1, LF1, LB1, RB3};
    return points;
}
vector<vector<double>> IK::upstair4(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.08,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));

    double RBt = dot(sub(sub(p,RB0),delta_x),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB3 = add(add(RB1, zb),delta_x);
    vector<double> RB4 = add(add(add(RB1, zb),delta_x),zc);

    vector<vector<double>> points = {RF1, LF1, LB1, RB4};
    return points;
}
vector<vector<double>> IK::upstair5(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.08,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};

    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB2 = add(LB1, zb);
    double RBt = dot(sub(sub(p,RB0),delta_x),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB3 = add(add(RB1, zb),delta_x);
    vector<double> RB4 = add(add(add(RB1, zb),delta_x),zc);

    vector<vector<double>> points = {RF1, LF1, LB2, RB4};
    return points;
}

vector<vector<double>> IK::upstair6(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.08,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));

    double LBt = dot(sub(sub(p,LB0),delta_x),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB3 = add(add(LB1, zb),delta_x);
    double RBt = dot(sub(sub(p,RB0),delta_x),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB3 = add(add(RB1, zb),delta_x);
    vector<double> RB4 = add(add(add(RB1, zb),delta_x),zc);


    vector<vector<double>> points = {RF1, LF1, LB3, RB4};
    return points;
}
vector<vector<double>> IK::upstair7(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.08,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));

    double LBt = dot(sub(sub(p,LB0),delta_x),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB3 = add(add(LB1, zb),delta_x);
    vector<double> LB4 = add(add(add(LB1, zb),delta_x),zc);

    double RBt = dot(sub(sub(p,RB0),delta_x),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB3 = add(add(RB1, zb),delta_x);
    vector<double> RB4 = add(add(add(RB1, zb),delta_x),zc);

    vector<vector<double>> points = {RF1, LF1, LB4, RB4};
    return points;
}

vector<vector<double>> IK::upstair8(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.08,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));
    
    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB5 = add(add(LB1, zb),zc);

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB5 = add(add(RB1, zb),zc);

    vector<vector<double>> points = {RF1, LF1, LB5, RB5};
     return points;
}

vector<vector<double>> IK::upstair10(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.08,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));
    vector<double> RF2 = add(RF1, zb);

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB5 = add(add(LB1, zb),zc);

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB5 = add(add(RB1, zb),zc);

    vector<vector<double>> points = {RF2, LF1, LB5, RB5};
    return points;
}

vector<vector<double>> IK::upstair11(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.02,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(sub(p,RF0),delta_x),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));
    vector<double> RF3 = add(add(RF1, zb),delta_x);

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB5 = add(add(LB1, zb),zc);

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB5 = add(add(RB1, zb),zc);

    vector<vector<double>> points = {RF3, LF1, LB5, RB5};
    return points;
}

vector<vector<double>> IK::upstair12(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.02,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));

    double RFt = dot(sub(sub(p,RF0),delta_x),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));
    vector<double> RF3 = add(add(RF1, zb),delta_x);
    vector<double> RF4 = add(add(add(RF1, zb),delta_x),zc);

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB5 = add(add(LB1, zb),zc);

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB5 = add(add(RB1, zb),zc);

    vector<vector<double>> points = {RF4, LF1, LB5, RB5};
    return points;
}

vector<vector<double>> IK::upstair13(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.02,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));
    vector<double> LF2 = add(LF1, zb);

    double RFt = dot(sub(sub(p,RF0),delta_x),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));
    vector<double> RF3 = add(add(RF1, zb),delta_x);
    vector<double> RF4 = add(add(add(RF1, zb),delta_x),zc);

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB5 = add(add(LB1, zb),zc);

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB5 = add(add(RB1, zb),zc);

    vector<vector<double>> points = {RF4, LF2, LB5, RB5};
    return points;
}

vector<vector<double>> IK::upstair14(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.02,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(sub(p,LF0),delta_x),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));
    vector<double> LF3 = add(add(LF1, zb),delta_x);

    double RFt = dot(sub(sub(p,RF0),delta_x),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));
    vector<double> RF3 = add(add(RF1, zb),delta_x);
    vector<double> RF4 = add(add(add(RF1, zb),delta_x),zc);

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB5 = add(add(LB1, zb),zc);

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB5 = add(add(RB1, zb),zc);

    vector<vector<double>> points = {RF4, LF3, LB5, RB5};
    return points;
}

vector<vector<double>> IK::upstair15(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.02,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(sub(p,LF0),delta_x),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));
    vector<double> LF3 = add(add(LF1, zb),delta_x);
    vector<double> LF4 = add(add(add(LF1, zb),delta_x),zc);

    double RFt = dot(sub(sub(p,RF0),delta_x),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));
    vector<double> RF3 = add(add(RF1, zb),delta_x);
    vector<double> RF4 = add(add(add(RF1, zb),delta_x),zc);

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB5 = add(add(LB1, zb),zc);

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB5 = add(add(RB1, zb),zc);

    vector<vector<double>> points = {RF4, LF4, LB5, RB5};
    return points;
}

vector<vector<double>> IK::upstair16(vector<double> normal, double l, double a)
{
    vector<double> delta_x = {-0.02,0,0};
    //about body length
    double LFx = 0.075; //190/2 * 0.001
    double LFy = 0.059;//118/2 * 0.001
    double RFx = 0.075; //190/2 * 0.001
    double RFy = -0.059;//-118/2 * 0.001
    double LBx = -0.095;//-190/2 * 0.001
    double LBy = 0.059;//118/2 * 0.001
    double RBx = -0.095;//-190/2 * 0.001
    double RBy = -0.059;//-118/2 * 0.001
    
    vector<double> LF0 = {LFx, LFy, 0};
    vector<double> RF0 = {RFx, RFy, 0};
    vector<double> LB0 = {LBx, LBy, 0};
    vector<double> RB0 = {RBx, RBy, 0};

    vector<double> n = multiply(normal,(double)(1/(sqrt(dot(normal,normal)))));
    vector<double> z = {0,0,1};
    vector<double> p = multiply(z,-l);

    double b = a/n[2];
    double c = 0.02/n[2];
    vector<double> zb = {0,0,b};
    vector<double> zc = {0,0,-c};
    double LFt = dot(sub(p,LF0),n)/dot(z,n);
    vector<double> LF1 = add(LF0, multiply(z,LFt));
    vector<double> LF5 = add(add(LF1, zb),zc);

    double RFt = dot(sub(p,RF0),n)/dot(z,n);
    vector<double> RF1 = add(RF0, multiply(z,RFt));
    vector<double> RF5 = add(add(RF1, zb),zc);

    double LBt = dot(sub(p,LB0),n)/dot(z,n);
    vector<double> LB1 = add(LB0, multiply(z,LBt));
    vector<double> LB5 = add(add(LB1, zb),zc);

    double RBt = dot(sub(p,RB0),n)/dot(z,n);
    vector<double> RB1 = add(RB0, multiply(z,RBt));
    vector<double> RB5 = add(add(RB1, zb),zc);

    vector<vector<double>> points = {RF5, LF5, LB5, RB5};
    return points;
}

