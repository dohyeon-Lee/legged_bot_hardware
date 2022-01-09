#ifndef __INVERSE_KINEMETICS_H__
#define __INVERSE_KINEMETICS_H__

#include <iostream>
#include <cmath>
#include <vector>
class IK
{
    public:
        std::vector<double> IK_LF(double Yy, double Zz, double Xx);
        std::vector<double> IK_RF(double Yy, double Zz, double Xx);
        std::vector<double> IK_LB(double Yy, double Zz, double Xx);
        std::vector<double> IK_RB(double Yy, double Zz, double Xx);
        std::vector<std::vector<double>> inv_kinenmatics(double points[][3]);
        double dot(std::vector<double> point1, std::vector<double>point2);
        std::vector<std::vector<double>> plane(std::vector<double> normal, double l);
        std::vector<std::vector<double>> groundslope(std::vector<double> normal, double l);
        std::vector<double> aaa(double roll, double pitch);
        std::vector<double> make_normal_vec(double roll, double pitch);

        std::vector<double> sub(std::vector<double> v1, std::vector<double> v2);
        std::vector<double> add(std::vector<double> v1, std::vector<double> v2);
        std::vector<double> multiply(std::vector<double> v1, double v2);

        std::vector<std::vector<double>> upstair1(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair2(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair3(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair4(std::vector<double> normal, double l, double a);//RB
        std::vector<std::vector<double>> upstair5(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair6(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair7(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair8(std::vector<double> normal, double l, double a);//LB
        std::vector<std::vector<double>> upstair10(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair11(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair12(std::vector<double> normal, double l, double a);//RF
        std::vector<std::vector<double>> upstair13(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair14(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair15(std::vector<double> normal, double l, double a);
        std::vector<std::vector<double>> upstair16(std::vector<double> normal, double l, double a);//LF
        


};
#endif