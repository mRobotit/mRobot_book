#include<iostream>
#include<cmath>


const double A2R = M_PI/180;
const double OW_To_Angle = 1.0 / 180;

inline double angleCaculate(double theta){
    return theta * OW_To_Angle;

}

inline double angle2Radian(double theta)
{
    return theta*A2R;
}

inline double angleJudge(double theta){
    theta = fmod (theta, 2.0);
    if(theta >= 1.0)return 2 - theta;
}

inline double radianNormal(double theta)
{
    theta = fmod(theta, 2*M_PI);
    if(theta >= M_PI){
        theta -= 2*M_PI;
    }
    if(theta < -M_PI){
        theta += 2*M_PI;
    }
    return theta;

}

inline double angleNormal(double theta)
{
    return radianNormal(angle2Radian(theta));
}
