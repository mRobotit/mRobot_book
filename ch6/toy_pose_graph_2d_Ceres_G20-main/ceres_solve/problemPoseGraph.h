//
// Created by 万耀中 on 2020/10/26.
//

#ifndef POSEGRAPHDEMO_PROBLEMPOSEGRAPH_H
#define POSEGRAPHDEMO_PROBLEMPOSEGRAPH_H

#include <iostream>
#include <fstream>
#include <string>

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>

#include "../types.h"


template<typename T>
T normalize_angle(const T& angle_radians) {
    T two_pi = 2.0 * T(M_PI);
    return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

//由x,y,theta构建齐次坐标
template<typename T>
Eigen::Matrix<T,3,3> getEigenT(const T* const x,const T* const y,const T* const theta)
{
    Eigen::Matrix<T,3,3> Trans;
    Trans << ceres::cos(*theta) , ceres::sin(*theta), x,
             -ceres::sin(*theta) , ceres::cos(*theta),y,
             0 , 0, 1;
}

class SE2_COST {
public:
    SE2_COST(const double x_ab, const double y_ab, const double theta_ab,
             std::array<double, 3> decomposed_information)
            : x_ab_(x_ab), y_ab_(y_ab), theta_ab_(theta_ab),
              decomposed_information_(std::move(decomposed_information)) {}
    template<typename T>
    bool operator()(const T* const x_a, const T* const y_a, const T* const theta_a,
                    const T* const x_b, const T* const y_b, const T* const theta_b,
                    T* residuals_ptr) const {

        //getEigenT(x_a,y_a,theta_a);
        T residual_x =  ceres::cos(*theta_a) * (*x_b - *x_a) + ceres::sin(*theta_a) * (*y_b - *y_a) - static_cast<T>(x_ab_);
        T residual_y = -ceres::sin(*theta_a) * (*x_b - *x_a) + ceres::cos(*theta_a) * (*y_b - *y_a) - static_cast<T>(y_ab_);
        T residual_theta = normalize_angle((*theta_b - *theta_a) - static_cast<T>(theta_ab_));

        residuals_ptr[0] = residual_x * static_cast<T>(decomposed_information_.at(0));
        residuals_ptr[1] = residual_y * static_cast<T>(decomposed_information_.at(1));
        residuals_ptr[2] = residual_theta * static_cast<T>(decomposed_information_.at(2));

        return true;
    }

private:
    const double x_ab_;
    const double y_ab_;
    const double theta_ab_;
    const std::array<double, 3> decomposed_information_;
};


class problemPoseGraph
{
public:
    problemPoseGraph(const CONSTRAINTS & constraints, POSE_MAP& poses);
    bool solve();
private:
    ceres::Problem problem_;
};



#endif //POSEGRAPHDEMO_PROBLEMPOSEGRAPH_H
