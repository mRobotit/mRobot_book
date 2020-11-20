//
// Created by wyz on 2020/11/5.
//

#include<ceres/ceres.h>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<vector>
#include<iostream>
#include<random>

//从曲线上观测到的点的数量
const int dataSize = 500;

//拟合直线 y = log(x^2 + 2x + 3)
struct CurveFitting {
    CurveFitting(double x, double y) : _x(x), _y(y) {}

    // 残差的计算
    template<typename T>
    bool operator()(
            const T *const abc, // 模型参数，有3维
            T *residual) const {
        residual[0] = T(_y) - ceres::log(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
        return true;
    }

    const double _x, _y;    // x,y数据
};

int main(int argc,char** argv)
{
    std::default_random_engine randomEngine;
    //均值为0，方差为1.0的高斯分布
    std::normal_distribution<double> normalDistribution(0,1.0);
    std::uniform_real_distribution<double> uniformRealDistribution(-1,1);
    double a_true = 1, b_true = 2 , c_true = 3;

    double a_0 = 0.5 , b_0 =1.5 , c_0 = 5;
    std::vector<double> x_data,y_data;
    for(int i = 0 ; i < dataSize ;++i)
    {
        double x = uniformRealDistribution(randomEngine);

        //为数据添加噪声
        double y = std::log(a_true * x * x + b_true * x + c_true) + normalDistribution(randomEngine);
        x_data.push_back(x);
        y_data.push_back(y);
    }

    double abc[3];
    abc[0] = a_0; abc[1] = b_0; abc[2] = c_0;
    ceres::Problem problem;
    for (int i = 0; i < dataSize; i++) {
        problem.AddResidualBlock(     // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CurveFitting, 1, 3>(
                        new CurveFitting(x_data[i], y_data[i])
                ),
                nullptr,            // 核函数，这里不使用，为空
                abc                 // 待估计参数
        );
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  //增量方程的求解方式，主要针对H矩阵的特性进行优化求解，这儿使用最朴素的方法
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    Eigen::Map<Eigen::Vector3d> result(abc);
    std::cout << result.transpose() << std::endl;
}



sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libsxsparse3 libcholmod3