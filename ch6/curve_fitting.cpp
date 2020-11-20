//
// Created by wyz on 2020/11/5.
//
#include<Eigen/Core>
#include<Eigen/Dense>
#include<vector>
#include<iostream>
#include<random>

//从曲线上观测到的点的数量
const int dataSize = 500;

//拟合直线 y = exp(x^2 + 2x + 3)
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
        double y = std::log(a_true*x*x+ b_true*x +  c_true) + normalDistribution(randomEngine);
        x_data.push_back(x);
        y_data.push_back(y);
    }


    int max_iteration = 100;

    Eigen::Vector3d result{a_0,b_0,c_0};
    double lastCost = 0;

    for(int iter = 0 ; iter < max_iteration ; ++iter)
    {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        double cost = 0;

        for(int i = 0 ; i < dataSize; ++i)
        {
            double x_i = x_data[i] , y_i = y_data[i];
            Eigen::Vector3d J;
            double cur_map = (result[0]*x_i*x_i + result[1]*x_i + result[2]);

            double error = y_i - std::log(cur_map);
            J[0] = -x_i*x_i/cur_map; J[1] = -x_i/cur_map; J[2] = -1.0/cur_map;
            H += J*J.transpose();
            b += -error*J;

            cost += error*error;
        }

        Eigen::Vector3d delta = H.ldlt().solve(b);
        if(std::isnan(delta[0])){
            std::cout << "result is nan" <<std::endl;
            break;
        }

        if(iter >0 && cost >= lastCost)
        {
            std::cout << "cost less than lastCost .break. cost:" <<cost <<"\t\tlastCost:"<<lastCost <<std::endl;
            break;
        }
        std::cout << "total cost:" << cost <<", \t\tupdate:" << delta.transpose() << "\t\testimated params:"
        << result.transpose() << std::endl;

        lastCost = cost;
        result = result + delta;
    }

    std::cout << result.transpose() << std::endl;

    return 0;
}

