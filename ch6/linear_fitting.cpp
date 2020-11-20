//
// Created by wyz on 2020/11/4.
//
#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<random>
#include<vector>

//从直线上观测到的点的数量
const int dataSize = 500;

void compute1(std::vector<double>& x_data,std::vector<double>& y_data,Eigen::Matrix2d& H,Eigen::Vector2d& b);
void compute2(std::vector<double>& x_data,std::vector<double>& y_data,Eigen::Matrix2d& H,Eigen::Vector2d& b);


//拟合直线 y = 2x + 1

int main(int argc,char** argv)
{
    std::default_random_engine randomEngine;
    //均值为0，方差为1.0的高斯分布
    std::normal_distribution<double> normalDistribution(0,1.0);

    double a_true = 2.0, b_true = 1.0;
    std::vector<double> x_data,y_data;
    for(int i = 0 ; i < dataSize ;++i)
    {
        double x = randomEngine()%200;

        //为数据添加噪声
        double y = a_true*x + b_true + normalDistribution(randomEngine);
        x_data.push_back(x);
        y_data.push_back(y);
    }

    //J^T J的最终结果
    Eigen::Matrix2d H;
    //J^T y的最终结果
    Eigen::Vector2d b;

    //一共有两种等价的方式，其等价性还请读者自行验证并体会
    //compute1(x_data,y_data,H,b);
    compute2(x_data,y_data,H,b);
    Eigen::Vector2d result = H.inverse()*b;

    //将向量转制打印，好看一些
    std::cout << result.transpose();
}

void compute1(std::vector<double>& x_data,std::vector<double>& y_data,Eigen::Matrix2d& H,Eigen::Vector2d& b)
{
    int n = x_data.size();

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> J;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> y;

    J.resize(n,2);
    y.resize(n,1);

    for(int i = 0 ; i <  dataSize ; ++i)
    {
        J(i,0) = x_data[i];
        J(i,1) = 1;
    }
    for(int i = 0 ; i < dataSize ;++i)
    {
        y(i,0) = y_data[i];
    }
    H = J.transpose()*J;
    b = J.transpose()*y;
}

void compute2(std::vector<double>& x_data,std::vector<double>& y_data,Eigen::Matrix2d& H,Eigen::Vector2d& b)
{
    for(int i = 0 ; i < dataSize ; ++i)
    {
        Eigen::Vector2d j{x_data[i],1};
        H += j*j.transpose();
        b += j*y_data[i];
    }
}