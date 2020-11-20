#ifndef SE2_TYPES_H
#define SE2_TYPES_H

#include <istream>
#include <map>
#include <array>
#include <string>
#include <vector>
#include <cmath>


struct pose_2d {
public:
    double x_;
    double y_;
    double theta_;
};

struct Constraint
{
public:
    //每个constrain由两个位姿顶点和二者之间测量关系得出
    int id_start_;
    int id_end_;

    double x_;
    double y_;
    double theta_;

    //information代表优化的权重矩阵，往往由误差的协方差的逆得出，初学者可先认为该information全为1
    std::array<double, 3> information;
};

//定义存储信息的数据结构
typedef std::vector<Constraint> CONSTRAINTS;
typedef std::map<int, pose_2d, std::less<>> POSE_MAP;


#endif // SE2_TYPES_H
