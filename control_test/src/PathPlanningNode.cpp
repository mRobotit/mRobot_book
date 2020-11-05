#include <costmap_2d/costmap_2d_ros.h>
#include <boost/shared_ptr.hpp>
#include "control_test/CleaningPathPlanner.h"

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using rm::PoseStamped;
using std::string;
using std::vector;

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node"); 
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf(tfBuffer);
    //初始化costmap代价图
    costmap_2d::Costmap2DROS lcr("cleaning_costmap", tfBuffer);

    ros::Duration(5).sleep();
    //在清扫路径规划的构造函数里面，其实就是把costmap的图，转换了一下维度，对相应的遍历赋值一下，
    //注意在costmap中是左下角是原点，列是行，行是列，所有转换为我们正常思维的地图矩阵MAT方便后续的操作
    CleaningPathPlanning clr(&lcr);

    //该函数就是将map二维数据转换为世界坐标下的一维数据，存放在pathVecInROS_这个数组中，并且发布在话题上。
    //×××××××××××××××××该函数就是得到弓形路径的函数××××××××××××××××
    clr.GetPathInROS();
    ros::Rate r(1);
    while (ros::ok()) {
        clr.PublishCoveragePath(); //一样的，该函数就是将map二维数据转换为世界坐标下的一维数据，存放在pathVecInROS_这个数组中，并且发布在话题上。
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown(); //关闭节点以及所有与之相关的发布，订阅，调用与服务。
    return 0;
}
