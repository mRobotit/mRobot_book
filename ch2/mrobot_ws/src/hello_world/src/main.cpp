#include <ros/ros.h>

int main(int argc, char** argv) {
        
        /**ros::init()函数需要看到argc和argv，这样它就可以执行命令行提供的任何ros参数和名称重新映射。
        * 对于编程式重映射，您可以使用不同版本的init()，它直接接受重映射，但对于大多数命令行程序来说，传递argc和argv是最简单的方法。
        * init()的第三个参数是节点的名称。在使用ros系统的任何其他部分之前，必须调用ros::init()的其中一个版本。*/
        ros::init(argc, argv, "hello_world");
        
        //打印“Hello World”
        ROS_INFO("Hello World");
        
        return 0;
}