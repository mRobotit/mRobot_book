#include <ros/ros.h>//ros/ros.h 是一个实用的头文件，它引用了 ROS 系统中大部分常用的头文件。
#include  <learning_topic/Person.h>

int main(int argc, char** argv) {

    /**ros::init()函数需要看到argc和argv，这样它就可以执行命令行提供的任何ros参数和名称重新映射。
    * 对于编程式重映射，您可以使用不同版本的init()，它直接接受重映射，但对于大多数命令行程序来说，传递argc和argv是最简单的方法。
    * init()的第三个参数是节点的名称。在使用ros系统的任何其他部分之前，必须调用ros::init()的其中一个版本。*/
    ros::init(argc, argv, "person_publisher");//初始化节点

    /*NodeHandle是与ROS系统通信的主要访问点。构造的第一个节点句柄将完全初始化该节点，而最后一个被解构的节点句柄将关闭该节点。*/
    ros::NodeHandle nh;// 创建节点句柄

    /*advertise()函数用于告诉ROS您希望在给定的主题名称上发布内容。
    *这将调用对ROS主节点的调用，该节点保存谁在发布谁在订阅的注册表。
    *在进行了这个advertise()调用之后，主节点将通知任何试图订阅此主题名称的人，然后它们将与此节点协商一个对等连接。
    *advertise()返回一个Publisher对象，该对象允许您通过调用publish()来发布关于该主题的消息。
    *一旦返回的发布者对象的所有副本被销毁，主题将自动取消通告。广告()的第二个参数是用于发布消息的消息队列的大小。
    *如果消息发布的速度快于我们发送它们的速度，那么这里的数字指定在丢弃一些消息之前需要缓冲多少消息。*/
    ros::Publisher person_info_pub = nh.advertise<learning_topic::Person>("/person_info", 10);// 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10

    ros::Rate loop_rate(1);// 设置循环的频率

    while(ros::ok()) {

        /*这是一个消息对象。你用数据填充它，然后发布它。*/
        learning_topic::Person person_msg;// 初始化learning_topic::Person类型的消息

        person_msg.name = "Tom";
        person_msg.age = 18;
        person_msg.sex = learning_topic::Person::male;

        /*publish()函数是您发送消息的方式。参数是message对象。
        *这个对象的类型必须与作为模板形参给advertise<>()调用的类型一致，正如在上面的构造函数中所做的那样。*/
        person_info_pub.publish(person_msg);// 发布消息

        ROS_INFO("Publish Person Info: name:%s age:%d sex:%d", person_msg.name.c_str(), person_msg.age, person_msg.sex);

        loop_rate.sleep(); // 按照循环频率延时
    }
    return 0;
}
