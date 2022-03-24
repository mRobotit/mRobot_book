# coding=utf-8
"""
原始激光数据(odom、flaser、sonar)，制作bag
"""
import rospy
import rosbag
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import pi
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf
import sys

def make_tf_msg(x, y, theta, t, base, base0):
    """
    t时刻，base0系在base系中的坐标表示：(x, y, theta)
    """
    trans = TransformStamped()
    trans.header.stamp = t
    trans.header.frame_id = base
    trans.child_frame_id = base0
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    # 2d的旋转只需要一个yaw角
    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]

    msg = TFMessage()
    msg.transforms.append(trans)
    return msg

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python convert_clf_bag.py example.clf example.bag")
        exit()
    with open(sys.argv[1]) as dataset:
        with rosbag.Bag(sys.argv[2], 'w') as bag:
            i = 1
            for line in dataset.readlines():
                line = line.strip()
                tokens = line.split(' ')
                if len(tokens) <= 2:
                    continue
                if tokens[0] == 'FLASER':
                    msg = LaserScan()
                    # 激光束数量
                    num_scans = int(tokens[1])

                    # 只有Intel Research Lab满足
                    if num_scans != 180 or len(tokens) < num_scans + 9:
                        rospy.logwarn("unsupported scan format")
                        continue
                    # 激光坐标系
                    msg.header.frame_id = 'base_laser_link'
                    # 时间戳
                    t = rospy.Time(float(tokens[(num_scans + 8)]))
                    msg.header.stamp = t
                    msg.header.seq = i
                    i += 1
                    msg.angle_min = -90.0 / 180.0 * pi
                    msg.angle_max = 90.0 / 180.0 * pi
                    # [-90,90]范围
                    msg.angle_increment = pi / num_scans
                    msg.time_increment = 0.2 / 360.0
                    # 一帧
                    msg.scan_time = 0.2
                    msg.range_min = 0.001
                    msg.range_max = 50.0
                    # 激光范围数据
                    msg.ranges = [float(r) for r in tokens[2:(num_scans + 2)]]
                    # Karto运行时，需要181个scan
                    msg.ranges.append(float(0))

                    bag.write('scan', msg, t)

                    # 机器人坐标系base_link在odom系下的坐标，来自IMU里程计、轮式里程计等
                    odom_x, odom_y, odom_theta = [float(r) for r in tokens[(num_scans + 2):(num_scans + 5)]]
                    tf_msg = make_tf_msg(odom_x, odom_y, odom_theta, t,'odom','base_link')
                    bag.write('tf', tf_msg, t)

                elif tokens[0] == 'ODOM':
                    odom_x, odom_y, odom_theta = [float(t) for t in tokens[1:4]]
                    t = rospy.Time(float(tokens[7]))
                    # 激光坐标系base_laser_link在机器人坐标系base_link中的坐标，这里使二者相等
                    tf_msg = make_tf_msg(0, 0, 0, t,'base_link','base_laser_link')
                    bag.write('tf', tf_msg, t)
                    tf_msg = make_tf_msg(odom_x, odom_y, odom_theta, t, 'odom', 'base_link')
                    bag.write('tf', tf_msg, t)