/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one, const geometry_msgs::Point& two) {
    double dx = one.x - two.x;
    double dy = one.y - two.y;
    double dist = sqrt(dx * dx + dy * dy);
    return dist < 0.01;
}

namespace explore {
Explore::Explore()
    : private_nh_("~")
    , tf_listener_(ros::Duration(10.0))
    , costmap_client_(private_nh_, relative_nh_,
                      &tf_listener_) //三个参数分别是获取参数（param）的节点，订阅话题的节点，tf监听器。
    , move_base_client_("move_base") //初始化movebase:用于给定点的ros导航包，全局路径规划djistla，局部的是dwa。
    , prev_distance_(0)
    , last_markers_count_(0) {
    double timeout;
    double min_frontier_size;
    private_nh_.param("planner_frequency", planner_frequency_, 1.0);
    private_nh_.param("progress_timeout", timeout, 30.0);
    progress_timeout_ = ros::Duration(timeout);
    private_nh_.param("visualize", visualize_, false);
    private_nh_.param("potential_scale", potential_scale_, 1e-3);
    private_nh_.param("orientation_scale", orientation_scale_, 0.0);
    private_nh_.param("gain_scale", gain_scale_, 1.0);
    private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
    //障碍物寻找
    search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(), potential_scale_, gain_scale_,
                                                   min_frontier_size);
    //发布边界话题
    if (visualize_) {
        marker_array_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
    }
    ROS_INFO("Waiting to connect to move_base server");
    move_base_client_.waitForServer();
    ROS_INFO("Connected to move_base server");

    //为了保证程序按照一定的频率进行运行，开启一个ros的定时器，ros::Duration(1. / planner_frequency_)表示定时周期，
    //在定时器里面执行makeplan
    exploring_timer_ = relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                                                [this](const ros::TimerEvent&) { makePlan(); });
    //****************************程序结束******************************(makeplan里面涉及很多)
}

Explore::~Explore() { stop(); }

//可视化的函数我们不需要过多关注。。。。。
void Explore::visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers) {
    std_msgs::ColorRGBA blue;
    blue.r = 0;
    blue.g = 0;
    blue.b = 1.0;
    blue.a = 1.0;
    std_msgs::ColorRGBA red;
    red.r = 1.0;
    red.g = 0;
    red.b = 0;
    red.a = 1.0;
    std_msgs::ColorRGBA green;
    green.r = 0;
    green.g = 1.0;
    green.b = 0;
    green.a = 1.0;

    ROS_DEBUG("visualising %lu frontiers", frontiers.size());
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;

    m.header.frame_id = costmap_client_.getGlobalFrameID();
    m.header.stamp = ros::Time::now();
    m.ns = "frontiers";
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;
    // lives forever
    m.lifetime = ros::Duration(0);
    m.frame_locked = true;

    // weighted frontiers are always sorted
    double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

    m.action = visualization_msgs::Marker::ADD;
    size_t id = 0;
    for (auto& frontier : frontiers) {
        m.type = visualization_msgs::Marker::POINTS;
        m.id = int(id);
        m.pose.position = {};
        m.scale.x = 0.1;
        m.scale.y = 0.1;
        m.scale.z = 0.1;
        m.points = frontier.points;
        if (goalOnBlacklist(frontier.centroid)) {
            m.color = red;
        } else {
            m.color = blue;
        }
        markers.push_back(m);
        ++id;
        m.type = visualization_msgs::Marker::SPHERE;
        m.id = int(id);
        m.pose.position = frontier.initial;
        // scale frontier according to its cost (costier frontiers will be smaller)
        double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
        m.scale.x = scale;
        m.scale.y = scale;
        m.scale.z = scale;
        m.points = {};
        m.color = green;
        markers.push_back(m);
        ++id;
    }
    size_t current_markers_count = markers.size();

    // delete previous markers, which are now unused
    m.action = visualization_msgs::Marker::DELETE;
    for (; id < last_markers_count_; ++id) {
        m.id = int(id);
        markers.push_back(m);
    }

    last_markers_count_ = current_markers_count;
    marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan() {
    // 通过costmap_client来获取机器人的位置和姿态信息
    auto pose = costmap_client_.getRobotPose();

    //*****************searchFrom：该函数很重要，作用：（根据机器人当前的位置，找到若干个边界，每个边界都是一个结构体，包含若干cell的坐标）
    //函数返回的是std::vector<Frontier>，而每个Frontier是结构体，
    /*
    struct Frontier {
      std::uint32_t size;
      double min_distance;
      double cost;
      geometry_msgs::Point initial;//初始点
      geometry_msgs::Point centroid;//质心点
      geometry_msgs::Point middle;//中间点
      std::vector<geometry_msgs::Point> points;//存放边界的cell的点的坐标
    };
    */
    // 并且会根据代价值对获取到的若干边界进行代价值递增排序。
    auto frontiers = search_.searchFrom(pose.position);

    ROS_DEBUG("found %lu frontiers", frontiers.size());
    for (size_t i = 0; i < frontiers.size(); ++i) {
        ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
    }
    //************************判断是否已经找不到边界了的条件1*******************
    //判断终止的条件之一，没有产生任何边界
    if (frontiers.empty()) {
        stop();
        return;
    }

    // 可视化边界vector，在rivz可以查看
    if (visualize_) {
        visualizeFrontiers(frontiers);
    }

    // 遍历所有边界
    auto frontier =
        std::find_if_not(frontiers.begin(), frontiers.end(),
                         [this](const frontier_exploration::Frontier& f) { return goalOnBlacklist(f.centroid); });

    //************************判断是否已经找不到边界了的条件2*******************

    //判断终止的条件之二，遍历完所有边界都没有返回值
    if (frontier == frontiers.end()) {
        stop();
        return;
    }

    //注意理解frontier->centroid,其实就是边界的重心位置设为探索的目标点
    geometry_msgs::Point target_position = frontier->centroid;

    bool same_goal = prev_goal_ == target_position;
    prev_goal_ = target_position;

    //改过程就是机器人正在想设定的目标点运动的过程，分两种情况，一种是目标点变化了，第二种是prev_distance_ >
    // frontier->min_distance机器人在不对的靠近目标点
    if (!same_goal || prev_distance_ > frontier->min_distance) {
        // 设定的目标点不相同或者（当前目标点的距离大于之前边界的距离，因为有可能机器人在移动，而探索点没有移动）
        last_progress_ = ros::Time::now();
        prev_distance_ = frontier->min_distance;
    }
    // 很长时间没有规划了，可能该目标是障碍物，加入黑名单。
    if (ros::Time::now() - last_progress_ > progress_timeout_) {
        frontier_blacklist_.push_back(target_position);
        ROS_DEBUG("Adding current goal to black list");
        makePlan();
        return;
    }

    // 如果我们一直在追这同一个目标点，那么说明都不需要做，直接返回就好
    if (same_goal) {
        return;
    }

    // 前面条件不满足的话，这里是机器人正在跟踪一个新的目标点，并把它发送给movebase导航。
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position = target_position;
    goal.target_pose.pose.orientation.w = 1.;
    goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
    goal.target_pose.header.stamp = ros::Time::now();
    move_base_client_.sendGoal(goal, [this, target_position](const actionlib::SimpleClientGoalState& status,
                                                             const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
    });
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal) {
    constexpr static size_t tolerace = 5;
    costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

    // 检查当前追踪的目标点是否在黑名单里
    for (auto& frontier_goal : frontier_blacklist_) {
        double x_diff = fabs(goal.x - frontier_goal.x);
        double y_diff = fabs(goal.y - frontier_goal.y);

        if (x_diff < tolerace * costmap2d->getResolution() && y_diff < tolerace * costmap2d->getResolution())
            return true;
    }
    return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status, const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal) {
    ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
    if (status == actionlib::SimpleClientGoalState::ABORTED) {
        frontier_blacklist_.push_back(frontier_goal);
        ROS_DEBUG("Adding current goal to black list");
    }
    oneshot_ = relative_nh_.createTimer(ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); }, true);
}

void Explore::start() { exploring_timer_.start(); }

//不向movebase发送目标点，机器人停止运动
void Explore::stop() {
    move_base_client_.cancelAllGoals();
    exploring_timer_.stop();
    ROS_INFO("Exploration stopped.");
}

} // namespace explore

int main(int argc, char** argv) {
    ros::init(argc, argv, "explore");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    //通过构造函数来直接探索策略
    explore::Explore explore;
    ros::spin();

    return 0;
}
