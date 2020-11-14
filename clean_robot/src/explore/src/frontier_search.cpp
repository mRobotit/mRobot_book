#include <explore/frontier_search.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <explore/costmap_tools.h>

namespace frontier_exploration {
using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap, double potential_scale, double gain_scale,
                               double min_frontier_size)
    : costmap_(costmap)
    , potential_scale_(potential_scale)
    , gain_scale_(gain_scale)
    , min_frontier_size_(min_frontier_size) {}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position) {
    //存放所有边界的数据
    std::vector<Frontier> frontier_list;

    // 在寻找边界的过程中检查机器人是否处于costmap中。
    unsigned int mx, my;
    if (!costmap_->worldToMap(position.x, position.y, mx, my)) { //这里把机器人的真实位置，转化为在图像中的像素位置。
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    // 确保costmap的有效性，不必很深入的理解
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    // costmap的size
    map_ = costmap_->getCharMap();
    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();

    // 初始化cell是否为边界和是否被访问的标志位
    std::vector<bool> frontier_flag(size_x_ * size_y_, false); //初始的时候默认所有的cell都不是边界
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    // 采用广度优先搜索
    std::queue<unsigned int> bfs;

    ////////////////////////////////////这里是初始化查找的时候，找到最近的没有障碍物的costmap的cell//////////////////////////////
    //特别注意该函数意思是将map矩阵看成是一个一维的链表，找到机器人对应在一维数组里面的位置索引
    unsigned int clear, pos = costmap_->getIndex(mx, my);
    //判断是否存在最近的cell
    if (nearestCell(clear, pos, FREE_SPACE,
                    *costmap_)) { //这里采用c++的引用，如果找到离机器人附近的cell，那么就赋值给clear。
        bfs.push(clear);          //把机器人周围8个方向代价值最小的cell压进堆栈中
    } else {
        bfs.push(pos); //找不到的话就把当前机器人的cell push进入
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true; //标记当前cell为已访问

    ////////////////////////////////////这里是初始化查找之后的后续查找的过程，采用广度搜索来建立很多个边界数组//////////////////////////////
    //这里你要明白的是bfs里面已经存储了机器人当前cell周围的上下左右斜角的8个中代价值最小的cell，只有一个值哦。

    //广度搜索的模板
    while (!bfs.empty()) {
        unsigned int idx = bfs.front();
        bfs.pop();

        // 对队列中的cell进行周围四个邻居cell的遍历，初始的时候只有机器人所处的cell或者周围8个cell中的一个
        for (unsigned nbr : nhood4(idx, *costmap_)) {
            //如果当前遍历的cell的代价值小于bfs.front()的，并且没有访问过，则把这个压入队列，以便后续继续探索
            //这里是为了保证深度搜索队列中的每个cell的代价都应该很小，机器人可以安全的通过
            if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
                visited_flag[nbr] = true;
                bfs.push(nbr);
            }
            //当代价较大时，也不能不考虑，再考虑改cell周围其他的cell是否cost为0，也就是非常很安全。
            //检查是否是新的边界cell
            else if (isNewFrontierCell(
                         nbr, frontier_flag)) { //意思就是看当前cell周围的四个cell是否有可行区域，有的话返回true
                frontier_flag[nbr] = true;

                //*********************************关键的代码*****************************************
                // buildNewFrontier:通过广度搜索来不断的以当前cell为中心扩展，直到找到最远的cell为止，
                //把所有搜索过程中符合要求的cell全部压人new_frontier结构体的output中
                Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
                    frontier_list.push_back(new_frontier);
                }
            }
        }
    }

    // 设置边界的代价值。遍历存储好的vector边界
    for (auto& frontier : frontier_list) {
        frontier.cost = frontierCost(frontier); //边界点的个数越少，距离机器人越远，代价越大
    }
    //对边界进行递增排序
    std::sort(frontier_list.begin(), frontier_list.end(),
              [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

    return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                                          std::vector<bool>& frontier_flag) {
    // 初始化边界结构体
    Frontier output;
    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity(); //距离初始为无穷大

    // 记录和边界有联系的初始的点
    unsigned int ix, iy;
    costmap_->indexToCells(initial_cell, ix,
                           iy); //之前不是把cell变成一维了嘛，现在把这个一维initial_cell变成map下的二维坐标
    costmap_->mapToWorld(ix, iy, output.initial.x,
                         output.initial.y); //再把map的二维坐标转换到世界坐标系，其实就是有一个图像分辨率的问题，
    //图像的1个像素代表现实世界的0.05m，分辨率resolution=0.05

    // 把初始的cell压人队列
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    // 缓存机器人在世界坐标系下的位置
    unsigned int rx, ry;
    double reference_x, reference_y;
    costmap_->indexToCells(reference, rx, ry);
    costmap_->mapToWorld(rx, ry, reference_x, reference_y);

    while (!bfs.empty()) {
        unsigned int idx = bfs.front();
        bfs.pop();

        // 看是否可以把当前点的cell的8个邻居作为边界。
        for (unsigned int nbr : nhood8(idx, *costmap_)) {
            //意思就是看当前cell周围的四个cell是否有可行区域，有的话返回true
            if (isNewFrontierCell(nbr, frontier_flag)) {
                // 标记改cell可以作为边界
                frontier_flag[nbr] = true;
                unsigned int mx, my;
                double wx, wy;
                costmap_->indexToCells(nbr, mx, my);
                costmap_->mapToWorld(mx, my, wx, wy);
                //把改cell压人边界Frontier结构体里面的output的vector里面，注意这里是堆栈不是队列，是先入后出。
                geometry_msgs::Point point;
                point.x = wx;
                point.y = wy;
                output.points.push_back(point);

                // 更新边界的规模
                output.size++;

                // 更新边界的重心点的位置，特别注意这个是为了后续找到探索所有区域里面的平均点的位置用的
                output.centroid.x += wx;
                output.centroid.y += wy;

                //定义机器人所处位置到边界点重心点的距离，并且更新最小距离。
                double distance =
                    sqrt(pow((double(reference_x) - double(wx)), 2.0) + pow((double(reference_y) - double(wy)), 2.0));
                if (distance < output.min_distance) {
                    output.min_distance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }

                // 把当前有效的点压人队列继续进行探索。
                bfs.push(nbr);
            }
        }
    }

    // 把累加的边界点的坐标值求个平均，也就找到了重心的位置
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
    return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag) {
    // NO_INFORMATION=255意味改cell还没有被探索过是未知的，并且之前没有被看做是边界
    if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
        return false;
    }

    //如果当前cell周围4个cell存在可行空间则认为可以当做边界
    for (unsigned int nbr : nhood4(idx, *costmap_)) {
        if (map_[nbr] == FREE_SPACE) {
            return true;
        }
    }

    return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier) {
    return (potential_scale_ * frontier.min_distance * costmap_->getResolution()) -
           (gain_scale_ * frontier.size * costmap_->getResolution());
}
} // namespace frontier_exploration
