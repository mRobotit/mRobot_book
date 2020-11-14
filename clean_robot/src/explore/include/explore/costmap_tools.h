#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>

namespace frontier_exploration {
/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood4(unsigned int idx, const costmap_2d::Costmap2D& costmap) {
    // //把cell周围上下左右相邻的索引压入堆栈中
    std::vector<unsigned int> out;

    unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ - 1) {
        ROS_WARN("Evaluating nhood for offmap point");
        return out;
    }
    //把cell周围上下左右相邻的索引压入堆栈中，并且返回。
    if (idx % size_x_ > 0) {
        out.push_back(idx - 1);
    }
    if (idx % size_x_ < size_x_ - 1) {
        out.push_back(idx + 1);
    }
    if (idx >= size_x_) {
        out.push_back(idx - size_x_);
    }
    if (idx < size_x_ * (size_y_ - 1)) {
        out.push_back(idx + size_x_);
    }
    return out;
}

/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood8(unsigned int idx, const costmap_2d::Costmap2D& costmap) {
    //把cell周围上下左右相邻的索引压入堆栈中
    std::vector<unsigned int> out = nhood4(idx, costmap);

    unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ - 1) {
        return out;
    }
    //把cell周围斜角的四个相邻的索引压入堆栈中
    if (idx % size_x_ > 0 && idx >= size_x_) {
        out.push_back(idx - 1 - size_x_);
    }
    if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
        out.push_back(idx - 1 + size_x_);
    }
    if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
        out.push_back(idx + 1 - size_x_);
    }
    if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
        out.push_back(idx + 1 + size_x_);
    }

    return out;
}

/**
 * @brief Find nearest cell of a specified value
 * @param result Index of located cell
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap Reference to map data
 * @return True if a cell with the requested value was found
 */
bool nearestCell(unsigned int& result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap) {
    const unsigned char* map = costmap.getCharMap();
    const unsigned int size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();
    //如果机器人当前不在costmap中，则退出
    if (start >= size_x * size_y) {
        return false;
    }

    // 开始广度搜索（使用队列，先进先出）
    std::queue<unsigned int> bfs;
    //判断改cell是否被访问的标志
    std::vector<bool> visited_flag(size_x * size_y, false);

    // 把当前机器人的位置加入队列
    bfs.push(start);
    visited_flag[start] = true;

    // 依据value来找出最近的邻居
    while (!bfs.empty()) {
        unsigned int idx = bfs.front();
        bfs.pop();

        // 如果找到了非障碍物的cell，也就是cell.cost=0，则返回
        if (map[idx] == val) {
            result = idx;
            return true;
        }

        // 对最近的未访问的cell进行迭代
        for (unsigned nbr : nhood8(idx, costmap)) {
            if (!visited_flag[nbr]) { //如果没有访问过，就把当前的cell的索引压入队列。
                bfs.push(nbr);
                visited_flag[nbr] = true;
            }
        }
    }

    return false;
}
} // namespace frontier_exploration
#endif
