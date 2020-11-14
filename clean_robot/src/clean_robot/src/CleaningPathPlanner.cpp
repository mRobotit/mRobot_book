#include "CleaningPathPlanner.h"
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
CleaningPathPlanning::CleaningPathPlanning(costmap_2d::Costmap2DROS *costmap2d_ros) {
    // temp solution.
    costmap2d_ros_ = costmap2d_ros;
    // costmap2d_ros_->updateMap();
    costmap2d_ = costmap2d_ros->getCostmap();

    ros::NodeHandle private_nh("~/cleaning_plan_nodehandle");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("cleaning_path", 1);
    grid_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("covered_grid", 1);

    string sizeOfCellString, coveredValueStr;

    SIZE_OF_CELL = 3;
    if (private_nh.searchParam(
            "size_of_cell",
            sizeOfCellString)) //搜索参数,根据名称"size of cell"搜索参数，将对应名称下的参数值赋给sizeOfCellString.
        private_nh.param("size_of_cell", SIZE_OF_CELL, 3); //设置机器人占据n*n的栅格，决定规划的稀疏

    GRID_COVERED_VALUE = 0;
    if (private_nh.searchParam("grid_covered_value", coveredValueStr))
        private_nh.param("grid_covered_value", GRID_COVERED_VALUE, 0);

    int sizex = costmap2d_->getSizeInCellsX(); //获取地图尺寸
    int sizey = costmap2d_->getSizeInCellsY();
    cout << "The size of map is " << sizex << "  " << sizey << endl;
    resolution_ = costmap2d_->getResolution(); //分辨率

    //这里就是地图的转置，方便操作
    srcMap_ = Mat(sizey, sizex, CV_8U);
    for (int r = 0; r < sizey; r++) {
        for (int c = 0; c < sizex; c++) {
            srcMap_.at<uchar>(r, c) = costmap2d_->getCost(c, sizey - r - 1);
            // getCost（）:获取代价值
        }
    }

    initializeMats();
    // initializeCoveredGrid：该函数其实就是对成员变量covered_path_grid_赋值，他把costmap变为一维数组存进去了
    /*
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    nav_msgs/MapMetaData info
      time map_load_time
      float32 resolution
      uint32 width
      uint32 height
      geometry_msgs/Pose origin
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
    int8[] data//*************就是存在这里
    */
    initializeCoveredGrid();

    if (!srcMap_.empty())
        initialized_ = true; //这句话説明srcMap_里面得有东西才能说明初始化成功。
    else
        initialized_ = false;
}

//规划路径
vector<geometry_msgs::PoseStamped> CleaningPathPlanning::GetPathInROS() {
    if (!pathVecInROS_.empty()) pathVecInROS_.clear(); //清空操作
    //对我们要规划的路径数据进行清空pathVecInROS_是一个vector，数据类型是geometry_msgs::PoseStamped，结构体如下：
    /*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/

    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Pose pose;
    vector<cellIndex> cellvec;
    // cellIndex记录某个cell的位置和代价信息的结构体
    /*struct cellIndex {
    int row;
    int col;
    double theta; //{0, 45,90,135,180,225,270,315}     角度信息 
};*/

    //*****************************************************得到弓形路径的入口********************************************
    //该函数返回所有弓形的点的集合vector!!!!!!!!!!!!!!!!!!!!!!!!!!!!!重要
    cellvec = GetPathInCV();

    vector<cellIndex>::iterator iter; // cellIndex里面存放的是行，列以及角度信息。
    int sizey = cellMat_.rows;

    //该循环的作用就是把map数据变为世界坐标系下的数据，并且遍历形的点的集合cellvec，将所有点存在pathVecInROS_这个vector中，这里变成了一维数据。
    for (iter = cellvec.begin(); iter != cellvec.end(); iter++) {
        costmap2d_->mapToWorld((*iter).col * SIZE_OF_CELL + SIZE_OF_CELL / 2,
                               (sizey - (*iter).row - 1) * SIZE_OF_CELL + SIZE_OF_CELL / 2, pose.position.x,
                               pose.position.y);
        pose.orientation.w = cos((*iter).theta * PI / 180 / 2); //(sizey-(*iter).row-1)
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = sin((*iter).theta * PI / 180 / 2);
        posestamped.header.stamp = ros::Time::now();
        posestamped.header.frame_id = "map";
        posestamped.pose = pose;
        pathVecInROS_.push_back(posestamped);
    }

    //将改路径发布出去，供next_goal节点订阅，然后以此的去发布导航点。
    publishPlan(pathVecInROS_);
    cout << "The path size is " << pathVecInROS_.size() << endl;
    return pathVecInROS_;
}

vector<geometry_msgs::PoseStamped> CleaningPathPlanning::GetBorderTrackingPathInROS() {
    // vector<geometry_msgs::PoseStamped> resultPathInROS;
    if (!pathVecInROS_.empty()) pathVecInROS_.clear();
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Pose pose;
    vector<cv::Point2i> resultCV;
    GetBorderTrackingPathInCV(resultCV);
    vector<bool> visitedVec;

    cv::Point2i lastPoint = resultCV[0], curPoint;
    double theta = 0.0;
    visitedVec.assign(resultCV.size(), false);
    int i, j, k, index;
    for (i = 0; i < resultCV.size(); i++) {
        if (!visitedVec[i]) {
            for (k = -SIZE_OF_CELL / 2; k < SIZE_OF_CELL / 2; k++) {
                for (j = -SIZE_OF_CELL / 2; j < SIZE_OF_CELL / 2; j++) {
                    if (findElement(resultCV, Point2i(resultCV[i].x + j, resultCV[i].y + k), index)) {
                        visitedVec[index] = true;

                        curPoint = resultCV[index];
                        if (curPoint.x == lastPoint.x) {
                            if (curPoint.y > lastPoint.y)
                                theta = PI / 2;
                            else
                                theta = -PI / 2;
                        } else
                            theta = atan((curPoint.y - lastPoint.x) / (curPoint.x - lastPoint.x));

                        // srcMap_'s shape is same with costmap2d_,ie.. they have same resolution and size.
                        costmap2d_->mapToWorld(resultCV[index].x, srcMap_.rows - resultCV[index].y - 1, pose.position.x,
                                               pose.position.y);
                        pose.orientation.w = cos(theta * PI / 180 / 2);
                        pose.orientation.x = 0;
                        pose.orientation.y = 0;
                        pose.orientation.z = sin(theta * PI / 180 / 2);
                        posestamped.header.stamp = ros::Time::now();
                        posestamped.header.frame_id = "map";
                        posestamped.pose = pose;

                        pathVecInROS_.push_back(posestamped);
                    }
                }
            }
        }
    }
    publishPlan(pathVecInROS_);
    return pathVecInROS_;
}

// First make border path planning in original resolution using opencv tools.Then transform the result
// in ROS format and take the robot's shape into account.
void CleaningPathPlanning::GetBorderTrackingPathInCV(vector<cv::Point2i> &resultVec) {
    std::vector<cv::Point2i> borderPointsIndexVec; // todo:make point2i corrresponding to cellindex.
    resultVec.clear();
    if (srcMap_.empty()) return; // srcMap 是个啥？

    cv::Point2i temppoint;
    int r, c, i, j;
    for (r = 0; r < srcMap_.rows; r++) {
        for (c = 0; c < srcMap_.cols; c++) {
            // ROS_INFO("The value at row %d, col %d is %d",r,c,srcMap_.at<uchar>(r,c));
            if (srcMap_.at<uchar>(r, c) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                ROS_INFO("entered once!");
                if (srcMap_.at<uchar>(r, c - 1) != srcMap_.at<uchar>(r, c + 1) &&
                    (srcMap_.at<uchar>(r, c - 1) == costmap_2d::FREE_SPACE ||
                     srcMap_.at<uchar>(r, c + 1) == costmap_2d::FREE_SPACE)) {
                    temppoint.x = c;
                    temppoint.y = r;
                    borderPointsIndexVec.push_back(temppoint);
                } else if (srcMap_.at<uchar>(r - 1, c) != srcMap_.at<uchar>(r + 1, c) &&
                           (srcMap_.at<uchar>(r - 1, c) == costmap_2d::FREE_SPACE ||
                            srcMap_.at<uchar>(r + 1, c) == costmap_2d::FREE_SPACE)) {
                    temppoint.x = c;
                    temppoint.y = r;
                    borderPointsIndexVec.push_back(temppoint);
                }
            }
        }
    }

    std::cout << "The border path length is:" << borderPointsIndexVec.size() << std::endl;
    std::vector<bool> visitedVec;
    if (borderPointsIndexVec.empty()) return;

    int minDistIndex = -1, loopCounter = 0;
    double dist, minDist;
    visitedVec.assign(borderPointsIndexVec.size(), false); // vector assign是vector容器下的赋值函数。

    Point2i curPoint = borderPointsIndexVec[0];
    visitedVec[0] = true;

    while (loopCounter < borderPointsIndexVec.size()) {
        minDist = 1e6;
        resultVec.push_back(curPoint);
        for (j = 1; j < borderPointsIndexVec.size(); j++) {
            if (!visitedVec[j]) {
                dist = distance(curPoint, borderPointsIndexVec[j]);
                if (dist < minDist) {
                    minDist = dist;
                    minDistIndex = j;
                }
            }
        }
        curPoint = borderPointsIndexVec[minDistIndex];
        visitedVec[minDistIndex] = true;
        loopCounter++;
    }

    // Mat resultmat=Mat(srcMap_.rows,srcMap_.cols, CV_8UC3);;
    // writeResult(resultmat,resultVec);
}

void CleaningPathPlanning::SetCoveredGrid(double wx, double wy) {
    unsigned int mx, my, index;
    bool isok = costmap2d_->worldToMap(wx, wy, mx, my);
    if (!isok) {
        return;
    }

    for (int dx = -SIZE_OF_CELL / 2; dx < SIZE_OF_CELL / 2 + 1; dx++) {
        for (int dy = -SIZE_OF_CELL / 2; dy < SIZE_OF_CELL / 2 + 1; dy++) {
            index = costmap2d_->getIndex(mx + dx, my + dy);
            covered_path_grid_.data[index] = GRID_COVERED_VALUE;
        }
    }
}

void CleaningPathPlanning::PublishGrid() {
    if (!initialized_) initializeCoveredGrid();
    grid_pub_.publish(covered_path_grid_);
}

vector<cellIndex> CleaningPathPlanning::GetPathInCV() {
    //入口
    mainPlanningLoop();
    return this->pathVec_;
}

void CleaningPathPlanning::PublishCoveragePath() { publishPlan(this->pathVecInROS_); }

void CleaningPathPlanning::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path) {
    if (!initialized_) {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    // create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool CleaningPathPlanning::cellContainsPoint(Point2i pt, cellIndex cell) {
    return pt.y > cell.row * SIZE_OF_CELL && pt.y < (cell.row + 1) * SIZE_OF_CELL && pt.x > cell.col * SIZE_OF_CELL &&
           pt.x < (cell.col + 1) * SIZE_OF_CELL;
}

bool CleaningPathPlanning::initializeMats() {
    // 初始化成员变量
    if (srcMap_.empty()) return false;

    //这里就是将转置以后的cellMat_将一下维度，比如我们规定3*3的网格为一个网格，那么矩阵的大下就缩放了
    //如果3*3的costmap的cell都是free空间，也就是costvalue-=0,那么给矩阵cellMat_的对应位置赋予0.
    getCellMatAndFreeSpace(srcMap_, cellMat_, freeSpaceVec_);

    //过渡矩阵，在后续计算的时候用到
    neuralizedMat_ = Mat(cellMat_.rows, cellMat_.cols, CV_32F);
    //给过度矩阵赋值，对于致命的空间赋值-1000000，其他的值让其不要超过50
    //注意：neuralizedMat在初始的时候，致命代价是-1000000,其他区域的值不会超过50，并且是按列越高进行递减的。
    initializeNeuralMat(cellMat_, neuralizedMat_);

    return true;
}

void CleaningPathPlanning::getCellMatAndFreeSpace(Mat srcImg, Mat &cellMat, vector<cellIndex> &freeSpaceVec) {
    cellMat = Mat(srcImg.rows / SIZE_OF_CELL, srcImg.cols / SIZE_OF_CELL,
                  srcImg.type()); // cellMat是以之前规定的cell为最小单位重新划分的矩阵

    freeSpaceVec.clear();
    bool isFree = true;
    int r = 0, c = 0, i = 0, j = 0;
    for (r = 0; r < cellMat.rows; r++) {
        for (c = 0; c < cellMat.cols; c++) {
            isFree = true;
            for (i = 0; i < SIZE_OF_CELL; i++) {
                for (j = 0; j < SIZE_OF_CELL; j++) {
                    if (srcImg.at<uchar>(r * SIZE_OF_CELL + i, c * SIZE_OF_CELL + j) != costmap_2d::FREE_SPACE) {
                        isFree = false;
                        i = SIZE_OF_CELL;
                        break;
                    }
                }
            }
            if (isFree) {
                cellIndex ci;
                ci.row = r;
                ci.col = c;
                ci.theta = 0;
                freeSpaceVec.push_back(ci);
                cellMat.at<uchar>(r, c) = costmap_2d::FREE_SPACE; // 0
            } else {
                cellMat.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;
            } // 254
        }
    }
    cout << "freespace size:" << freeSpaceVec.size() << endl;
    // imwrite("cellMat.jpg",cellMat);
    return;
}

void CleaningPathPlanning::initializeNeuralMat(Mat cellMat, Mat neuralizedMat) {
    int i = 0, j = 0;
    for (i = 0; i < neuralizedMat.rows; i++) {
        for (j = 0; j < neuralizedMat.cols; j++) {
            if (cellMat.at<uchar>(i, j) == costmap_2d::LETHAL_OBSTACLE)
                neuralizedMat.at<float>(i, j) = -100000.0;
            else
                neuralizedMat.at<float>(i, j) = 50.0 / j;
        }
    }
    return;
}

void CleaningPathPlanning::writeResult(Mat resultmat, vector<cellIndex> pathVec) {
    int i = 0, j = 0;
    Point initpoint = Point(pathVec[0].col * SIZE_OF_CELL + SIZE_OF_CELL / 2,
                            pathVec[0].row * SIZE_OF_CELL + SIZE_OF_CELL / 2); //这里是取中心位置的意思吗？
    for (i = 1; i < pathVec.size(); i++) {
        Point cupoint =
            Point(pathVec[i].col * SIZE_OF_CELL + SIZE_OF_CELL / 2, pathVec[i].row * SIZE_OF_CELL + SIZE_OF_CELL / 2);
        if (sqrt((initpoint.x - cupoint.x) * (initpoint.x - cupoint.x) +
                 (initpoint.y - cupoint.y) * (initpoint.y - cupoint.y)) > 2) {
            line(resultmat, initpoint, cupoint, Scalar(0, 255, 0), 0.3, 8); // line就是划线函数
        } else
            line(resultmat, initpoint, cupoint, Scalar(0, 0, 255), 0.5);
        initpoint = cupoint;
        cout << "The point of step " << i << " is: " << pathVec[i].row << " " << pathVec[i].col << endl;
        /*resultMat.at<Vec3b>(pathVec[i].row*SIZE_OF_CELL,pathVec[i].col*SIZE_OF_CELL)[0] = 0;
        resultMat.at<Vec3b>(pathVec[i].row*SIZE_OF_CELL,pathVec[i].col*SIZE_OF_CELL)[1] = 0;
        resultMat.at<Vec3b>(pathVec[i].row*SIZE_OF_CELL,pathVec[i].col*SIZE_OF_CELL)[2] = 255;*/
    }
    //    imshow("resultMat",resultmat);
    //    waitKey(0);
    imwrite("reaultMat.jpg", resultmat);
}

void CleaningPathPlanning::writeResult(Mat resultmat, vector<cv::Point2i> pathVec) {
    int i = 0, j = 0;
    Point initpoint = pathVec[0];
    for (i = 1; i < pathVec.size(); i++) {
        Point cupoint = pathVec[i];
        line(resultmat, initpoint, cupoint, Scalar(0, 0, 255), 0.5);
        initpoint = cupoint;
        // std::cout<<"X: "<<cupoint.x<<","<<"Y:"<<cupoint<<std::endl;
    }
    //    imshow("resultMat",resultmat);
    //    waitKey(0);
}

void CleaningPathPlanning::mainPlanningLoop() {
    // cellIndex是存放点的行列坐标，还有相对于机器人角度的结构体
    cellIndex initPoint, nextPoint, currentPoint;
    initPoint.theta = 90;
    //判断是否获取到机器人的初始位置
    bool isok = costmap2d_ros_->getRobotPose(initPose_);
    if (!isok) {
        ROS_INFO("Failed to get robot location! Please check where goes wrong!");
        return;
    }
    // mx,my是map坐标系像素的位置，wx,wy是时间坐标系下的真实位置
    unsigned int mx, my;
    double wx = initPose_.pose.position.x; //获取原点的x坐标
    double wy = initPose_.pose.position.y;
    //转换到世界坐标
    bool getmapcoor = costmap2d_->worldToMap(wx, wy, mx, my);
    if (!getmapcoor) {
        ROS_INFO("Failed to get robot location in map! Please check where goes wrong!");
        return;
    }
    //获取机器人在map下的坐标
    initPoint.row = cellMat_.rows - my / SIZE_OF_CELL - 1;
    initPoint.col = mx / SIZE_OF_CELL;

    currentPoint = initPoint; //将初始化的点赋予给当前点。

    // pathVec_是vector<cellIndex>，存放路径点的位置和相对于机器人的朝向的
    pathVec_.clear();
    pathVec_.push_back(initPoint);

    float initTheta = initPoint.theta; // initial orientation

    //******************************弓形算法开始*************
    //这是后面算法中要用到的参数。
    const float c_0 = 50;
    float e = 0.0, v = 0.0, v_1 = 0.0, deltaTheta = 0.0, lasttheta = initTheta, PI = 3.14159;
    int fx;
    //一个方向数组，就是到时候用来遍历机器人周围这几个方向的数组
    vector<float> thetaVec = {0, 45, 90, 135, 180, 225, 270, 315};

    for (int loop = 0; loop < 9000; loop++) {
        vector<cellIndex>::iterator it;

        //这里进行的是有关方向上的抉择。
        int maxIndex = 0;
        float max_v = -300;
        //所有的数据都是在这个过渡矩阵上面操作，将当前机器人的位置赋值-250，意味这再不会进行访问。
        neuralizedMat_.at<float>(currentPoint.row, currentPoint.col) = -250.0;
        //存放当前的点相对于机器人的朝向
        lasttheta = currentPoint.theta;
        //对当前点周围的8个点进行遍历
        for (int id = 0; id < 8; id++) {
            //计算每个点和上次点的角度差
            deltaTheta = max(thetaVec[id], lasttheta) - min(thetaVec[id], lasttheta);
            if (deltaTheta > 180) deltaTheta = 360 - deltaTheta;
            //使得增益不超过1，因为deltaTheta不会超过180度，后续会提到
            e = 1 - abs(deltaTheta) / 180; //角度参数？

            //***********注意：neuralizedMat在初始的时候，致命代价是-1000000,其他区域的值不会超过50，并且是按列越高进行递减的。
            switch (id) {
                case 0:

                    //         90度
                    //         ^
                    // 180度 <-  -> 0度

                    //如果当前点的朝向是0度的话，，，绝对不能碰到最右边的墙壁，是否当前点的位置在地图的最右边界
                    if (currentPoint.col == neuralizedMat_.cols - 1) {
                        v = -100000; //边界赋值
                        break;
                    }

                    //不处于边界的话，计算对应的有效值。因为e不超过1，neuralizedMat所以不会超过50+50=100
                    v = neuralizedMat_.at<float>(currentPoint.row, currentPoint.col + 1) + c_0 * e;

                    break;
                case 1:
                    //后续同理自己推一下，注意45度方向的不优先考虑所以多减去200.
                    if (currentPoint.col == neuralizedMat_.cols - 1 || currentPoint.row == 0) {
                        v = -100000;
                        break;
                    }
                    v = neuralizedMat_.at<float>(currentPoint.row - 1, currentPoint.col + 1) + c_0 * e - 200;

                    break;
                case 2:
                    if (currentPoint.row == 0) {
                        v = -100000;
                        break;
                    }
                    v = neuralizedMat_.at<float>(currentPoint.row - 1, currentPoint.col) + c_0 * e;

                    break;
                case 3:
                    if (currentPoint.col == 0 || currentPoint.row == 0) {
                        v = -100000;
                        break;
                    }
                    v = neuralizedMat_.at<float>(currentPoint.row - 1, currentPoint.col - 1) + c_0 * e - 200;

                    break;
                case 4:
                    if (currentPoint.col == 0) {
                        v = -100000;
                        break;
                    }
                    v = neuralizedMat_.at<float>(currentPoint.row, currentPoint.col - 1) + c_0 * e;

                    break;
                case 5:
                    if (currentPoint.col == 0 || currentPoint.row == neuralizedMat_.rows - 1) {
                        v = -100000;
                        break;
                    }
                    v = neuralizedMat_.at<float>(currentPoint.row + 1, currentPoint.col - 1) + c_0 * e - 200;

                    break;
                case 6:
                    if (currentPoint.row == neuralizedMat_.rows - 1) {
                        v = -100000;
                        break;
                    }
                    v = neuralizedMat_.at<float>(currentPoint.row + 1, currentPoint.col) + c_0 * e;

                    break;
                case 7:
                    if (currentPoint.col == neuralizedMat_.cols - 1 || currentPoint.row == neuralizedMat_.rows - 1) {
                        v = -100000;
                        break;
                    }
                    v = neuralizedMat_.at<float>(currentPoint.row + 1, currentPoint.col + 1) + c_0 * e - 200;

                    break;
                default:
                    break;
            }

            // max_v=-300，如果比-300大的话，也就是不会是-100000,证明不会碰到墙壁，
            //同时可以找出离机器人最近8个点中的v最大的点的角度值maxIndex(相对于机器人机器人的角度)
            if (v > max_v) {
                max_v = v;
                maxIndex = id;
            }

            if (v == max_v && id > maxIndex) {
                max_v = v;
                maxIndex = id;
            }
        }
        //机器人周围遇到了map的边界，应该在整个freeSpaceVec_遍历来找到有效的点。
        if (max_v <= 0) {
            float dist = 0.0, min_dist = 100000000;
            // vector<cellIndex>::iterator min_iter;
            int ii = 0, min_index = -1;
            //遍历map中所有的自由空间
            for (it = freeSpaceVec_.begin(); it != freeSpaceVec_.end(); it++) {
                if (neuralizedMat_.at<float>((*it).row, (*it).col) > 0) {
                    if (Boundingjudge((*it).row,
                                      (*it).col)) //周围是否网格中存在-250的点，证明遍历的map的点落在了机器人的位置周围
                    {
                        dist = sqrt((currentPoint.row - (*it).row) * (currentPoint.row - (*it).row) +
                                    (currentPoint.col - (*it).col) * (currentPoint.col - (*it).col));

                        if (dist < min_dist) {
                            min_dist = dist;
                            min_index = ii;
                        }
                    }
                }
                ii++;
            }
            // if(min_dist==0 || min_index == -1)
            //{break;}
            // else
            if (min_index != -1 && min_dist != 100000000) {
                cout << "next point index: " << min_index << endl;
                cout << "distance: " << min_dist << endl;
                nextPoint = freeSpaceVec_[min_index];
                currentPoint = nextPoint;
                pathVec_.push_back(nextPoint);

                continue;
            } else //产生了自锁现象
            {
                ROS_INFO("The program has been dead because of the self-locking");
                ROS_INFO("The program has gone through %ld steps", pathVec_.size());
                break;
            }
        }
        // 正常的情况，来找到v最大的对应的点的坐标值
        switch (maxIndex) {
            case 0:
                nextPoint.row = currentPoint.row;
                nextPoint.col = currentPoint.col + 1;
                break;
            case 1:
                nextPoint.row = currentPoint.row - 1;
                nextPoint.col = currentPoint.col + 1;
                break;
            case 2:
                nextPoint.row = currentPoint.row - 1;
                nextPoint.col = currentPoint.col;
                break;
            case 3:
                nextPoint.row = currentPoint.row - 1;
                nextPoint.col = currentPoint.col - 1;
                break;
            case 4:
                nextPoint.row = currentPoint.row;
                nextPoint.col = currentPoint.col - 1;
                break;
            case 5:
                nextPoint.row = currentPoint.row + 1;
                nextPoint.col = currentPoint.col - 1;
                break;
            case 6:
                nextPoint.row = currentPoint.row + 1;
                nextPoint.col = currentPoint.col;
                break;
            case 7:
                nextPoint.row = currentPoint.row + 1;
                nextPoint.col = currentPoint.col + 1;
                break;
            default:
                break;
        }
        //记录改点相对于机器人的角度
        nextPoint.theta = thetaVec[maxIndex];
        currentPoint = nextPoint;
        //把该点放入弓字路径的点的坐标里面
        pathVec_.push_back(nextPoint);
    }

    Mat resultMat = Mat(srcMap_.rows, srcMap_.cols, CV_8UC3);
    //测试用的函数，可以不看。。。。。
    writeResult(resultMat, pathVec_);
}

double CleaningPathPlanning::distance(Point2i pta, Point2i ptb) {
    return sqrt((pta.x - ptb.x) * (pta.x - ptb.x) + (pta.y - ptb.y) * (pta.y - ptb.y));
}

bool CleaningPathPlanning::findElement(vector<Point2i> pointsVec, Point2i pt, int &index) {
    for (int i = 0; i < pointsVec.size(); i++) {
        if (pointsVec[i].x == pt.x && pointsVec[i].y == pt.y) {
            index = i;
            return true;
        }
    }
    index = -1;
    return false;
}

bool CleaningPathPlanning::initializeCoveredGrid() // CoverGrid的理解为覆盖栅格。
{
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap2d_->getMutex()));
    double resolution = costmap2d_->getResolution(); //分辨率

    covered_path_grid_.header.frame_id = "map"; // covered_path_grid_是costmap库中的占据栅格地图消息。
    covered_path_grid_.header.stamp = ros::Time::now();
    covered_path_grid_.info.resolution = resolution;

    covered_path_grid_.info.width = costmap2d_->getSizeInCellsX();
    covered_path_grid_.info.height = costmap2d_->getSizeInCellsY();

    double wx, wy;
    costmap2d_->mapToWorld(0, 0, wx, wy); //从地图坐标系转换至世界坐标系。
    covered_path_grid_.info.origin.position.x = wx - resolution / 2;
    covered_path_grid_.info.origin.position.y = wy - resolution / 2;
    covered_path_grid_.info.origin.position.z = 0.0;
    covered_path_grid_.info.origin.orientation.w = 1.0;

    covered_path_grid_.data.resize(covered_path_grid_.info.width *
                                   covered_path_grid_.info.height); //这里可以理解为一共有多少个栅格，所以长×宽

    unsigned char *data =
        costmap2d_
            ->getCharMap(); //返回一个指针，指向一个底层无符号字符数组，数组中存储着代价值，这个数组貌似还可以用来作为代价地图。
    for (unsigned int i = 0; i < covered_path_grid_.data.size(); i++) {
        covered_path_grid_.data[i] = data[i]; //将代价值赋予到栅格地图的每个对应栅格当中去。
    }
    return true;
}

bool CleaningPathPlanning::Boundingjudge(int a, int b) {
    int num = 0;
    for (int i = -1; i <= 1; i++) {
        for (int m = -1; m <= 1; m++) {
            if (i == 0 && m == 0) {
                continue;
            }
            if (neuralizedMat_.at<float>((a + i), (b + m)) == -250.0) num++;
        }
    }
    if (num != 0)
        return true;
    else
        return false;
}
