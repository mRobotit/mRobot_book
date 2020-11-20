#include <iostream>
#include <string>
#include <vector>

#include "g2oDataManager.h"
#include "types.h"
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/sparse_optimizer.h>

std::string script_path = "../script/";
void output_poses(const std::string& filename, const POSE_MAP & poses);


int main(int argc,char** argv)
{

    std::string g2o_file = argv[1];
    g2oDataManager g2o_manager(g2o_file);

    //测试读取模块的正确性
    g2o_manager.read_g2o_file();
    auto poses = g2o_manager.getPoses();
    auto constraints = g2o_manager.getConstraints();
    std::cout << "Number of poses: " << poses.size() << std::endl;
    std::cout << "Number of constraints: " << constraints.size() << std::endl;
    output_poses(script_path+"poses_original_g2o.txt", poses);

    //构建图优化问题的求解块大小——均为3*3的矩阵
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,3>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);


    //调用g2o提供的VertexSE2构建顶点
    //用指针数组保存顶点信息
    std::vector<g2o::VertexSE2*> Vertexes;
    for(auto& pose:poses)
    {
        auto *v = new g2o::VertexSE2();
        //设置顶点id
        v->setId(pose.first);

        //固定住地一个顶点
        if(pose.first == 0){
            v->setFixed(true);
        }

        double se2[3] = {pose.second.x_,pose.second.y_,pose.second.theta_};
        v->setEstimateDataImpl(se2);
        optimizer.addVertex(v);
        Vertexes.push_back(v);
    }

    //调用g2o提供的EdgeSE2构建边
    int edge_cnt = 0;
    for(auto& constraint:constraints)
    {
        auto *e = new g2o::EdgeSE2();
        int idx1 = constraint.id_start_;
        int idx2 = constraint.id_end_;
        e->setId(edge_cnt++);
        e->setVertex(0,optimizer.vertices()[idx1]);
        e->setVertex(1,optimizer.vertices()[idx2]);
        e->setMeasurement(g2o::SE2(constraint.x_,constraint.y_,constraint.theta_));
        e->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(e);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(30);

    //设置最终的结果
    for(auto& vertex:Vertexes)
    {
       int idx = vertex->id();
       Eigen::Vector3d pose_result = vertex->estimate().toVector();
       poses[idx].x_ = pose_result.x();
       poses[idx].y_ = pose_result.y();
       poses[idx].theta_ = pose_result.z();
    }

    //打印
    output_poses(script_path+"poses_optimized_g2o.txt", poses);
    std::string saved_path(g2o_file.begin(),g2o_file.end()-4);
    saved_path.append("_result.g2o");
    optimizer.save(saved_path.c_str());
}






void output_poses(const std::string& filename, const POSE_MAP & poses) {
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
        std::cerr << "Couldn't open a file: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
    for (const auto& pair : poses) {
        outfile << pair.first << ","
                << pair.second.x_ << ","
                << pair.second.y_ << ","
                << pair.second.theta_ << std::endl;
    }
}