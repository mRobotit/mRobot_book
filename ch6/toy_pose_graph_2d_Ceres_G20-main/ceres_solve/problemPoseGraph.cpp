//
// Created by 万耀中 on 2020/10/26.
//

#include "problemPoseGraph.h"
problemPoseGraph::problemPoseGraph(const CONSTRAINTS & constraints, POSE_MAP& poses) {
    ceres::LossFunction* loss_function = nullptr;

    for (const auto& constraint: constraints) {
        auto pose_start_itr = poses.find(constraint.id_start_);
        if (pose_start_itr == poses.end()) {
            std::cerr << "Pose ID: " << constraint.id_start_ << " not found." << std::endl;
            continue;
        }
        auto pose_end_itr = poses.find(constraint.id_end_);
        if (pose_end_itr == poses.end()) {
            std::cerr << "Pose ID: " << constraint.id_end_ << " not found." << std::endl;
            continue;
        }
        std::array<double, 3> decomposed_information;
        for (unsigned int i = 0; i < 3; ++i) {
            decomposed_information.at(i) = sqrt(constraint.information.at(i));
        }


        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<SE2_COST, 3, 1, 1, 1, 1, 1, 1>(
                        new SE2_COST(constraint.x_, constraint.y_, constraint.theta_, decomposed_information));

        problem_.AddResidualBlock(cost_function, loss_function,
                                  &(pose_start_itr->second.x_),
                                  &(pose_start_itr->second.y_),
                                  &(pose_start_itr->second.theta_),
                                  &(pose_end_itr->second.x_),
                                  &(pose_end_itr->second.y_),
                                  &(pose_end_itr->second.theta_));
    }

    auto pose_start_itr = poses.begin();
    if (pose_start_itr == poses.end()) {
        std::cerr << "There are no poses" << std::endl;
        exit(EXIT_FAILURE);
    }

    //固定第一个位姿
    problem_.SetParameterBlockConstant(&(pose_start_itr->second.x_));
    problem_.SetParameterBlockConstant(&(pose_start_itr->second.y_));
    problem_.SetParameterBlockConstant(&(pose_start_itr->second.theta_));
}



bool problemPoseGraph::solve() {
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);

    std::cout << summary.FullReport() << std::endl;

    return summary.IsSolutionUsable();
}


