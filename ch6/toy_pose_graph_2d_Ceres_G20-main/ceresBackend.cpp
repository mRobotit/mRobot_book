#include <iostream>
#include <string>
#include "g2oDataManager.h"
#include "ceres_solve/problemPoseGraph.h"


void output_poses(const std::string& filename, const POSE_MAP & poses);

std::string script_path = "../script/";
int main(int argc, char* argv[]) {

    std::string g2o_file = argv[1];
    g2oDataManager g2o_manager(g2o_file);

    //测试读取模块的正确性
    g2o_manager.read_g2o_file();
    auto poses = g2o_manager.getPoses();
    auto constraints = g2o_manager.getConstraints();
    std::cout << "Number of poses: " << poses.size() << std::endl;
    std::cout << "Number of constraints: " << constraints.size() << std::endl;

    output_poses(script_path+"poses_original_ceres.txt", poses);


    problemPoseGraph problemDemo(constraints,poses);

    problemDemo.solve();
    output_poses(script_path+"poses_optimized_ceres.txt", poses);

    return 0;
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