//
// Created by 万耀中 on 2020/10/26.
//
#include "g2oDataManager.h"

static double normalize_angle(const double& angle_radians) {
    double two_pi = 2.0 * M_PI;
    return angle_radians - two_pi * std::floor((angle_radians + (M_PI)) / two_pi);
}

//重载 >> 方便读取
std::istream& operator>>(std::istream& input, pose_2d& pose) {
    input >> pose.x_ >> pose.y_ >> pose.theta_;
    pose.theta_ = normalize_angle(pose.theta_);
    return input;
}

std::istream& operator>>(std::istream& input, Constraint& constraint) {
    input >> constraint.id_start_ >> constraint.id_end_ >> constraint.x_ >> constraint.y_ >> constraint.theta_;
    constraint.theta_ = normalize_angle(constraint.theta_);
    for (int i = 0; i < 3 && input.good(); ++i) {
        for (int j = i; j < 3 && input.good(); ++j) {
            double information_i_j;
            input >> information_i_j;
            if (i == j) {
                constraint.information.at(static_cast<unsigned int>(i)) = information_i_j;
            }
        }
    }
    return input;
}


void g2oDataManager::read_vertex(std::ifstream& infile) {
    int id;
    pose_2d pose;
    infile >> id >> pose;
    if (poseMap_.count(id) != 0) {
        std::cout << "Duplicate vertex with ID: " << id << std::endl;
        exit(EXIT_FAILURE);
    }
    poseMap_[id] = pose;
}

void g2oDataManager::read_constraint(std::ifstream& infile) {
    Constraint constraint;
    infile >> constraint;
    constraints_.push_back(constraint);
}

void g2oDataManager::read_g2o_file() {
    poseMap_.clear();
    constraints_.clear();

    std::ifstream infile(g2o_file_.c_str());
    if (!infile) {
        std::cerr << "Invalid file name: " << g2o_file_ << std::endl;
        exit(EXIT_FAILURE);
    }

    while (infile.good()) {
        std::string data_type;
        infile >> data_type;
        if (data_type == "VERTEX_SE2") {
            read_vertex(infile);
        }
        else if (data_type == "EDGE_SE2") {
            read_constraint(infile);
        }
        else {
            std::cout << "Unknown data type: " << data_type << std::endl;
            exit(EXIT_FAILURE);
        }
        infile >> std::ws;
    }
}