//
// Created by 万耀中 on 2020/10/26.
//

#ifndef POSEGRAPHDEMO_G2ODATAMANAGER_H
#define POSEGRAPHDEMO_G2ODATAMANAGER_H
#include <fstream>
#include <string>
#include <iostream>
#include "types.h"


class g2oDataManager {
public:
    g2oDataManager(const std::string& g2o_file):g2o_file_(g2o_file){}
    void read_g2o_file();
    POSE_MAP& getPoses(){return poseMap_;}
    CONSTRAINTS& getConstraints(){return constraints_;}
protected:

    void read_vertex(std::ifstream& infile);
    void read_constraint(std::ifstream& infile);

private:
    std::string g2o_file_;
    POSE_MAP poseMap_;
    CONSTRAINTS constraints_;
};


#endif //POSEGRAPHDEMO_G2ODATAMANAGER_H
