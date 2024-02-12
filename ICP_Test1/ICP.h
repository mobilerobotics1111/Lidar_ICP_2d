#pragma once

#include <math.h>
#include <string>
#include <iostream>
#include <vector>

#include "Eigen/Dense"

class ICP{
public:
    void solve(std::vector<Eigen::Vector2f>& scan1, std::vector<Eigen::Vector2f>& scan2);
    void point_base_matching(const std::vector<std::vector<Eigen::Vector2f>>& point_pairs);

    void print_rotation();
    void print_trans();
    void print_T();

    double get_transx();
    double get_transy();
    double get_rotation();

private:
protected:
    Eigen::Matrix2d rotation_;
    Eigen::Vector2d trans_;
    Eigen::Matrix3d T_;
};