#pragma once

#include <math.h>
#include <string>
#include <iostream>
#include <vector>
#include "Eigen/Dense"

using std::cout;
using std::endl;

class Kdnode
{
public:
    Eigen::Vector2f data;
    Kdnode* left = NULL;
    Kdnode* right = NULL;

    ~Kdnode() {
        delete left;
        delete right;
    }
    Kdnode() = default;
    Kdnode& operator=(const Kdnode&) = default;

    void print_kdnode()
    {
        cout << "classdata: " << data(0) << " " << data(1) << endl;
        if (left)
        {
            left->print_kdnode();
        }
        if (right)
        {
            right->print_kdnode();
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void print_point(const std::vector<Eigen::Vector2f>& reference_point)
{
    for (unsigned int i = 0; i < reference_point.size(); i++)
    {
        cout << reference_point[i](0) << " " << reference_point[i](1) << endl;
    }
    cout << "end" << endl;
    return;
}

void kd_tree(std::vector<Eigen::Vector2f>& reference_point, const int depth, Kdnode* current)
{
    int index = reference_point.size();
    int axis = depth % 2;
    if (index == 0)
    {
        return;
    }

    if (axis == 0)
    {
        sort(reference_point.begin(), reference_point.end(), [&](const Eigen::Vector2f& a, const Eigen::Vector2f& b) -> bool
            { return a(0) < b(0); });
    }
    else
    {
        sort(reference_point.begin(), reference_point.end(), [&](const Eigen::Vector2f& a, const Eigen::Vector2f& b) -> bool
            { return a(1) < b(1); });
    }
    int median = index / 2;
    current->data = reference_point[median];

    if (median == 0)
    {
        reference_point.clear();
        std::vector<Eigen::Vector2f>().swap(reference_point);
        return;
    }

    std::vector<Eigen::Vector2f> temp1 = std::vector<Eigen::Vector2f>(reference_point.begin(), reference_point.begin() + median);
    std::vector<Eigen::Vector2f> temp2 = std::vector<Eigen::Vector2f>(reference_point.begin() + median + 1, reference_point.end());

    Kdnode* left = new Kdnode;
    Kdnode* right = new Kdnode;

    if (temp1.size()) {
        current->left = left;
        kd_tree(temp1, depth + 1, left);
    }
    if (temp2.size()) {
        current->right = right;
        kd_tree(temp2, depth + 1, right);
    }
}

Kdnode* closest(Kdnode* A, Kdnode* B, const Eigen::Vector2f& point)
{
    if (!A)
        return B;

    if (!B)
        return A;

    double d1 = (A->data - point).transpose() * (A->data - point);
    double d2 = (B->data - point).transpose() * (B->data - point);

    if (d1 < d2)
        return A;
    else
        return B;
}

Kdnode* nn(Kdnode* current, const Eigen::Vector2f& point, int depth)
{
    if (!current)
        return NULL;

    Kdnode* next_b, * other_b;

    if (point(depth % 2) < current->data(depth % 2))
    {
        next_b = current->left;
        other_b = current->right;
    }
    else
    {
        next_b = current->right;
        other_b = current->left;
    }

    Kdnode* temp = nn(next_b, point, depth + 1);
    Kdnode* best = closest(temp, current, point);

    double radius = (best->data - point).transpose() * (best->data - point);
    double dist = point(depth % 2) - current->data(depth % 2);

    if (radius >= dist * dist)
    {
        temp = nn(other_b, point, depth + 1);
        best = closest(temp, best, point);
    }
    return best;
}