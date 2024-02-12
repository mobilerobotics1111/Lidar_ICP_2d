#include "ICP.h"
#include "NN.h"

void ICP::solve(std::vector<Eigen::Vector2f>& scan1, std::vector<Eigen::Vector2f>& scan2)
{
    Kdnode* root, * best;
    std::vector<Eigen::Vector2f> point_pair;
    std::vector<std::vector<Eigen::Vector2f>> point_pairs;
    Eigen::Vector2f A, B;
    double distance;

    T_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    for (int itertate = 0; itertate < 50; itertate++) {
        //cout << "iteration: " << itertate << endl;

        root = new Kdnode;
        kd_tree(scan1, 0, root);

        for (unsigned int i = 0; i < scan2.size(); i++)
        {
            // cout << "point paire ready...: " << i << endl;
            best = nn(root, scan2[i], 0);
            // cout << "best best best...: " << best->data << endl;
            distance = (best->data - scan2[i]).transpose() * (best->data - scan2[i]);

            if (distance > 20)
                continue;
            A << best->data(0), best->data(1);
            B << scan2[i](0), scan2[i](1);

            point_pair.push_back(B);
            point_pair.push_back(A);
            point_pairs.push_back(point_pair);

            point_pair.clear();
            std::vector<Eigen::Vector2f>().swap(point_pair);
        }
        delete root;

        if (point_pairs.size() == 0)
        {
            cout << "error" << endl;
            return;
        }

        // cout << "point paire end" << endl;
        point_base_matching(point_pairs);

        point_pairs.clear();
        std::vector<std::vector<Eigen::Vector2f>>().swap(point_pairs);

        for (unsigned int i = 0; i < scan2.size(); i++)
        {
            Eigen::Vector2d temp;
            temp << scan2[i](0), scan2[i](1);
            temp = rotation_ * temp;
            temp(0) = temp(0) + trans_(0);
            temp(1) = temp(1) + trans_(1);

            scan2.at(i) << temp(0), temp(1);
        }

        //print_T();

        if (trans_(0) < 0.0001 && trans_(1) < 0.0001 && acos(rotation_(0, 0)) < 0.00001)
        {
            break;
        }
    }
}

void ICP::point_base_matching(const std::vector<std::vector<Eigen::Vector2f>>& point_pairs)
{
    long double x_mean = 0;
    long double y_mean = 0;
    long double xp_mean = 0;
    long double yp_mean = 0;
    int index = point_pairs.size();

    for (int i = 0; i < index; i++)
    {
        x_mean += point_pairs[i][0](0);
        y_mean += point_pairs[i][0](1);
        xp_mean += point_pairs[i][1](0);
        yp_mean += point_pairs[i][1](1);
    }

    x_mean /= index;
    y_mean /= index;
    xp_mean /= index;
    yp_mean /= index;

    double s_x_xp = 0;
    double s_y_yp = 0;
    double s_x_yp = 0;
    double s_y_xp = 0;

    for (int i = 0; i < index; i++)
    {
        s_x_xp += (point_pairs[i][0](0) - x_mean) * (point_pairs[i][1](0) - xp_mean);
        s_y_yp += (point_pairs[i][0](1) - y_mean) * (point_pairs[i][1](1) - yp_mean);
        s_x_yp += (point_pairs[i][0](0) - x_mean) * (point_pairs[i][1](1) - yp_mean);
        s_y_xp += (point_pairs[i][0](1) - y_mean) * (point_pairs[i][1](0) - xp_mean);
    }

    double rot_angle = atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp);
    double translation_x = xp_mean - (x_mean * cos(rot_angle) - y_mean * sin(rot_angle));
    double translation_y = yp_mean - (x_mean * sin(rot_angle) + y_mean * cos(rot_angle));

    rotation_ << cos(rot_angle), -sin(rot_angle), sin(rot_angle), cos(rot_angle);
    trans_ << translation_x, translation_y;

    Eigen::Matrix3d T;
    T << rotation_, trans_, 0, 0, 1;
    T_ = T * T_;
}

void ICP::print_rotation()
{
    cout << "rotations " << endl;
    cout << rotation_ << endl;
}

void ICP::print_trans()
{
    cout << "trans " << endl;
    cout << trans_ << endl;
}

void ICP::print_T()
{
    cout << "T " << endl;
    cout << T_ << endl;
}

double ICP::get_transx()
{
    return T_(0, 2);
}

double ICP::get_transy()
{
    return T_(0, 2);
}

double ICP::get_rotation()
{
    return acos(T_(0, 0));
}