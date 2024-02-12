#include <iostream>
#include "opencv2/opencv.hpp"
#include "ICP.h"

using std::cout;
using std::endl;

std::vector<Eigen::Vector2f> scan1, scan2;

int main() {
	ICP icp;

    cv::Mat img1, img2;
    img1 = cv::imread("img/lidar1.png", cv::IMREAD_GRAYSCALE);
    img2 = cv::imread("img/lidar2.png", cv::IMREAD_GRAYSCALE);

    while(true){
        scan1.clear();
        std::vector<Eigen::Vector2f>().swap(scan1);
        scan2.clear();
        std::vector<Eigen::Vector2f>().swap(scan2);
        for (int i = 0; i < img1.rows; i++) {
            for (int j = 0; j < img1.cols; j++) {
                if (img1.data[i * img1.cols + j] == 255) {
                    scan1.push_back(Eigen::Vector2f(j, i));
                }
            }
        }

        for (int i = 0; i < img2.rows; i++) {
            for (int j = 0; j < img2.cols; j++) {
                if (img2.data[i * img2.cols + j] == 255) {
                    scan2.push_back(Eigen::Vector2f(j, i));
                }
            }
        }

        cv::Mat src(300, 300, CV_8UC3, cv::Scalar(0, 0, 0)), dst(300, 300, CV_8UC3, cv::Scalar(0, 0, 0));
        int x, y;

        for (int i = 0; i < scan1.size(); i++) {
            x = scan1[i](0);
            y = scan1[i](1);
            cv::circle(src, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
        }
        for (int i = 0; i < scan2.size(); i++) {
            x = scan2[i](0);
            y = scan2[i](1);
            cv::circle(src, cv::Point(x, y), 1, cv::Scalar(255, 0, 0), -1);
        }
        cv::resize(src, src, cv::Size(600, 600));
        cv::imshow("src", src);

	    icp.solve(scan1, scan2);

        for (int i = 0; i < scan1.size(); i++) {
            x = scan1[i](0);
            y = scan1[i](1);
            cv::circle(dst, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
        }
        for (int i = 0; i < scan2.size(); i++) {
            x = scan2[i](0);
            y = scan2[i](1);
            cv::circle(dst, cv::Point(x, y), 1, cv::Scalar(255, 0, 0), -1);
        }
        cv::resize(dst, dst, cv::Size(600, 600));
        cv::imshow("dst", dst);
        cv::waitKey(10);
    }

	return 0;
}