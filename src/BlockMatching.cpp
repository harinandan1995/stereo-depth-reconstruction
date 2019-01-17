//
// Created by harinandan on 05.01.19.
//

#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include <iostream>
#include <algorithm>
#include "BlockMatching.h"

namespace stereo_depth
{

    BlockMatching::BlockMatching(int block_range, int IMAGE_WIDTH, int IMAGE_HEIGHT,
                                 int MAX_DISPARITY) {

        this->block_range = block_range;
        this->IMAGE_WIDTH = IMAGE_WIDTH;
        this->IMAGE_HEIGHT = IMAGE_HEIGHT;
        this->MAX_DISPARITY = MAX_DISPARITY;

    }

    double BlockMatching::getSADIntensities(cv::Point2i left_point, cv::Point2i right_point, cv::Mat &left_image,
                                      cv::Mat &right_image) {

        //std::cout << left_point.x << " " << left_point.y << " " << right_point.x << " " << right_point.y << std::endl;

        int negative_range_x = std::min(std::min(left_point.x, block_range), right_point.x);
        int negative_range_y = std::min(std::min(left_point.y, block_range), right_point.y);

        int positive_range_x = std::min(std::min(IMAGE_HEIGHT - left_point.x - 1, block_range), IMAGE_HEIGHT - right_point.x - 1);
        int positive_range_y = std::min(std::min(IMAGE_WIDTH - left_point.y - 1, block_range), IMAGE_WIDTH - right_point.y - 1);

        //std::cout << negative_range_x << " " << negative_range_y << " " << positive_range_x << " " << positive_range_y << std::endl;

        double sad = 0;

        for(int i = -negative_range_x; i < positive_range_x+1; i++) {
            for(int j = -negative_range_y; j < positive_range_y+1; j++) {
                //std::cout<< (double) left_image.at<uchar>(left_point.x + i, left_point.y + j) <<std::endl;

                sad += abs((double)left_image.at<uchar>(left_point.x + i, left_point.y + j) - (double)right_image.at<uchar>(right_point.x + i, right_point.y + j) );
                //std::cout << sad << std::endl;
            }
        }

        return sad;
    }

    Eigen::MatrixXi BlockMatching::generateDisparityMap(cv::Mat &left_image, cv::Mat &right_image) {

        Eigen::MatrixXi disparity_map = Eigen::MatrixXi::Zero(IMAGE_HEIGHT, IMAGE_WIDTH);
        cv::Mat dx_left, dx_right;

        cv::Sobel(left_image, dx_left, CV_64F, 1, 0, 5);
        cv::Sobel(right_image, dx_right, CV_64F, 1, 0, 5);

        int counter = 0, total = 0;

        for(int x = 0; x < IMAGE_HEIGHT; x++) {
            for(int y = 0; y < IMAGE_WIDTH; y++) {
                total++;
                if(abs(dx_left.at<double>(x, y)) < 1500) continue;

                counter++;
                double min_SADIntensity = std::numeric_limits<double>::max();

                cv::Point2i left_point(x, y);

                for(int k = 0; k >= -MAX_DISPARITY; k--) {
                    if (y+k <0 || y+k >= IMAGE_WIDTH) continue;

                    //if(abs(dx_right.at<double>(x, y+k)) < 2500) continue;

                    cv::Point2i right_point(x, y+k);

                    double SADIntensity = getSADIntensities(left_point, right_point, left_image, right_image);

                    if (SADIntensity < min_SADIntensity) {
                        min_SADIntensity = SADIntensity;
                        disparity_map(x, y) = k;
                    }

                }
            }
        }

        std::cout << counter << "/" << total << std::endl;
        std::cout << disparity_map.minCoeff() << " " << disparity_map.maxCoeff() << " " << disparity_map.mean() << " " << disparity_map.rows() << " " << disparity_map.cols() << std::endl;

        return disparity_map;

    }

}