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

    cv::Mat BlockMatching::computeLeftBlock(cv::Point2i left_point, cv::Mat &left_image) {

        int negative_range_x = std::min(left_point.x, block_range);
        int negative_range_y = std::min(left_point.y, block_range);

        int positive_range_x = std::min(IMAGE_HEIGHT - left_point.x - 1, block_range);
        int positive_range_y = std::min(IMAGE_WIDTH - left_point.y - 1, block_range);

        //std::cout << negative_range_x << " " << negative_range_y << " " << positive_range_x << " " << positive_range_y << std::endl;

        cv::Range left_block_range_x = cv::Range(left_point.x-negative_range_x, left_point.x + positive_range_x);
        cv::Range left_block_range_y = cv::Range(left_point.y-negative_range_y, left_point.y + positive_range_y);

        return left_image(left_block_range_x, left_block_range_y);

    }

    cv::Mat BlockMatching::computeRightBlock(cv::Point2i left_point, cv::Point2i right_point, cv::Mat &right_image) {

        int negative_range_x = std::min(std::min(left_point.x, block_range), right_point.x);
        int negative_range_y = std::min(std::min(left_point.y, block_range), right_point.y);

        int positive_range_x = std::min(std::min(IMAGE_HEIGHT - left_point.x - 1, block_range), IMAGE_HEIGHT - right_point.x - 1);
        int positive_range_y = std::min(std::min(IMAGE_WIDTH - left_point.y - 1, block_range), IMAGE_WIDTH - right_point.y - 1);

        cv::Range right_block_range_x = cv::Range(right_point.x-negative_range_x, right_point.x + positive_range_x);
        cv::Range right_block_range_y = cv::Range(right_point.y-negative_range_y, right_point.y + positive_range_y);

        return right_image(right_block_range_x, right_block_range_y);

    }

    double computeSAD(cv::Mat left_block, cv::Mat right_block) {

        cv::Mat intensity_diff;
        cv::absdiff(left_block, right_block, intensity_diff);

        return cv::sum(intensity_diff)[0];

    }

    double BlockMatching::getSADIntensities(cv::Point2i left_point, cv::Point2i right_point, cv::Mat &left_image,
                                      cv::Mat &right_image) {

        //std::cout << left_point.x << " " << left_point.y << " " << right_point.x << " " << right_point.y << std::endl;

        int negative_range_x = std::min(std::min(left_point.x, block_range), right_point.x);
        int negative_range_y = std::min(std::min(left_point.y, block_range), right_point.y);

        int positive_range_x = std::min(std::min(IMAGE_HEIGHT - left_point.x - 1, block_range), IMAGE_HEIGHT - right_point.x - 1);
        int positive_range_y = std::min(std::min(IMAGE_WIDTH - left_point.y - 1, block_range), IMAGE_WIDTH - right_point.y - 1);

        //std::cout << negative_range_x << " " << negative_range_y << " " << positive_range_x << " " << positive_range_y << std::endl;

        cv::Range left_block_range_x = cv::Range(left_point.x-negative_range_x, left_point.x + positive_range_x);
        cv::Range left_block_range_y = cv::Range(left_point.y-negative_range_y, left_point.y + positive_range_y);

        cv::Range right_block_range_x = cv::Range(right_point.x-negative_range_x, right_point.x + positive_range_x);
        cv::Range right_block_range_y = cv::Range(right_point.y-negative_range_y, right_point.y + positive_range_y);

        // cv::Rect left_block_range(left_point.x-negative_range_x, left_point.y-negative_range_y, negative_range_x + positive_range_x + 1, negative_range_y + positive_range_y + 1);

        //std::cout << left_block_range_x << left_block_range_y << right_block_range_x << right_block_range_y << std::endl;

        cv::Mat left_block = left_image(left_block_range_x, left_block_range_y);
        cv::Mat right_block = right_image(right_block_range_x, right_block_range_y);

        cv::Mat intensity_diff;
        cv::absdiff(left_block, right_block, intensity_diff);

        return cv::sum(intensity_diff)[0];

    }

    Eigen::MatrixXi BlockMatching::generateDisparityMap(cv::Mat left_image, cv::Mat right_image) {

        Eigen::MatrixXi disparity_map = Eigen::MatrixXi::Zero(IMAGE_HEIGHT, IMAGE_WIDTH);
        cv::Mat dx_left, dx_right;

        cv::Sobel(left_image, dx_left, CV_64F, 1, 0, 5);
        cv::Sobel(right_image, dx_right, CV_64F, 1, 0, 5);
        //cv::Sobel(left_image, dy, CV_64F, 0, 1, 5);

        int counter = 0, total = 0;

        for(int x = 0; x < IMAGE_HEIGHT; x++) {
            for(int y = 0; y < IMAGE_WIDTH; y++) {

                total++;
                // std::cout << abs(dx.at<double>(x, y)) <<std::endl;

                if(abs(dx_left.at<double>(x, y)) < 3000) continue;

                counter++;
                double min_SADIntensity = std::numeric_limits<double>::max();


                cv::Point2i left_point(x, y);
                cv::Mat left_block = computeLeftBlock(left_point, left_image);

                for(int k = 0; k >= -MAX_DISPARITY; k--) {
                    if (y+k <0 || y+k >= IMAGE_WIDTH) continue;

                    if(abs(dx_right.at<double>(x, y+k)) < 3000) continue;

                    double SADIntensity;
                    cv::Point2i right_point(x, y+k);

                    int negative_range_x = std::min(std::min(left_point.x, block_range), right_point.x);
                    int negative_range_y = std::min(std::min(left_point.y, block_range), right_point.y);

                    int positive_range_x = std::min(std::min(IMAGE_HEIGHT - left_point.x - 1, block_range), IMAGE_HEIGHT - right_point.x - 1);
                    int positive_range_y = std::min(std::min(IMAGE_WIDTH - left_point.y - 1, block_range), IMAGE_WIDTH - right_point.y - 1);

                    if(negative_range_x != block_range || negative_range_y != block_range || positive_range_x != block_range || positive_range_y != block_range) {
                        SADIntensity = getSADIntensities(cv::Point2i(x, y), cv::Point2i(x, y+k), left_image, right_image);
                    } else {
                        cv::Mat right_block = computeRightBlock(left_point, right_point, right_image);
                        SADIntensity = computeSAD(left_block, right_block);
                    }


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