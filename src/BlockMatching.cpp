//
// Created by harinandan on 05.01.19.
//

#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include <iostream>
#include "BlockMatching.h"

namespace stereo_depth
{

    BlockMatching::BlockMatching(int block_size_x, int block_size_y, int IMAGE_WIDTH, int IMAGE_HEIGHT,
                                 int MAX_DISPARITY) {

        this->block_size_x = block_size_x;
        this->block_size_y = block_size_y;
        this->IMAGE_WIDTH = IMAGE_WIDTH;
        this->IMAGE_HEIGHT = IMAGE_HEIGHT;
        this->MAX_DISPARITY = MAX_DISPARITY;

    }

    cv::Point2i BlockMatching::getTopLeftPoint(cv::Point2i current_point) {

        cv::Point2i topLeftPoint;

        int max_diff_x = (int) floor(this->block_size_x/2);
        int max_diff_y = (int) floor(this->block_size_y/2);

        if(current_point.x - max_diff_x >= 0) {
            topLeftPoint.x = current_point.x - max_diff_x;
        } else {
            topLeftPoint.x = 0;
        }

        if(current_point.y - max_diff_y >= 0) {
            topLeftPoint.y = current_point.y - max_diff_y;
        } else {
            topLeftPoint.y = 0;
        }

        return topLeftPoint;

    }

    cv::Point2i BlockMatching::getBottomRightPoint(cv::Point2i current_point) {

        cv::Point2i bottomRightPoint;

        int max_diff_x = (int) ceil(this->block_size_x/2);
        int max_diff_y = (int) ceil(this->block_size_y/2);

        if(current_point.x + max_diff_x < IMAGE_HEIGHT) {
            bottomRightPoint.x = current_point.x + max_diff_x;
        } else {
            bottomRightPoint.x = IMAGE_HEIGHT-1;
        }

        if(current_point.y + max_diff_y < IMAGE_WIDTH) {
            bottomRightPoint.y = current_point.y + max_diff_y;
        } else {
            bottomRightPoint.y = IMAGE_WIDTH-1;
        }

        return bottomRightPoint;

    }

    double BlockMatching::getSADIntensities(cv::Point2i left_point, cv::Point2i right_point, cv::Mat &left_image,
                                      cv::Mat &right_image) {

        // std::cout << left_point.x << " " << left_point.y << " " << right_point.x << " " << right_point.y << std::endl;

        cv::Point2i tl_point_left = getTopLeftPoint(left_point);
        cv::Point2i br_point_left = getBottomRightPoint(left_point);

        cv::Point2i tl_point_right = getTopLeftPoint(right_point);
        cv::Point2i br_point_right = getBottomRightPoint(right_point);


        cv::Mat left_block = left_image(cv::Range(tl_point_left.x, br_point_left.x), cv::Range(tl_point_left.y, br_point_left.y));
        cv::Mat right_block = right_image(cv::Range(tl_point_right.x, br_point_right.x), cv::Range(tl_point_right.y, br_point_right.y));

        cv::Mat intensity_diff;
        cv::absdiff(left_block, right_block, intensity_diff);

        return cv::sum(intensity_diff )[0];

    }

    Eigen::MatrixXi BlockMatching::generateDisparityMap(cv::Mat left_image, cv::Mat right_image) {

        Eigen::MatrixXi disparity_map = Eigen::MatrixXi::Zero(IMAGE_HEIGHT, IMAGE_WIDTH);

        for(int x = 60; x < IMAGE_HEIGHT-60; x++) {
            for(int y = 60; y < IMAGE_WIDTH-60; y++) {
                double min_SADIntensity = std::numeric_limits<double>::max();

                for(int k = -MAX_DISPARITY; k <= 0; k++) {
                    if (y+k <0 || y+k >= IMAGE_WIDTH) continue;

                    double SADIntensity = getSADIntensities(cv::Point2i(x, y), cv::Point2i(x, y+k), left_image, right_image);
                    if (SADIntensity < min_SADIntensity) {
                        min_SADIntensity = SADIntensity;
                        disparity_map(x, y) = k;
                    }

                }
            }
        }

        std::cout << disparity_map.minCoeff() << " " << disparity_map.maxCoeff() << " " << disparity_map.mean() << " " << disparity_map.rows() << " " << disparity_map.cols() << std::endl;

        return disparity_map;

    }

}