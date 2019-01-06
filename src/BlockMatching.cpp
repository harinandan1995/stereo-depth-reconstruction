//
// Created by harinandan on 05.01.19.
//

#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
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

        if(current_point.x + max_diff_x < IMAGE_WIDTH) {
            bottomRightPoint.x = current_point.x + max_diff_x;
        } else {
            bottomRightPoint.x = IMAGE_WIDTH-1;
        }

        if(current_point.y + max_diff_y < IMAGE_HEIGHT) {
            bottomRightPoint.y = current_point.y + max_diff_y;
        } else {
            bottomRightPoint.y = IMAGE_HEIGHT-1;
        }

        return bottomRightPoint;

    }

    double BlockMatching::getSADIntensities(cv::Point2i left_point, cv::Point2i right_point, cv::Mat &left_image,
                                      cv::Mat &right_image) {

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

        Eigen::MatrixXi disparity_map = Eigen::MatrixXi::Constant(IMAGE_WIDTH, IMAGE_HEIGHT, 0);;

        for(int i = 0; i < IMAGE_WIDTH; i++) {
            for(int j = 0; j < IMAGE_WIDTH; j++) {

                double min_SADintensity = std::numeric_limits<double>::max();

                for(int k = -MAX_DISPARITY; k < MAX_DISPARITY; k++) {
                    if (k == 0) continue;
                    if (i+k <0 || i+k >= IMAGE_WIDTH) continue;

                    double SADIntensity = getSADIntensities(cv::Point2i(i, j), cv::Point2i(i+k, j), left_image, right_image);
                    if (SADIntensity < min_SADintensity) {
                        disparity_map(j, i) = k;
                    }

                }
            }
        }

        return disparity_map;

    }

}