//
// Created by harinandan on 05.01.19.
//

#ifndef STEREO_DEPTH_RECONSTRUCTION_BLOCKMATCHING_H
#define STEREO_DEPTH_RECONSTRUCTION_BLOCKMATCHING_H

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace stereo_depth
{

    class BlockMatching {

        int block_range;

        int IMAGE_WIDTH;
        int IMAGE_HEIGHT;

        int MAX_DISPARITY;

        double getSADIntensities(cv::Point2i left_point, cv::Point2i right_point, cv::Mat &left_image, cv::Mat &right_image);

    public:

        BlockMatching(int block_range, int IMAGE_WIDTH, int IMAGE_HEIGHT, int MAX_DISPARITY);

        Eigen::MatrixXi generateDisparityMap(cv::Mat left_image, cv::Mat right_image);

    };

}

#endif //STEREO_DEPTH_RECONSTRUCTION_BLOCKMATCHING_H
