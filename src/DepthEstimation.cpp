//
// Created by harinandan on 06.01.19.
//

#include "DepthEstimation.h"

namespace stereo_depth
{

    const int BLOCK_SIZE_X = 7;
    const int BLOCK_SIZE_Y = 7;
    const int MAX_DISPARITY = 16;

    DepthReconstruction::DepthReconstruction(int IMAGE_WIDTH, int IMAGE_HEIGHT, Eigen::Matrix3f left_camera_intrinsic_parameters,
                                             Eigen::Matrix3f right_camera_intrinsic_parameters) {

        this->IMAGE_WIDTH = IMAGE_WIDTH;
        this->IMAGE_HEIGHT = IMAGE_HEIGHT;
        this->left_camera_intrinsic_parameters = left_camera_intrinsic_parameters;
        this->right_camera_intrinsic_parameters = right_camera_intrinsic_parameters;

        this->blockMatching = new BlockMatching(BLOCK_SIZE_X, BLOCK_SIZE_Y, IMAGE_WIDTH, IMAGE_HEIGHT, MAX_DISPARITY);
        this->triangulation = new Triangulation(left_camera_intrinsic_parameters, right_camera_intrinsic_parameters);

    }

    cv::Mat DepthReconstruction::getDepthMapFromStereoImages(cv::Mat &left_image, cv::Mat &right_image) {

        Eigen::MatrixXi disparity_map = blockMatching->generateDisparityMap(left_image, right_image);

        cv::Mat depth_map = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32F);

        for (int i = 0; i < IMAGE_HEIGHT; i++) {
            for (int j = 0; j < IMAGE_WIDTH; j++) {

                depth_map.at<uchar>(i, j) = static_cast<unsigned char>(triangulation->getDepthFromDisparity(disparity_map(i, j)));

            }
        }

        cv::normalize(depth_map, depth_map, 255, 0, cv::NORM_L2);

        return depth_map;

    }

}