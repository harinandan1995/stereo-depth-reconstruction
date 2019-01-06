//
// Created by harinandan on 06.01.19.
//

#ifndef STEREO_DEPTH_RECONSTRUCTION_DEPTHESTIMATION_H
#define STEREO_DEPTH_RECONSTRUCTION_DEPTHESTIMATION_H


#include "BlockMatching.h"
#include "Triangulation.h"

namespace stereo_depth {

    class DepthReconstruction {

        int IMAGE_WIDTH;
        int IMAGE_HEIGHT;

        Eigen::Matrix3f left_camera_intrinsic_parameters;
        Eigen::Matrix3f right_camera_intrinsic_parameters;

        BlockMatching *blockMatching;
        Triangulation *triangulation;

    public:

        DepthReconstruction(int IMAGE_WIDTH, int IMAGE_HEIGHT, Eigen::Matrix3f left_camera_intrinsic_parameters,
                            Eigen::Matrix3f right_camera_intrinsic_parameters);

        cv::Mat getDepthMapFromStereoImages(cv::Mat &left_image, cv::Mat &right_image);

    };

}

#endif //STEREO_DEPTH_RECONSTRUCTION_DEPTHESTIMATION_H
