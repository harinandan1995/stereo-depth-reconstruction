//
// Created by harinandan on 06.01.19.
//

#include <iostream>
#include "Triangulation.h"

namespace stereo_depth {

    Triangulation::Triangulation(Eigen::Matrix3f left_camera_intrinsic_parameters,
                                 Eigen::Matrix3f right_camera_intrinsic_parameters) {

        this->left_camera_intrinsic_parameters = left_camera_intrinsic_parameters;
        this->right_camera_intrinsic_parameters = right_camera_intrinsic_parameters;

        this->baseline = this->left_camera_intrinsic_parameters(0, 2) - this->right_camera_intrinsic_parameters(0, 2);
        this->left_focal_length = this->left_camera_intrinsic_parameters(0, 0);

    }

    float Triangulation::getDepthFromDisparity(int disparity) {

        return  disparity;

        if(disparity == 0) return 0;

        return abs(left_focal_length*baseline/disparity);

    }

}
