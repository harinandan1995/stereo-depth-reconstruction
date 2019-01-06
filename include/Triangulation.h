//
// Created by harinandan on 06.01.19.
//

#ifndef STEREO_DEPTH_RECONSTRUCTION_TRIANGULATION_H
#define STEREO_DEPTH_RECONSTRUCTION_TRIANGULATION_H

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace stereo_depth {

    class Triangulation{

        // In the form [f 0 cx; 0 f cy; 0 0 1] where f is the focal length and cx, cy is the principal point
        Eigen::Matrix3f left_camera_intrinsic_parameters;
        Eigen::Matrix3f right_camera_intrinsic_parameters;

        float baseline;
        float left_focal_length;

    public:

        Triangulation(Eigen::Matrix3f left_camera_intrinsic_parameters, Eigen::Matrix3f right_camera_intrinsic_parameters);

        float getDepthFromDisparity(int disparity);

    };

}

#endif //STEREO_DEPTH_RECONSTRUCTION_TRIANGULATION_H
