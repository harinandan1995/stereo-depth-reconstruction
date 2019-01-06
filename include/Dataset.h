//
// Created by harinandan on 06.01.19.
//

#ifndef STEREO_DEPTH_RECONSTRUCTION_DATASET_H
#define STEREO_DEPTH_RECONSTRUCTION_DATASET_H

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace stereo_depth
{

    class StereoDataset {

        std::string path;
        int num_frames;
        int counter = 0;

        Eigen::Matrix3f left_camera_intrinsics;
        Eigen::Matrix3f right_camera_intrinsics;


    public:

        StereoDataset(int num_frames, std::string path);

        int getNumberOfFrames();

        int getNumberOfFramesLeft();

        Eigen::Matrix3f getLeftCameraIntrinsics();

        Eigen::Matrix3f getRightCameraIntrinsics();

        // Left image will be at 0 index, right image will be at 1 index
        std::vector<cv::Mat> getNextFrame();

    };

}

#endif //STEREO_DEPTH_RECONSTRUCTION_DATASET_H
