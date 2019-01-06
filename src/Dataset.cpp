//
// Created by harinandan on 06.01.19.
//

#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include "Dataset.h"

namespace stereo_depth
{

    StereoDataset::StereoDataset(int num_frames, std::string path) {

        this->num_frames = num_frames;
        this->path = path;

        this->left_camera_intrinsics << 6338.47, 0, 1035.675, 0, 6338.47, 960.073, 0, 0, 1;
        this->right_camera_intrinsics << 6338.47, 0, 1515.164, 0, 6338.47, 960.073, 0, 0, 1;

    }

    int StereoDataset::getNumberOfFrames() {

        return num_frames;

    }

    int StereoDataset::getNumberOfFramesLeft() {

        return num_frames - counter;

    }

    Eigen::Matrix3f StereoDataset::getLeftCameraIntrinsics() {

        return this->left_camera_intrinsics;

    }

    Eigen::Matrix3f StereoDataset::getRightCameraIntrinsics() {

        return this->right_camera_intrinsics;

    }

    std::vector<cv::Mat> StereoDataset::getNextFrame() {

        if (counter == num_frames) {

            std::cout << "No frames left to load" << std::endl;
            return std::vector<cv::Mat>();
        }

        std::vector<cv::Mat> next_frame(2);

        std::string filename_rgb_left_cam = std::string(path + "/") + "im0.png";
        cv::Mat gray_8u_left_cam = cv::imread(filename_rgb_left_cam, cv::IMREAD_GRAYSCALE);
        if (gray_8u_left_cam.empty()){
            std::cout << "Could not read left camera image for " << counter << std::endl;
            std::exit(-1);
        }
        next_frame[0] = gray_8u_left_cam;

        std::string filename_rgb_right_cam = std::string(path + "/") + "im1.png";
        cv::Mat gray_8u_right_cam = cv::imread(filename_rgb_left_cam, cv::IMREAD_GRAYSCALE);
        if (gray_8u_right_cam.empty()){
            std::cout << "Could not read left camera image for " << counter << std::endl;
            std::exit(-1);
        }
        next_frame[1] = gray_8u_right_cam;

        counter++;

        return next_frame;

    }

}
