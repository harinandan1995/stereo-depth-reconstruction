#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <Dataset.h>
#include <DepthEstimation.h>

#ifndef N_FRAMES
#define N_FRAMES 1 // 200
#endif


const std::string DATASET_PATH = "../dataset/Cable-perfect";
const int IMAGE_WIDTH = 12;
const int IMAGE_HEIGHT = 12;

int main() {

    stereo_depth::StereoDataset *stereoDataset = new stereo_depth::StereoDataset(N_FRAMES, DATASET_PATH);
    stereo_depth::DepthReconstruction *depthReconstruction = new stereo_depth::DepthReconstruction(IMAGE_WIDTH, IMAGE_HEIGHT, stereoDataset->getLeftCameraIntrinsics(),
            stereoDataset->getRightCameraIntrinsics());

    while (stereoDataset->getNumberOfFramesLeft() > 0) {

        std::vector<cv::Mat> next_frame = stereoDataset->getNextFrame();

        cv::imshow( "Left Image", next_frame[0]);
        cv::imshow( "Right Image", next_frame[1]);

        cv::imshow("Depth Image", depthReconstruction->getDepthMapFromStereoImages(next_frame[0], next_frame[1]));

        cv::waitKey(0);

    }

    std::cout << "Finished estimating depth for " << stereoDataset->getNumberOfFrames() << std::endl;

    return 0;
}