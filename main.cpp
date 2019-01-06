#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <Dataset.h>

#ifndef N_FRAMES
#define N_FRAMES 1 // 200
#endif


const std::string dataPath = "../dataset/Cable-perfect";


int main() {

    stereo_depth::StereoDataset *stereoDataset = new stereo_depth::StereoDataset(N_FRAMES, dataPath);

    while (stereoDataset->getNumberOfFramesLeft() > 0) {

        std::vector<cv::Mat> next_frame = stereoDataset->getNextFrame();

        cv::imshow( "Left Image", next_frame[0]);
        cv::imshow( "Right Image", next_frame[1]);


        cv::waitKey(0);

    }

    std::cout << "Finished estimating depth for " << stereoDataset->getNumberOfFrames() << std::endl;

    return 0;
}