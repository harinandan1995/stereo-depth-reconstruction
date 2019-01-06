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


    public:

        StereoDataset(int num_frames, std::string path);

        int getNumberOfFrames();

        int getNumberOfFramesLeft();

        // Left image will be at 0 index, right image will be at 1 index
        std::vector<cv::Mat> getNextFrame();

    };

}

#endif //STEREO_DEPTH_RECONSTRUCTION_DATASET_H
