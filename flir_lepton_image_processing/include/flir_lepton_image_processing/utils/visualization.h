/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_UTILS_VISUALIZATION_H
#define FLIR_LEPTON_IMAGE_PROCESSING_UTILS_VISUALIZATION_H

#include "utils/parameters.h"
#include "utils/thermal_rois_conveyor.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton_rpi2
{
namespace flir_lepton_image_processing
{
  /**
    @class Visualization
    @brief Provides methods for image visualization
   **/
  class Visualization
  {
    public:
      /**
        @brief Shows multiple images in one window
        @param[in] title [const std::string&] The window's title
        @param[in] imgs [const std::vector<cv::Mat>&] The images to be shown
        @param[in] titles [const std::vector<std::string>&] The titles for each
        image
        @param[in] maxSize [const int&] The maximum size of the window
        @param[in] ms [const int&] How many seconds the showing lasts
        @return [cv::Mat] An image featuring multiple images
       **/
      static cv::Mat multipleShow(
        const std::string& title,
        const std::vector<cv::Mat>& imgs,
        const std::vector<std::string>& titles,
        const int& maxSize,
        const int& ms);

      /**
        @brief Scales an image from its original format to CV_8UC1
        @param[in] inImage [const cv::Mat&] The image to show
        @param[in] ms [const int&] How many ms the showing lasts
        @return void
       **/
      static cv::Mat scaleImageForVisualization(
        const cv::Mat& inImage,
        const int& method);

      /**
        @brief Overrides the cv::imshow function.
        @param[in] windowTitle [const std::string&] The window title
        @param[in] inImage [const cv::Mat&] The image to show
        @param[in] ms [const int&] How many ms the showing lasts
        @return void
       **/
      static void show(
        const std::string& windowTitle,
        const cv::Mat& inImage,
        const int& ms);

      /**
        @brief Depicts the contents of a RoisConveyor on an image
        @param[in] windowTitle [const std::string&] The window title
        @param[in] inImage [const cv::Mat&] The image to show
        @param[in] conveyor [const RoisConveyor&] The rois conveyor
        @param[in] ms [const int&] How many ms the showing lasts
        @param[in] msgs [const std::vector<std::string>&] Message to show to
        each keypoint
        @param[in] hz [const float&] If positive holds the Hz
        @return [cv::Mat] The drawn image
       **/
      static cv::Mat showRois(
        const std::string& windowTitle,
        const cv::Mat& inImage,
        const RoisConveyor& conveyor,
        const int& ms,
        const std::vector<std::string>& msgs,
        const float& hz = -1);

      /**
        @brief Depicts the keypoints and bounding boxes
        @param[in] windowTitle [const std::string&] The window title
        @param[in] inImage [const cv::Mat&] The image to show
        @param[in] ms [const int&] How many ms the showing lasts
        @param[in] keypoints [const std::vector<cv::KeyPoint>&] The keypoints
        @return void
       **/
      static cv::Mat showKeypoints(
        const std::string& windowTitle,
        const cv::Mat& inImage,
        const int& ms,
        const std::vector<cv::KeyPoint>& keypoints);

      /**
        @brief Overrides the cv::imshow function. Provides image scaling from
        the image's original forma to CV_8UC1 format
        @param[in] windowTitle [const std::string&] The window title
        @param[in] inImage [const cv::Mat&] The image to show
        @param[in] ms [const int&] How many ms the showing lasts
        @return void
       **/
      static void showScaled(
        const std::string& windowTitle,
        const cv::Mat& inImage,
        const int& ms);
  };

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton_rpi2

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_UTILS_VISUALIZATION_H
