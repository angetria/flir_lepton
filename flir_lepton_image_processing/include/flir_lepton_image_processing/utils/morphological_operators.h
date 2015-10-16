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

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_UTILS_MORPHOLOGICAL_OPERATORS_H
#define FLIR_LEPTON_IMAGE_PROCESSING_UTILS_MORPHOLOGICAL_OPERATORS_H

#include "utils/visualization.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton_rpi2
{
namespace flir_lepton_image_processing
{
  /**
    @class Morphology
    @brief Provides methods for image binary morphological operators
   **/
  class Morphology
  {
    public:
      /**
        @brief Performs steps of closing
        @param img [cv::Mat*] The input image in CV_8UC1 format
        @param steps [const int&] Number of operator steps
        @param visualize [const bool&] True for step-by-step visualization
        @return void
       **/
      static void closing(cv::Mat* img, const int& steps,
        const bool& visualize = false);

      /**
        @brief Performs steps of dilation
        @param img [cv::Mat*] The input image in CV_8UC1 format
        @param steps [const int&] Number of operator steps
        @param visualize [const bool&] True for step-by-step visualization
        @return void
       **/
      static void dilation(cv::Mat* img, const int& steps,
        const bool& visualize = false);

      /**
        @brief Performs steps of dilation. Each non-zero pixel is assigned
        the maximum value of its non-zero neighbors.
        @param img [cv::Mat&*] The input image in CV_8UC1 format
        @param steps [const int&] Number of operator steps
        @param visualize [const bool&] True for step-by-step visualization
        @return void
       **/
      static void dilationRelative(cv::Mat* img, const int& steps,
        const bool& visualize = false);

      /**
        @brief Performs steps of erosion
        @param img [cv::Mat&] The input image in CV_8UC1 format
        @param steps [const int&] Number of operator steps
        @param visualize [const bool&] True for step-by-step visualization
        @return void
       **/
      static void erosion(cv::Mat* img, const int& steps,
        const bool& visualize = false);

      /**
        @brief Checks if a kernel in a specific point in an image is satisfied
        Caution: this method presupposes that @param center does not lie on
        the edges of @param img
        @param kernel [const char [3][3]] The kernel
        @param img [const cv::Mat&] The input image (uchar)
        @param center [const cv::Point&] The center of the kernel
        @return bool : True on match
       **/
      static bool kernelCheck(const char kernel[3][3], const cv::Mat& img,
        const cv::Point& center);

      /**
        @brief Performs steps of opening
        @param img [cv::Mat*] The input image in CV_8UC1 format
        @param steps [const int&] Number of operator steps
        @param visualize [const bool&] True for step-by-step visualization
        @return void
       **/
      static void opening(cv::Mat* img, const int& steps,
        const bool& visualize = false);

      /**
        @brief Performs steps of strict pruning (removes more stuff)
        Caution: This method presupposes that the input image @param img is
        a thinned image
        @param img [cv::Mat*] The input image in CV_8UC1 format
        @param steps [const int&] Number of operator steps
        @return void
       **/
      static void pruningStrictIterative(cv::Mat* inImage, const int& steps);

      /**
        @brief Performs steps of thinning
        (http://homepages.inf.ed.ac.uk/rbf/HIPR2/thin.htm)
        @param inImage [const cv::Mat&] The input image in CV_8UC1 format
        @param outImage [cv::Mat*] The input image in CV_8UC1 format
        @param steps [const int&] Number of operator steps
        @param visualize [const bool&] True for step-by-step visualization
        @return void
       **/
      static void thinning(const cv::Mat& inImage, cv::Mat* outImage,
        const int& steps, const bool& visualize = false);
  };

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton_rpi2

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_UTILS_MORPHOLOGICAL_OPERATORS_H
