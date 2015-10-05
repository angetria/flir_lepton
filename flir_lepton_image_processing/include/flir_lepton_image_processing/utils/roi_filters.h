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

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_UTILS_ROI_FILTERS_H
#define FLIR_LEPTON_IMAGE_PROCESSING_UTILS_ROI_FILTERS_H

#include "utils/edge_detection.h"
#include "utils/outline_discovery.h"
#include "utils/bounding_box_detection.h"
#include "utils/thermal_rois_conveyor.h"
#include <math.h>

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class RoiFilters
    @brief Provides methods for validating blobs
   **/
  class RoiFilters
  {
    public:
      /**
        @brief Given a set of keypoints and an edges image, this function
        returns the valid keypoints and for each one, its respective, least
        area rotated bounding box and the points of its outline.
        @param[in] keyPoints [const std::vector<cv::KeyPoint>&]
        The original keypoints found.
        @param[in] thermalImageEdges [cv::Mat*] The original thermal edges image
        @param[in] detectionMethod [const int&] The method by which the
        outline of a blob is obtained.
        0 means by means of brushfire,
        1 by means of raycasting
        @param[in,out] conveyor [RoisConveyor*] A struct that contains
        the final valid candidate rois
        @return void
       **/
      static void validateBlobs(
        const std::vector<cv::KeyPoint>& keyPoints,
        cv::Mat* thermalImageEdges,
        const int& detectionMethod,
        RoisConveyor* conveyor);

      /**
        @brief This functions takes as input arguments a keypoints vector of
        size N, a rectangles vector of size M (the rectangle is represented
        by its 4 vertices so that the input can be either a Rectangle or a
        Rotated Rectangle) and a vector with the area of each rectangle
        with purpose to identify the least area rectangle in which a
        keypoint resides. It outputs a vector of keypoints (each keypoint must
        reside in at least one rectangle) and a vector of rectangles (again
        represented by its 4 vertices). There is a one-to-one association
        between the keypoints and the rectangles.
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The key points
        @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
        The rectangles found
        @param[in] inRectanglesArea [const std::vector<float>&] The area of each
        rectangle
        @param[in] inContours [const std::vector<std::vector<cv::Point2f> >&]
        The outline of each blob found
        @param[out] conveyor [RoisConveyor*] The container of vector of blobs'
        keypoints, outlines and areas
        @return void
       **/
      static void validateKeypointsToRectangles(
        const std::vector<cv::KeyPoint>& inKeyPoints,
        const std::vector<std::vector<cv::Point2f> >& inRectangles,
        const std::vector<float>& inRectanglesArea,
        const std::vector<std::vector<cv::Point2f> >& inContours,
        RoisConveyor* conveyor);
  };

}  // namespace pandora_vision

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_UTILS_ROI_FILTERS_H
