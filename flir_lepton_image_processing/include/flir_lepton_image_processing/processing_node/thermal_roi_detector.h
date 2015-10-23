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
 * Authors: Alexandros Philotheou, Manos Tsardoulias,
 * Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_PROCESSING_NODE_THERMAL_ROI_DETECTOR_H
#define FLIR_LEPTON_IMAGE_PROCESSING_PROCESSING_NODE_THERMAL_ROI_DETECTOR_H

#include "utils/blob_detection.h"
#include "utils/thermal_rois_conveyor.h"
#include "utils/roi_filters.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton
{
namespace flir_lepton_image_processing
{
  /**
    @class RoiDetector
    @brief Provides the functionalities for detecting thermal Rois via analysis
    of a thermal image acquired by Flir Lepton camera.
   **/
  class RoiDetector
  {
    public:
      /**
        @brief Finds the Rois in a thermal image in CV_8UC1 format.
        First, the edges of the thermal image are detected.
        Then, keypoints of blobs are detected in the above image.
        Finally, the potential Rois outline is found, along with the bounding
        boxes of those outlines.
        @param[in] thermalImage [const cv::Mat&] The thermal image in CV_8UC1 format
        @return RoisConveyor The struct that contains the rois found.
       **/
      static RoisConveyor findRois(const cv::Mat& thermalImage);
  };

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_PROCESSING_NODE_THERMAL_ROI_DETECTOR_H
