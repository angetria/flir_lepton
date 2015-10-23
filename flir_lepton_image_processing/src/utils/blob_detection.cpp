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

#include "utils/blob_detection.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton
{
namespace flir_lepton_image_processing
{
  /**
    @brief Detects blobs in an image
    @param[in] inImage [const cv::Mat&] The input image
    @param[out] keyPointsOut [std::vector<cv::KeyPoint>*] The ouput
    @return void
   **/
  void BlobDetection::detectBlobs(const cv::Mat& inImage,
    std::vector<cv::KeyPoint>* keyPointsOut)
  {
    #ifdef DEBUG_TIME
    Timer::start("detectBlobs", "findHoles");
    #endif

    cv::SimpleBlobDetector::Params params;

    params.minThreshold = Parameters::Blob::min_threshold;  // 40;
    params.maxThreshold = Parameters::Blob::max_threshold;  // 60;
    params.thresholdStep = Parameters::Blob::threshold_step;

    params.minArea = Parameters::Blob::min_area;
    params.maxArea = Parameters::Blob::max_area;

    params.minConvexity = Parameters::Blob::min_convexity;  // 0.6;
    params.maxConvexity = Parameters::Blob::max_convexity;

    params.minInertiaRatio = Parameters::Blob::min_inertia_ratio;  // 0.5;

    params.maxCircularity = Parameters::Blob::max_circularity;
    params.minCircularity = Parameters::Blob::min_circularity;  // 0.3;

    params.filterByColor = Parameters::Blob::filter_by_color;
    params.filterByCircularity = Parameters::Blob::filter_by_circularity;

    cv::SimpleBlobDetector blobDetector(params);
    blobDetector.create("SimpleBlob");

    std::vector<cv::KeyPoint> keyPoints;

    // detect blobs. store their center point
    blobDetector.detect(inImage, keyPoints);

    for (int keypointId = 0; keypointId < keyPoints.size(); keypointId++)
    {
      // if the keypoint is out of image limits, discard it
      if (keyPoints[keypointId].pt.x < inImage.cols &&
        keyPoints[keypointId].pt.x >= 0 &&
        keyPoints[keypointId].pt.y < inImage.rows &&
        keyPoints[keypointId].pt.y >= 0)
      {
        keyPointsOut->push_back(keyPoints[keypointId]);
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("detectBlobs");
    #endif
  }

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton
