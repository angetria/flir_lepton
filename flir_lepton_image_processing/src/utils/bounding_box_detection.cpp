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

#include "utils/bounding_box_detection.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton_rpi2
{
namespace flir_lepton_image_processing
{
  /**
    @brief Finds rotated bounding boxes from blob outlines.
    The blob's area must be larger than
    Parameters::bounding_box_min_area_threshold.
    The blob and its bounding rectangle must be inside the image's limits.
    @param[in] inImage [const cv::Mat&] The input image
    @param[in] blobsOutlineVector
    [const std::vector<std::vector<cv::Point2f> >&]
    The outline points of the blobs
    @param[in] blobsArea [const std::vector<float>&] The blobs' area
    @param[out] outRectangles [std::vector< std::vector<cv::Point2f> >*]
    The rectangles of the bounding boxes
    @return void
   **/
  void BoundingBoxDetection::findRotatedBoundingBoxesFromOutline(
    const cv::Mat& inImage,
    const std::vector<std::vector<cv::Point2f> >& blobsOutlineVector,
    const std::vector<float>& blobsArea,
    std::vector<std::vector<cv::Point2f> >* outRectangles)
  {
    #ifdef DEBUG_TIME
    Timer::start("findRotatedBoundingBoxesFromOutline", "validateBlobs");
    #endif

    // Find the rotated rectangles for each blob based on its outline
    std::vector<cv::RotatedRect> minRect;
    for (unsigned int i = 0; i < blobsOutlineVector.size(); i++)
    {
      // The area of the blob should be greater than a threshold,
      // so that tiny formations of pixels are not identified as blobs
      if (blobsArea[i] >= Parameters::Blob::min_area)
      {
        minRect.push_back(minAreaRect(cv::Mat(blobsOutlineVector[i])));
      }
    }


    // For each rotated rectangle whose corresponding blob exceeds the minimum
    // area threshold, if its vertices reside within the image's boundaries,
    // store its vertices
    for (unsigned int i = 0; i < minRect.size(); i++)
    {
      // The for vertices of the rotated rectangle
      cv::Point2f rect_points[4];
      minRect[i].points(rect_points);

      // Check if the vertices reside in the image's boundaries
      int numVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        if (rect_points[j].x < inImage.cols &&
          rect_points[j].x >= 0 &&
          rect_points[j].y < inImage.rows &&
          rect_points[j].y >= 0)
        {
          numVerticesWithinImageLimits++;
        }
      }

      // If the rotated rectangle's edges reside outside the image's edges,
      // discard this rotated rectangle
      if (numVerticesWithinImageLimits < 4)
      {
        continue;
      }

      // If all four vertices reside within the image's boundaries,
      // store them in their respective position

      // Same as rect_points array, but vector
      std::vector<cv::Point2f> rect_points_vector;

      for (int j = 0; j < 4; j++)
      {
        rect_points_vector.push_back(rect_points[j]);
      }

      // Push back the 4 vertices of rectangle i
      outRectangles->push_back(rect_points_vector);
    }

    #ifdef DEBUG_TIME
    Timer::tick("findRotatedBoundingBoxesFromOutline");
    #endif
  }

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton_rpi2
