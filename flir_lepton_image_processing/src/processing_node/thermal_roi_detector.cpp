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

#include "processing_node/thermal_roi_detector.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton_rpi2
{
namespace flir_lepton_image_processing
{
  /**
    @brief Finds the Rois in the thermal image in CV_8UC1 format.
    First, the edges of the thermal image are detected.
    Then, keypoints of blobs are detected in the above image.
    Finally, the potential Rois outline is found, along with the bounding
    boxes of those outlines.
    @param[in] thermalImage [const cv::Mat&] The thermal
    image in CV_8UC1 format.
    @return RoisConveyor The struct that contains the rois found.
   **/
  RoisConveyor RoiDetector::findRois(const cv::Mat& thermalImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("findRois", "inputThermalImageCallback");
    #endif

    #ifdef DEBUG_SHOW
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    if (Parameters::Debug::show_find_rois)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Thermal image";
      msgs.push_back(msg);
      cv::Mat tmp = Visualization::scaleImageForVisualization(
        thermalImage, Parameters::Image::scale_method);
      imgs.push_back(tmp);
    }
    #endif

    // Detect edges in the thermal image
    cv::Mat thermalImageEdges;
    EdgeDetection::computeThermalEdges(thermalImage,
      &thermalImageEdges);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_find_rois)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Edges after denoise");
      msgs.push_back(msg);
      cv::Mat tmp;
      thermalImageEdges.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // Find blobs in the edges image. Each blob is represented as
    // a keypoint which is the center of the blob found
    std::vector<cv::KeyPoint> keyPoints;
    BlobDetection::detectBlobs(thermalImageEdges, &keyPoints);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_find_rois)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Initial keypoints");
      msgs.push_back(msg);
      imgs.push_back(
        Visualization::showKeypoints(msg, thermalImageEdges, -1,
          keyPoints));
    }
    #endif

    // The final vectors of keypoints, rectangles and blobs' outlines.
    RoisConveyor conveyor;

    /**
      Get me blobs that their center point is inside the image,
      their bounding box is also entirely inside the image, and their area is
      greater than Parameters::bounding_box_min_area_threshold.
      Each keypoint is associated with exactly one rectangle.
      The end product here is a set of keypoints, a set of rectangles that
      enclose them and a set of the outlines of the blobs found, all tightly
      packed in the conveyor struct.
     **/
    RoiFilters::validateBlobs(
      keyPoints,
      &thermalImageEdges,
      Parameters::Outline::outline_detection_method,
      &conveyor);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_find_rois)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Blobs");
      msgs.push_back(msg);
      imgs.push_back(
        Visualization::showRois(
          msg,
          thermalImage,
          conveyor,
          -1,
          std::vector<std::string>()));
    }
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_find_rois)  // Debug
    {
      // A vector of keypoints
      std::vector<cv::KeyPoint> keypointsVector;

      for (int i = 0; i < conveyor.size(); i++)
      {
        keypointsVector.push_back(conveyor.rois[i].keypoint);
      }

      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Keypoints found from this process");
      msgs.push_back(msg);
      imgs.push_back(
        Visualization::showKeypoints(
          msg,
          thermalImage,
          -1,
          keypointsVector));
    }
    if (Parameters::Debug::show_find_rois)
    {
      Visualization::multipleShow("Thermal node", imgs, msgs,
        Parameters::Debug::show_find_rois_size, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("findRois");
    #endif

    return conveyor;
  }

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton_rpi2
