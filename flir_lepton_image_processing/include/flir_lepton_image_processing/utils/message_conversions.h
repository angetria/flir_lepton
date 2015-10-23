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
 * Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_UTILS_MESSAGE_CONVERSIONS_H
#define FLIR_LEPTON_IMAGE_PROCESSING_UTILS_MESSAGE_CONVERSIONS_H

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include "flir_lepton_ros_comm/CandidateRoisVectorMsg.h"
#include "utils/defines.h"
#include "utils/outline_discovery.h"
#include "utils/thermal_rois_conveyor.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton
{
namespace flir_lepton_image_processing
{
  /**
    @class MessageConversions
    @brief Provides methods for converting images from and to ROS messages
   **/
  class MessageConversions
  {
    public:
      /**
        @brief Converts a cv::Mat image into a sensor_msgs::Image message
        @param[in] image [const cv::Mat&] The image
        @param[in] encoding [const std::string&] The image message's encoding
        @param[in] msg [const sensor_msgs::Image&] A message needed for
        setting the output message's header by extracting its header
        @return [sensor_msgs::Image] The output image message
       **/
      static sensor_msgs::Image convertImageToMessage(
        const cv::Mat& image, const std::string& encoding,
        const sensor_msgs::Image& msg);

      /**
        @brief Constructs a flir_lepton_image_processing/CandidateRoisVectorMsg
        message
        @param[in] conveyor [RoisConveyor&] A struct containing
        vectors of the rois keypoints, bounding rectangles' vertices
        and blobs' outlines.
        @param[out] candidateRoisVector
        [std::vector<flir_lepton_ros_comm::CandidateRoisVectorMsg>*]
        The vector containing the conveyor's rois in
        flir_lepton_ros_comm::CandidateRoisVectorMsg format
        @return void
       **/
      static void createCandidateRoisVector(
        const RoisConveyor& conveyor,
        std::vector<flir_lepton_ros_comm::CandidateRoiMsg>*
        candidateRoisVector);

      /**
        @brief Constructs a flir_lepton_image_processing/CandidateRoisVectorMsg
        message
        @param[in] conveyor [RoisConveyor&] A struct containing
        vectors of the rois keypoints, bounding rectangles' vertices
        and blobs' outlines.
        @param[in] image [cv::Mat&] The image to be packed in the message
        @param[out] candidateRoisVectorMsg
        [flir_lepton_ros_comm::CandidateRoisVectorMsg*]
        The output message
        @param[in] encoding [std::string&] The image's encoding
        @param[in] msg [const sensor_msgs::Image&] Needed to extract
        its header and place it as the header of the output message
        @return void
       **/
      static void createCandidateRoisVectorMessage(
        const RoisConveyor& conveyor,
        const cv::Mat& image,
        flir_lepton_ros_comm::CandidateRoisVectorMsg*
        candidateRoisVectorMsg,
        const std::string& encoding,
        const sensor_msgs::Image& msg);

      /**
        @brief Extracts a cv::Mat image from a ROS image message
        @param[in] msg [const sensor_msgs::Image&] The input ROS image
        message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractImageFromMessage(
        const sensor_msgs::Image& msg, cv::Mat* image,
        const std::string& encoding);

      /**
        @brief Extracts a cv::Mat image from a custom ROS message  of type
        flir_lepton_image_processing::CandidateRoisVectorMsg
        @param[in] msg [const flir_lepton_ros_comm::CandidateRoisVectorMsg&]
        The input ROS message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractImageFromMessageContainer(
        const flir_lepton_ros_comm::CandidateRoisVectorMsg& msg,
        cv::Mat* image, const std::string& encoding);

      /**
        @brief Recreates the RoisConveyor struct for the candidate rois
        from the flir_lepton_image_processing::CandidateRoiMsg message
        @param[in] candidateRoisVector
        [const std::vector<flir_lepton_ros_comm::CandidateRoiMsg>&]
        The input candidate rois
        @param[out] conveyor [RoisConveyor*] The output conveyor
        struct
        @return void
       **/
      static void fromCandidateRoiMsgToConveyor(
        const std::vector<flir_lepton_ros_comm::CandidateRoiMsg>&
        candidateRoisVector,
        RoisConveyor* conveyor);

      /**
        @brief Unpacks the the RoisConveyor struct for the
        candidate rois.
        @param[in] roisMsg
        [flir_lepton_ros_comm::CandidateRoisVectorMsg&] The input
        candidate rois message obtained through the processor node
        @param[out] conveyor [RoisConveyor*] The output conveyor
        struct
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The encoding used for
        @return void
       **/
      static void unpackMessage(
        const flir_lepton_ros_comm::CandidateRoisVectorMsg& roisMsg,
        RoisConveyor* conveyor,
        cv::Mat* image,
        const std::string& encoding);

      /**
       @brief Convert the Float32MultiArray data to cv::Mat.
       Its cv format  will be CV_8UC1.
       @param[in] inArray [const std_msgs::Float32MultiArray&]
       The input MultiArray
       @return cv::Mat
       **/
      static cv::Mat convertFloat32MultiArrayToMat(
        const std_msgs::Float32MultiArray& inArray);
  };

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_UTILS_MESSAGE_CONVERSIONS_H
