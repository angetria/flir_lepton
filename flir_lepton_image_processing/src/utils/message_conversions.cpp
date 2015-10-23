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

#include "utils/message_conversions.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton
{
namespace flir_lepton_image_processing
{
  /**
    @brief Converts a cv::Mat image into a sensor_msgs::Image message
    @param[in] image [const cv::Mat&] The image
    @param[in] encoding [const std::string&] The image message's encoding
    @param[in] msg [const sensor_msgs::Image&] A message needed for
    setting the output message's header by extracting its header
    @return [sensor_msgs::Image] The output image message
   **/
  sensor_msgs::Image MessageConversions::convertImageToMessage(
    const cv::Mat& image, const std::string& encoding,
    const sensor_msgs::Image& msg)
  {
    cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

    msgPtr->header = msg.header;
    msgPtr->encoding = encoding;
    msgPtr->image = image;

    return *msgPtr->toImageMsg();
  }

  /**
    @brief Constructs a flir_lepton_image_processing/CandidateRoisVectorMsg
    message
    @param[in] conveyor [RoisConveyor&] A struct containing
    vectors of the rois keypoints, bounding rectangles' vertices
    and blobs' outlines
    @param[out] candidateRoisVector
    [std::vector<flir_lepton_ros_comm::CandidateRoisVectorMsg>*]
    The vector containing the conveyor's rois in
    flir_lepton_image_processing::CandidateRoisVectorMsg format
    @return void
   **/
  void MessageConversions::createCandidateRoisVector(
    const RoisConveyor& conveyor,
    std::vector<flir_lepton_ros_comm::CandidateRoiMsg>* candidateRoisVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCandidateRoisVector");
    #endif

    // Fill the flir_lepton_ros_comm::CandidateRoisVectorMsg's
    // candidateRois vector
    for (unsigned int i = 0; i < conveyor.size(); i++)
    {
      flir_lepton_ros_comm::CandidateRoiMsg roiMsg;

      // Push back the keypoint
      roiMsg.keypointX = conveyor.rois[i].keypoint.pt.x;
      roiMsg.keypointY = conveyor.rois[i].keypoint.pt.y;

      // Push back the bounding rectangle's vertices
      for (int v = 0; v < conveyor.rois[i].rectangle.size(); v++)
      {
        roiMsg.verticesX.push_back(conveyor.rois[i].rectangle[v].x);
        roiMsg.verticesY.push_back(conveyor.rois[i].rectangle[v].y);
      }

      // Push back the blob's outline points
      for (int o = 0; o < conveyor.rois[i].outline.size(); o++)
      {
        roiMsg.outlineX.push_back(conveyor.rois[i].outline[o].x);
        roiMsg.outlineY.push_back(conveyor.rois[i].outline[o].y);
      }

      // Push back one roi to the rois vector message
      candidateRoisVector->push_back(roiMsg);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createCandidateRoisVector");
    #endif
  }

  /**
    @brief Constructs a flir_lepton_image_processing/CandidateRoisVectorMsg
    message
    @param[in] conveyor [RoisConveyor&] A struct containing
    vectors of the rois keypoints, bounding rectangles' vertices
    and blobs' outlines
    @param[in] image [cv::Mat&] The image to be packed in the message
    @param[out] candidateRoisVectorMsg
    [flir_lepton_ros_comm::CandidateRoisVectorMsg*] The output message
    @param[in] encoding [std::string&] The image's encoding
    @param[in] msg [const sensor_msgs::Image&] Needed to extract
    its header and place it as the header of the output message
    @return void
   **/
  void MessageConversions::createCandidateRoisVectorMessage(
    const RoisConveyor& conveyor,
    const cv::Mat& image,
    flir_lepton_ros_comm::CandidateRoisVectorMsg* candidateRoisVectorMsg,
    const std::string& encoding,
    const sensor_msgs::Image& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCandidateRoisVectorMessage");
    #endif

    // Fill the flir_lepton_ros_comm::CandidateRoisVectorMsg's
    // candidateRois vector
    std::vector<flir_lepton_ros_comm::CandidateRoiMsg> candidateRoisVector;
    createCandidateRoisVector(conveyor, &candidateRoisVector);

    candidateRoisVectorMsg->candidateRois = candidateRoisVector;

    // Fill the flir_lepton_image_processing::CandidateRoisVectorMsg's image
    candidateRoisVectorMsg->image =
      convertImageToMessage(image, encoding, msg);

    // Fill the flir_lepton_image_processing::CandidateRoisVectorMsg's header
    candidateRoisVectorMsg->header = msg.header;

    #ifdef DEBUG_TIME
    Timer::tick("createCandidateRoisVectorMessage");
    #endif
  }

  /**
    @brief Extracts a cv::Mat image from a ROS image message
    @param[in] msg [const sensor_msgs::Image&] The input ROS image
    message
    @param[out] image [cv::Mat*] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessage(
    const sensor_msgs::Image& msg,
    cv::Mat* image,
    const std::string& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("extractImageFromMessage");
    #endif

    cv_bridge::CvImagePtr in_msg;

    in_msg = cv_bridge::toCvCopy(msg, encoding);

    *image = in_msg->image.clone();

    #ifdef DEBUG_TIME
    Timer::tick("extractImageFromMessage");
    #endif
  }

  /**
    @brief Extracts a cv::Mat image from a custom ROS message of type
    flir_lepton_image_processing::CandidateRoisVectorMsg
    @param[in] msg [const flir_lepton_ros_comm::candidateRoisVectorMsg&]
    The input ROS message
    @param[out] image [cv::Mat*] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessageContainer(
    const flir_lepton_ros_comm::CandidateRoisVectorMsg& msg,
    cv::Mat* image, const std::string& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("extractThermalImageFromMessageContainer");
    #endif

    sensor_msgs::Image imageMsg = msg.image;
    extractImageFromMessage(imageMsg, image, encoding);

    #ifdef DEBUG_TIME
    Timer::tick("extractThermalImageFromMessageContainer");
    #endif
  }


  /**
    @brief Recreates the RoisConveyor struct for the candidate rois
    from the flir_lepton_image_processing::CandidateRoisMsg message
    @param[in] candidateRoisVector
    [const std::vector<flir_lepton_ros_comm::CandidateRoiMsg>&]
    The input candidate rois
    @param[out] conveyor [RoisConveyor*] The output conveyor
    struct
    @return void
   **/
  void MessageConversions::fromCandidateRoiMsgToConveyor(
    const std::vector<flir_lepton_ros_comm::CandidateRoiMsg>&
    candidateRoisVector,
    RoisConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("fromCandidateRoiMsgToConveyor", "unpackMessage");
    #endif

    for (unsigned int i = 0; i < candidateRoisVector.size(); i++)
    {
      // A single roi
      RoiConveyor roi;

      // Recreate the roi's keypoint
      roi.keypoint.pt.x = candidateRoisVector[i].keypointX;
      roi.keypoint.pt.y = candidateRoisVector[i].keypointY;

      // Recreate the roi's rectangle points
      std::vector<cv::Point2f> renctangleVertices;
      for (unsigned int v = 0;
        v < candidateRoisVector[i].verticesX.size(); v++)
      {
        cv::Point2f vertex;
        vertex.x = candidateRoisVector[i].verticesX[v];
        vertex.y = candidateRoisVector[i].verticesY[v];
        renctangleVertices.push_back(vertex);
      }
      roi.rectangle = renctangleVertices;

      // Recreate the roi's outline points
      std::vector<cv::Point2f> outlinePoints;
      for (unsigned int o = 0;
        o < candidateRoisVector[i].outlineX.size(); o++)
      {
        cv::Point2f outlinePoint;
        outlinePoint.x = candidateRoisVector[i].outlineX[o];
        outlinePoint.y = candidateRoisVector[i].outlineY[o];
        outlinePoints.push_back(outlinePoint);
      }
      roi.outline= outlinePoints;

      // Push roi back into the conveyor
      conveyor->rois.push_back(roi);
    }

    #ifdef DEBUG_TIME
    Timer::tick("fromCandidateRoiMsgToConveyor");
    #endif
  }



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
  void MessageConversions::unpackMessage(
    const flir_lepton_ros_comm::CandidateRoisVectorMsg& roisMsg,
    RoisConveyor* conveyor,
    cv::Mat* image,
    const std::string& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackMessage");
    #endif

    // Unpack the image
    extractImageFromMessageContainer(roisMsg, image, encoding);

    // Recreate the conveyor
    fromCandidateRoiMsgToConveyor(
      roisMsg.candidateRois,
      conveyor);

    #ifdef DEBUG_TIME
    Timer::tick("unpackMessage");
    #endif
  }

  /**
    @brief Convert the Float32MultiArray data to cv::Mat.
    Its cv format  will be CV_8UC1.
    @param[in] inArray [const std_msgs::Float32MultiArray&]
    The input MultiArray
    @return cv::Mat
   **/
  cv::Mat MessageConversions::convertFloat32MultiArrayToMat(
    const std_msgs::Float32MultiArray& inArray)
  {
    // The width and height of the input temperature multiarray
    int width = inArray.layout.dim[1].size;
    int height = inArray.layout.dim[0].size;

    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1);

    for (unsigned int i = 0; i < height; i++)
    {
      for (unsigned int j = 0; j < width; j++)
      {
        image.data[i * width + j] = inArray.data[i * width + j];
      }
    }
    return image;
  }

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton
