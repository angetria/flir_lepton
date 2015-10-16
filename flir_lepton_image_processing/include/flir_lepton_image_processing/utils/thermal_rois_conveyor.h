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
 * Angelos Triantafyllidis <aggelostriadafillidis@gmail.comg>
 *********************************************************************/

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_UTILS_THERMAL_ROIS_CONVEYOR_H
#define FLIR_LEPTON_IMAGE_PROCESSING_UTILS_THERMAL_ROIS_CONVEYOR_H

#include <std_msgs/Header.h>
#include "utils/defines.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton_rpi2
{
namespace flir_lepton_image_processing
{
  /**
    @brief The structure that represents a single thermal region of interest.
    @param keypoint [cv::KeyPoint] The roi's keypoint
    @param rectangle [std::vector<cv::Point2f>] The vector of the roi's
    rotated bounding box vertices
    @param outline [std::vector<cv::Point2f>] The vector of the roi's
    outline points
   **/
  struct RoiConveyor
  {
    cv::KeyPoint keypoint;
    std::vector<cv::Point2f> rectangle;
    std::vector<cv::Point2f> outline;

    // For Thermal Rois these variables get filled too.
    float roiProbability;
    float roiTemperature;
  };

  typedef boost::shared_ptr<RoiConveyor> RoiConveyorPtr;
  typedef boost::shared_ptr<RoiConveyor const> RoiConveyorConstPtr;

  /**
    A struct conveying information about multiple thermal rois
    @param header [std_msgs::Header] Header in respect with the frame of rois
    @param rois [std::vector<RoiConveyor>] The vector of rois
   **/
  struct RoisConveyor
  {
    std_msgs::Header header;
    std::vector<RoiConveyor> rois;

    /**
      @brief Returns the size of the @param rois vector
      @param void
      @return [unsigned int] The number of entries in the @param rois vector
     **/
    unsigned int size() const
    {
      return rois.size();
    }
  };

  typedef boost::shared_ptr<RoisConveyor> RoisConveyorPtr;
  typedef boost::shared_ptr<RoisConveyor const> RoisConveyorConstPtr;

  /**
    @Class RoisConveyorUtils
    @brief Provides methods pertinent to the RoisConveyor struct
   **/
  class RoisConveyorUtils
  {
    public:
      /**
        @brief Appends one RoisConveyor struct to another.
        @param[in] src [const RoisConveyor&] The source conveyor
        @param[out] dst [RoisConveyor*] The destination conveyor
        @return void
       **/
      static void append(const RoisConveyor& src, RoisConveyor* dst);

      /**
        @brief Appends a dummy RoisConveyor to a RoisConveyor struct
        @param[in] rectangleUpperLeft [const cv::Point2f&] The upper left
        vertex of the bounding rectangle
        @param[in] outlineUpperLeft [const cv::Point2f] The upper left
        vertex of the roi's outline
        @param[in] rx [const int&] The width of the rectangle
        @param[in] ry [const int&] The height of the rectangle
        @param[in] ox [const int&] The width of the outline rectangle
        @param[in] ry [const int&] The height of the outline rectangle
        @param[in,out] conveyor [RoisConveyor*] The conveyor to which the
        dummy RoisConveyor will be appended
        @return void
       **/
      static void appendDummyConveyor(
        const cv::Point2f& rectangleUpperLeft,
        const cv::Point2f& outlineUpperLeft,
        const int& rx, const int& ry,
        const int& ox, const int& oy,
        RoisConveyor* conveyor);

      /**
        @brief Hollows a RoisConveyor struct, deleting every entry in it
        @param[in,out] conveyor [RoisConveyor*] The conveyor struct that will
        be cleared
        @return void
       **/
      static void clear(RoisConveyor* conveyor);

      /**
        @brief Copies one RoisConveyor struct to another. If the dst conveyor
        is not empty, it empties it first, and then copies the src to the dst
        @param[in] src [const RoisConveyor&] The source struct
        @param[out] dst [RoisConveyor*] The destination struct
        @return void
       **/
      static void copyTo(const RoisConveyor& src, RoisConveyor* dst);

      /**
        @brief Generates a vector of cv::Point2f that represents the 4 vertices
        of a rectangle
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex point
        of the rectangle
        @param[in] x [const int&] The length at x direction
        @param[in] y [const int&] The length at y direction
        @param[in] intent [const int&] 0 for vertices'
        construction, 1 for a coherent outline construction
        @return std::vector<cv::Point2f> A vector of four vertices
        of type cv::Point2f
       **/
      static std::vector<cv::Point2f> generateRectangle(
        const cv::Point2f& upperLeft, const int& x, const int& y,
        const int& intent);

      /**
        @brief Extracts the specified roi from a RoisConveyor into a new
        RoisConveyor struct that is returned
        @param[in] conveyor [const RoisConveyor&] The RoisConveyor struct
        @param[in] index [const int&] The index of the roi inside the conveyor
        @return A RoisConveyor struct that containes the index-th roi of the
        conveyor
       **/
      static RoisConveyor getRoi(const RoisConveyor& conveyor,
        const int& index);

      /**
        @brief Given two sources of struct RoisConveyor, this function
        merge them into one struct.
        @param[in] srcA [const RoisConveyor&] The first RoisConveyor source
        @param[in] srcB [const RoisConveyor&] The second RoisConveyor source
        @param[out] dst [RoisConveyor*] The final struct
       **/
      static void merge(const RoisConveyor& srcA,
        const RoisConveyor& srcB, RoisConveyor* dst);

      /**
        @brief Prints data pertaining to the contents of a RoisConveyor struct,
        that is, the keypoints, rectangle points and outline points of the
        rois it contains
        @param[in] conveyor [const RoisConveyor&] The conveyor
        @param[in] id [const int&] The identifier of a specific roi
        @return void
       **/
      static void print(const RoisConveyor& conveyor, const int& id = -1);

      /**
        @brief Replaces an entire RoisConveyor struct with another
        @param[in] src [const RoisConveyor&] The source conveyor struct
        @param[out] dst [RoisConveyor*] The destination conveyor struct
        @return void
       **/
      static void replace(const RoisConveyor& src, RoisConveyor* dst);

      /**
        @brief Deletes a roi from RoisConveyor struct,
        @param[in,out] conveyor [RoisConveyor*] The conveyor struct from which
        the roi will be removed
        @param[in] id [const int&] The index of the roi in the conveyor
        @return void
       **/
      static void removeRoi(RoisConveyor* conveyor, const int& id);

      /**
        @brief Replaces a specified roi from a RoisConveyor dst struct
        with the roi of index srcIndex of the src RoisConveyor struct entry
        @param[in] src [const RoisConveyor&] The RoisConveyor source struct
        @param[in] srcIndex [const int&] The index of the roi inside the
        src conveyor that will be copied into the dst RoisConveyor struct,
        in the dstIndex position
        @param[out] dst [RoisConveyor*] The RoisConveyor destination struct
        @param[in] dstIndex [const int&] The index of the roi inside the
        dst conveyor that will be replaced by the srcIndex-th of the src
        RoisConveyor struct
        @return void
       **/
      static void replaceRoi(const RoisConveyor& src,
        const int& srcIndex, RoisConveyor* dst, const int& dstIndex);

      /**
        @brief Shuffles the contents of a RoisConveyor
        @param[in,out] src [RoisConveyor*] The conveyor
        @return void
       **/
      static void shuffle(RoisConveyor* src);
  };

}  // namespace flir_lepton_image_processsing
}  // namespace flir_lepton_rpi2

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_UTILS_THERMAL_ROIS_CONVEYOR_H
