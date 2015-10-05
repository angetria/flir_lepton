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

#include "utils/thermal_rois_conveyor.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Appends one RoisConveyor struct to another.
    @param[in] src [const RoissConveyor&] The source conveyor
    @param[out] dst [RoissConveyor*] The destination conveyor
    @return void
   **/
  void RoisConveyorUtils::append(const RoisConveyor& src, RoisConveyor* dst)
  {
    for (int i = 0; i < src.size(); i++)
    {
      dst->rois.push_back(src.rois[i]);
    }
  }



  /**
    @brief Appends a dummy RoisConveyor to a RoiConveyor struct
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
  void RoisConveyorUtils::appendDummyConveyor(
    const cv::Point2f& rectangleUpperLeft,
    const cv::Point2f& outlineUpperLeft,
    const int& rx, const int& ry,
    const int& ox, const int& oy,
    RoisConveyor* conveyor)
  {
    // A conveyor of a single roi
    RoiConveyor roi;

    // Assign the keypoint
    cv::KeyPoint k(outlineUpperLeft.x + ox / 2,
      outlineUpperLeft.y + oy / 2, 1);

    roi.keypoint = k;

    // Assign the rectangle points
    roi.rectangle =
      generateRectangle(rectangleUpperLeft, rx, ry, 0);

    // Assign the outline points
    roi.outline =
      generateRectangle(outlineUpperLeft, ox, oy, 1);

    // Append roi into conveyor
    conveyor->rois.push_back(roi);
  }



  /**
    @brief Hollows a RoisConveyor struct, deleting every entry in it
    @param[in,out] conveyor [RoisConveyor*] The conveyor struct that will
    be cleared
    @return void
   **/
  void RoisConveyorUtils::clear(RoisConveyor* conveyor)
  {
    conveyor->rois.clear();
  }



  /**
    @brief Copies one RoisConveyor struct to another. If the dst conveyor
    is not empty, it empties it first, and then copies the src to the dst
    @param[in] src [const RoisConveyor&] The source struct
    @param[out] dst [RoisConveyor*] The destination struct
    @return void
   **/
  void RoisConveyorUtils::copyTo(const RoisConveyor& src,
    RoisConveyor* dst)
  {
    // If the dst is not empty, clear it
    if (dst->size() > 0)
    {
      clear(dst);
    }

    // Append the src to the dst
    append(src, dst);
  }



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
  std::vector<cv::Point2f> RoisConveyorUtils::generateRectangle(
    const cv::Point2f& upperLeft, const int& x, const int& y,
    const int& intent)
  {
    // The vector of the rectangle's vertices
    std::vector<cv::Point2f> rectangleVertices;

    // The four vertices of the rectangle
    cv::Point2f vertex_1(upperLeft.x, upperLeft.y);
    cv::Point2f vertex_2(upperLeft.x, upperLeft.y + y);
    cv::Point2f vertex_3(upperLeft.x + x, upperLeft.y + y);
    cv::Point2f vertex_4(upperLeft.x + x, upperLeft.y);

    // Push them back into the vector
    rectangleVertices.push_back(vertex_1);
    rectangleVertices.push_back(vertex_2);
    rectangleVertices.push_back(vertex_3);
    rectangleVertices.push_back(vertex_4);

    // Outline construction
    if (intent == 1)
    {
      rectangleVertices.clear();

      cv::Point2f a[] = {vertex_1, vertex_2, vertex_3, vertex_4};

      cv::Mat canvas = cv::Mat::zeros(480, 640, CV_8UC1);

      for (unsigned int j = 0; j < 4; j++)
      {
        cv::line(canvas, a[j], a[(j + 1) % 4], cv::Scalar(255, 0, 0), 1, 8);
      }

      for (int i = 0; i < 480; i++)
      {
        for (int j = 0; j < 640; j++)
        {
          if (canvas.at<unsigned char>(i, j) != 0)
          {
            rectangleVertices.push_back(cv::Point2f(j, i));
          }
        }
      }
      return rectangleVertices;
    }
    return rectangleVertices;
  }



  /**
    @brief Extracts the specified roi from a RoisConveyor into a new
    RoisConveyor struct that is returned
    @param[in] conveyor [const RoisConveyor&] The RoisConveyor struct
    @param[in] index [const int&] The index of the roi inside the conveyor
    @return A RoisConveyor struct that containes the index-th roi of the
    conveyor
   **/
  RoisConveyor RoisConveyorUtils::getRoi(const RoisConveyor& conveyor,
    const int& index)
  {
    // The conveyor that will be returned
    RoisConveyor temp;

    // Push back the index-th roi of the conveyor into temp
    temp.rois.push_back(conveyor.rois[index]);

    return temp;
  }



  /**
    @brief Given two sources of struct RoisConveyor, this function
    merge them into one struct.
    @param[in] srcA [const RoisConveyor&] The first RoisConveyor source
    @param[in] srcB [const RoisConveyor&] The second RoisConveyor source
    @param[out] dst [RoisConveyor*] The final struct
   **/
  void RoisConveyorUtils::merge(const RoisConveyor& srcA,
    const RoisConveyor& srcB, RoisConveyor* dst)
  {
    // Clear the destination conveyor if not empty
    if (dst->size() > 0)
    {
      clear(dst);
    }

    // Append the first source to dst
    append(srcA, dst);

    // Append the second source to dst
    append(srcB, dst);
  }



  /**
    @brief Deletes a roi from RoisConveyor struct,
    @param[in,out] conveyor [RoisConveyor*] The conveyor struct from which
    the roi will be removed
    @param[in] id [const int&] The index of the roi in the conveyor
    @return void
   **/
  void RoisConveyorUtils::removeRoi(RoisConveyor* conveyor, const int& id)
  {
    // Delete the respective keypoint
    conveyor->rois.erase(conveyor->rois.begin() + id);
  }



  /**
    @brief Replaces an entire RoisConveyor struct with another
    @param[in] src [const RoisConveyor&] The source conveyor struct
    @param[out] dst [RoisConveyor*] The destination conveyor struct
    @return void
   **/
  void RoisConveyorUtils::replace(const RoisConveyor& src, RoisConveyor* dst)
  {
    // Clear the dst
    clear(dst);

    // Fill it with the src
    copyTo(src, dst);
  }



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
  void RoisConveyorUtils::replaceRoi(const RoisConveyor& src,
    const int& srcIndex, RoisConveyor* dst, const int& dstIndex)
  {
    // Replace the dst's dstIndex-th roi's keypoint
    dst->rois[dstIndex].keypoint = src.rois[srcIndex].keypoint;

    // Replace the dst's dstIndex-th roi's rectangle points
    dst->rois[dstIndex].rectangle = src.rois[srcIndex].rectangle;

    // Replace the dst's dstIndex-th roi's outline points
    dst->rois[dstIndex].outline = src.rois[srcIndex].outline;
  }



  /**
    @brief Shuffles the contents of a RoisConveyor
    @param[in,out] src [RoisConveyor*] The conveyor
    @return void
   **/
  void RoisConveyorUtils::shuffle(RoisConveyor* src)
  {
    // Keep the original rois' arrangement
    RoisConveyor temp;
    copyTo(*src, &temp);

    // Hollow-out the src
    clear(src);

    // The vector of rois' indices
    std::vector<int> indices;
    for (int i = 0; i < temp.size(); i++)
    {
      indices.push_back(i);
    }

    // Shuffle the indices
    std::random_shuffle(indices.begin(), indices.end());

    // Fill the src conveyor with the shuffled rois
    for (int i = 0; i < temp.size(); i++)
    {
      append(getRoi(temp, indices[i]), src);
    }
  }

}  // namespace pandora_vision
