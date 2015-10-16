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

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_UTILS_EDGE_DETECTION_H
#define FLIR_LEPTON_IMAGE_PROCESSING_UTILS_EDGE_DETECTION_H

#include "utils/outline_discovery.h"
#include "utils/morphological_operators.h"

#define CANNY 0
#define SCHARR 1
#define SOBEL 2
#define LAPLACIAN 3
#define MIXED 4

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton_rpi2
{
namespace flir_lepton_image_processing
{
  struct GraphNode
  {
    GraphNode(int x, int y)
    {
      this->x = x;
      this->y = y;
    }

    GraphNode(void)
    {
      this->x = 0;
      this->y = 0;
    }

    int x;
    int y;

    std::vector<GraphNode*> connections;

    std::vector<int> dists;

    unsigned int dist;
  };

  /**
    @class EdgeDetection
    @brief Provides methods for edge detection
   **/
  class EdgeDetection
  {
    public:
      /**
        @brief Applies the Canny edge detector
        @param[in] inImage [const cv::Mat&] Input image in CV_8U depth
        @param[out] outImage [cv::Mat*] The processed image in CV_8U depth
        @return void
       **/
      static void applyCanny(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Applies the Scharr edge transform
        @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
        @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
        @return void
       **/
      static void applyScharr(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Applies the Sobel edge transform
        @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
        @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
        @return void
       **/
      static void applySobel(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Applies the Laplacian edge transform
        @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
        @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
        @return void
       **/
      static void applyLaplacian(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Applies contamination to an image of edges.
        It keeps only the edges that are not iteratively neighbors
        to the image's limits
        @param[in,out] inImage [cv::Mat*] Input image in CV_8UC1 format
        @return void
       **/
      static void applyEdgeContamination(cv::Mat* inImage);

      /**
        @brief Takes as input a thermal image containing unsigned chars,
        locates the edges in it and tries to clear as much noise as possible
        in the edges image.
        It outputs a binary image that contains areas that we wish to validate
        as thermal rois.
        @param[in] inImage [const cv::Mat&] The thermal image extracted from the
        thermal camera, of type CV_8UC1
        @param[out] edges [cv::Mat*] The final denoised edges image that
        corresponds to the input image
        @return void
       **/
      static void computeThermalEdges(const cv::Mat& inImage, cv::Mat* edges);

      /**
        @brief Connects each point of a number of pair of points with a line
        or an elliptic arc
        @param[in,out] inImage [cv::Mat*] The image whose selected points will
        be connected and line or the elliptic arc drawn on
        @param[in] pairs [const std::vector<std::pair<GraphNode,GraphNode> >&]
        The vector of pair points
        @param[in] method [const int&] Denotes the connection type.
        0 for line,
        1 for elliptic arc
        @return void
       **/
      static void connectPairs(cv::Mat* inImage,
        const std::vector<std::pair<GraphNode, GraphNode> >& pairs,
        const int& method);

      /**
        @brief Takes an input image of edges of CV_8U depth
        and tries to isolate specific shapes so as to facilitate
        the blob detection process.

        The execution flow is as follows: first, all pixels connected
        directly or indirectly to the image's borders are eliminated,
        leaving behind all standalone shapes and curves.
        Next, all open-ended curves are contidionally closed.
        The reason behind this is that the open-ended curves might be
        edges of thermal rois that have either been smudged by the appliance of
        a threshold after the production of edges, or that have in the first
        place not been detected due to low contrast in a roi's borders.
        After the connection of the end-points of open-ended shapes,
        pruning is applied to get rid of all minor open-ended curves not
        closed by the connection process. The result here is a binary image
        featuring only closed shapes, but whose insides may be cluttered with
        non-zero value pixels, since the edges of what may be inside a roi
        are still detected and present in the current product image.
        What we want, based on this image, is to find only the external
        limits of each closed curve, because these pixels will be the
        outline of each blob. A method is then invoked to do just so.
        The end product of this method is a binary image with closed curves.
        Everything inside each closed curve is black and everything outside
        all closed curves is void.
        @param[in,out] img [cv::Mat*] The input image in unsigned char format
        @return void
       **/
      static void denoiseEdges(cv::Mat* img);

      /**
        @brief Detects edges depending on a edge detection selection method.
        @param[in] inImage [const cv::Mat&] The image whose edges one wishes
        to detect.
        @param[out] outImage [cv::Mat*] The image of edges, in format CV_8UC1
        @param[in] detectionMethod [const int&] The edge detection method
        selector.
        @return void
       **/
      static void detectEdges(const cv::Mat& inImage, cv::Mat* outImage,
        const int& detectionMethod);

      /**
        @brief Identifies in which curve a point lies on and returns the
        curve's two end points.
        @param[in] img [cv::Mat*] The input binary image
        @param[in] x_ [const int&] The x coordinate of the point
        @param[in] y_ [const int&] The y coordinate of the point
        @param[out] ret [std::set<unsigned int>&] The points that represent the
        curve on which the point lies on
        @return edgePoints [std::pair<GraphNode, GraphNode>*] The curve's pair
        of end points
       **/
      static std::pair<GraphNode, GraphNode> identifyCurveAndEndpoints(
        cv::Mat* img, const int& x_, const int& y_, std::set<unsigned int>* ret);

      /**
        @brief Given an image of CV_8UC1 format, this method locates and
        identifies all continuous curves, along with their end-points.
        CAUTION: the length of each curve must exceed a certain threshold.
        @param[in,out] image [cv::Mat*] The image whose curves and their
        endpoints one wishes to locate and identify. CAUTION: image is cleared
        at the end of the process.
        @param[out] lines [std::vector<std::set<unsigned int> >*]
        A set containing the indices of points that constite
        @param[out] endPoints [std::vector<std::pair<GraphNode, GraphNode> >*]
        A vector of endpoints.
        @return void
       **/
      static void identifyCurvesAndEndpoints(cv::Mat* image,
        std::vector<std::set<unsigned int> >*,
        std::vector<std::pair<GraphNode, GraphNode> >* endPoints);
  };

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton_rpi2

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_UTILS_EDGE_DETECTION_H
