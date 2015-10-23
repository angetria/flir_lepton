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

#include "utils/edge_detection.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton
{
namespace flir_lepton_image_processing
{
  /**
    @brief Applies the Canny edge detector
    @param[in] inImage [const cv::Mat&] Input image in CV_8U depth
    @param[out] outImage [cv::Mat*] The processed image in CV_8U depth
    @return void
   **/
  void EdgeDetection::applyCanny(const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "EdgeDetection::applyCanny : Inappropriate image depth.");

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("applyCanny", "computeEdges");
    #endif

    inImage.copyTo(*outImage);

    // Reduce noise with a kernel with size
    // Parameters::Edge::canny_blur_noise_kernel_size ^ 2
    cv::blur(*outImage, *outImage,
      cv::Size(
        Parameters::Edge::canny_blur_noise_kernel_size,
        Parameters::Edge::canny_blur_noise_kernel_size));

    // Canny detector
    cv::Canny(*outImage, *outImage,
      Parameters::Edge::canny_low_threshold,
      Parameters::Edge::canny_low_threshold * Parameters::Edge::canny_ratio,
      Parameters::Edge::canny_kernel_size);

    #ifdef DEBUG_TIME
    Timer::tick("applyCanny");
    #endif
  }



  /**
    @brief Applies the Scharr edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
   **/
  void EdgeDetection::applyScharr(const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "EdgeDetection::applyScharr : Inappropriate image depth.");

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("applyScharr", "computeEdges");
    #endif

    // appropriate values for scale, delta and ddepth
    int scale = 1;

    // the value for the non-edges
    int delta = 0;

    int ddepth = CV_16S;

    cv::Mat edges;
    inImage.copyTo(edges);

    // Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Gradient X
    cv::Scharr(edges, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    cv::Scharr(edges, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Total Gradient (approximate)
    cv::Mat grad_g;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad_g);

    *outImage = grad_g;

    #ifdef DEBUG_TIME
    Timer::tick("applyScharr");
    #endif
  }



  /**
    @brief Applies the Sobel edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
   **/
  void EdgeDetection::applySobel(const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "EdgeDetection::applySobel : Inappropriate image depth.");

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("applySobel", "computeEdges");
    #endif

    // appropriate values for scale, delta and ddepth
    int scale = 1;

    // the value for the non-edges
    int delta = 0;

    int ddepth = CV_16S;

    cv::Mat edges;
    inImage.copyTo(edges);

    // Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Gradient X
    cv::Sobel(edges, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    cv::Sobel(edges, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Total Gradient (approximate)
    cv::Mat grad_g;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad_g);

    *outImage = grad_g;

    #ifdef DEBUG_TIME
    Timer::tick("applySobel");
    #endif
  }



  /**
    @brief Applies the Laplacian edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
   **/
  void EdgeDetection::applyLaplacian(const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "EdgeDetection::applyLaplacian : Inappropriate image depth.");

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("applyLaplacian", "computeEdges");
    #endif

    // appropriate values for scale, delta and ddepth
    int scale = 1;

    // the value for the non-edges
    int delta = 0;

    int ddepth = CV_16S;

    cv::Mat edges;
    inImage.copyTo(edges);

    cv::Laplacian(edges, *outImage, ddepth, 1, scale, delta, cv::BORDER_DEFAULT);
    convertScaleAbs(*outImage, *outImage);

    #ifdef DEBUG_TIME
    Timer::tick("applyLaplacian");
    #endif
  }



  /**
    @brief Applies contamination to an image of edges.
    It keeps only the edges that are not iteratively neighbors
    to the image's limits
    @param[in,out] inImage [cv::Mat*] Input image in CV_8UC1 format
    @return void
   **/
  void EdgeDetection::applyEdgeContamination(cv::Mat* inImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyEdgeContamination", "denoiseEdges");
    #endif

    int rows = inImage->rows;
    int cols = inImage->cols;
    std::set<unsigned int> current, next, visited;

    // Make the vertical borders of the image black
    for (unsigned int i = 0 ; i < rows ; i++)
    {
      inImage->data[i * inImage->cols] = 0;
      inImage->data[i * inImage->cols + cols - 1] = 0;
    }

    // Make the horizontal borders of the image black
    for (unsigned int j = 0 ; j < cols ; j++)
    {
      inImage->data[j] = 0;
      inImage->data[(rows - 1) * inImage->cols + j] = 0;
    }

    // Vertically, find the outer white borders
    for (unsigned int i = 1 ; i < rows - 1 ; i++)
    {
      if (inImage->data[i * inImage->cols + 1] > 0)
      {
        current.insert(i * cols + 1);
        inImage->data[i * inImage->cols + 1] = 0;
      }
      if (inImage->data[i * inImage->cols + cols - 2] > 0)
      {
        current.insert(i * cols + cols - 2);
        inImage->data[i * inImage->cols + cols - 2] = 0;
      }
    }

    // Horizontally, find the outer white borders
    for (unsigned int j = 1 ; j < cols - 1 ; j++)
    {
      if (inImage->data[1 * inImage->cols + j] > 0)
      {
        current.insert(1 * cols + j);
        inImage->data[1 * inImage->cols + j] = 0;
      }

      if (inImage->data[(rows - 2) * inImage->cols + j] > 0)
      {
        current.insert((rows - 2) * cols + j);
        inImage->data[(rows - 2) * inImage->cols + j] = 0;
      }
    }

    // Iterative contamination.
    // Find all the pixels that are iteratively neighboring non-zero value
    // pixels that lie next to the borders of the image
    while (current.size() != 0)
    {
      for (std::set<unsigned int>::iterator i = current.begin() ;
        i != current.end() ; i++)
      {
        int x = *i / cols;
        int y = *i % cols;

        if (inImage->data[(x - 1) * inImage->cols + y - 1] != 0)
        {
          next.insert((x - 1) * cols + y - 1);
          inImage->data[(x - 1) * inImage->cols + y - 1] = 0;
        }

        if (inImage->data[(x - 1) * inImage->cols + y] != 0)
        {
          next.insert((x - 1) * cols + y);
          inImage->data[(x - 1) * inImage->cols + y] = 0;
        }

        if (inImage->data[(x - 1) * inImage->cols + y + 1] != 0)
        {
          next.insert((x - 1) * cols + y + 1);
          inImage->data[(x - 1) * inImage->cols + y + 1] = 0;
        }

        if (inImage->data[x * inImage->cols + y + 1] != 0)
        {
          next.insert(x * cols + y + 1);
          inImage->data[x * inImage->cols + y + 1] = 0;
        }

        if (inImage->data[x * inImage->cols + y - 1] != 0)
        {
          next.insert(x * cols + y - 1);
          inImage->data[x * inImage->cols + y - 1] = 0;
        }

        if (inImage->data[(x + 1) * inImage->cols + y - 1] != 0)
        {
          next.insert((x + 1) * cols + y - 1);
          inImage->data[(x + 1) * inImage->cols + y - 1] = 0;
        }

        if (inImage->data[(x + 1) * inImage->cols + y] != 0)
        {
          next.insert((x + 1) * cols + y);
          inImage->data[(x + 1) * inImage->cols + y] = 0;
        }

        if (inImage->data[(x + 1) * inImage->cols + y + 1] != 0)
        {
          next.insert((x + 1) * cols + y + 1);
          inImage->data[(x + 1) * inImage->cols + y + 1] = 0;
        }
      }
      current.swap(next);
      next.clear();
    }

    #ifdef DEBUG_TIME
    Timer::tick("applyEdgeContamination");
    #endif
  }

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
  void EdgeDetection::computeThermalEdges(const cv::Mat& inImage, cv::Mat* edges)
  {
    if (inImage.type() != CV_8UC1)
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "EdgeDetection::computeEdges : Inappropriate thermal image type.");

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("computeEdges", "findThermalRois");
    #endif

    // The input thermal image, in CV_8UC1 format
    cv::Mat visualizableThermalImage = Visualization::scaleImageForVisualization(
      inImage, Parameters::Image::scale_method);

    // The image of edges of the denoised thermal image, in CV_8UC1 format
    cv::Mat denoisedThermalImageEdges;

    // Detect edges in the visualizableDepthImage
    detectEdges(visualizableThermalImage, &denoisedThermalImageEdges,
      Parameters::Edge::edge_detection_method);

    // Apply a threshold to the image of edges
    // to get rid of low value - insignificant - edges
    // The next two steps are insignificant when we process the Temperature
    // image extracted from thermal camera
    cv::threshold(denoisedThermalImageEdges, denoisedThermalImageEdges,
      Parameters::Edge::denoised_edges_threshold, 255, 3);

    // Make all non zero pixels have a value of 255
    cv::threshold(denoisedThermalImageEdges, denoisedThermalImageEdges, 0, 255,
      cv::THRESH_BINARY);

    // Denoise the edges found
    denoisedThermalImageEdges.copyTo(*edges);
    denoiseEdges(edges);

    #ifdef DEBUG_TIME
    Timer::tick("computeEdges");
    #endif
  }

  /**
    @brief Connects each point of a number of pair of points with a line
    or an elliptic arc
    @param[in,out] inImage [cv::Mat*] The image whose selected points will be
    connected and line or the elliptic arc drawn on
    @param[in] pairs [const std::vector<std::pair<GraphNode,GraphNode> >&]
    The vector of pair points
    @param[in] method [const int&] Denotes the connection type.
    0 for line,
    1 for elliptic arc
    @return void
   **/
  void EdgeDetection::connectPairs(cv::Mat* inImage,
    const std::vector<std::pair<GraphNode, GraphNode> >& pairs,
    const int& method)
  {
    if (inImage->depth() != CV_8U)
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "EdgeDetection::connectPairs: Inappropriate image depth.");

      return;
    }

    #ifdef DEBUG_SHOW
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    if (Parameters::Debug::show_connect_pairs)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Connection before";
      msgs.push_back(msg);
      cv::Mat tmp;
      inImage->copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::start("connectPairs", "denoiseEdges");
    #endif

    // Connects each pair of points via a line
    if (method == 0)
    {
      for (unsigned int i = 0; i < pairs.size(); i++)
      {
        cv::line(
          *inImage,
          cv::Point(pairs[i].first.y, pairs[i].first.x),
          cv::Point(pairs[i].second.y, pairs[i].second.x),
          cv::Scalar(255, 255, 255), 1, 8);
      }
    }
    // Connects each pair of points via an arc
    else if (method == 1)
    {
      // The input image. On it the elliptical arcs will be drawn
      cv::Mat inImageDrawnOnce;

      // The image on which only the elliptical arcs will be drawn
      cv::Mat addedArcs = cv::Mat::zeros(inImage->size(), CV_8UC1);

      for (unsigned int i = 0; i < pairs.size(); i++)
      {
        inImage->copyTo(inImageDrawnOnce);

        // The euclidean distance of the pair
        float pairsDistance = sqrt(
          pow((pairs[i].first.x - pairs[i].second.x), 2) +
          pow((pairs[i].first.y - pairs[i].second.y), 2));

        // The middle point of the segment connecting the pair points
        cv::Point2f bisectorPoint(
          round((pairs[i].first.y + pairs[i].second.y) / 2),
          round((pairs[i].first.x + pairs[i].second.x) / 2));

        // The angle that the line connecting the two pair points
        // forms with the horizontal axis
        float bisectorAngle = atan2(
          pairs[i].second.x - pairs[i].first.x,
          pairs[i].first.y - pairs[i].second.y);

        // Given the line that is perpendicular to the line that connects
        // the pair points, move on it one point at a time, in opposite
        // directions. The first non-zero point found will be one of the
        // curve that the pair lies on.

        // Indicator that a non-zero value point has been found
        bool foundOutlinePoint = false;

        // The outline point found
        cv::Point2f outlinePoint;

        int counter = 0;

        // Since there are two rays leaving the bisector point,
        // in opposite directions,
        // these two variables indicate the presence of the tip of the each ray
        // within the image's borders
        bool inLimitsOne = true;
        bool inLimitsTwo = true;

        while (!foundOutlinePoint)
        {
          counter++;

          // Has the tip of the first ray gone out of bounds?
          if (bisectorPoint.y + counter * cos(bisectorAngle) > inImage->rows - 1
            || bisectorPoint.y + counter * cos(bisectorAngle) < 0 ||
            bisectorPoint.x + counter * sin(bisectorAngle) > inImage->cols - 1
            || bisectorPoint.x + counter * sin(bisectorAngle) < 0)
          {
            inLimitsOne = false;
          }

          // Has the tip of the second ray gone out of bounds?
          if (bisectorPoint.y - counter * cos(bisectorAngle) > inImage->rows - 1
            || bisectorPoint.y - counter * cos(bisectorAngle) < 0 ||
            bisectorPoint.x - counter * sin(bisectorAngle) > inImage->cols - 1
            || bisectorPoint.x - counter * sin(bisectorAngle) < 0)
          {
            inLimitsTwo= false;
          }

          if (inLimitsOne)
          {
            // Sweep the neighbors of the tip of the ray in order to find
            // a valid non-zero value point, aka the outline point
            // that is being sought
            for (int m = -1; m < 2; m++)
            {
              for (int n = -1; n < 2; n++)
              {
                if (inImage->at<uchar>(
                    bisectorPoint.y + m + counter * cos(bisectorAngle),
                    bisectorPoint.x + n + counter * sin(bisectorAngle)) != 0)
                {
                  outlinePoint = cv::Point2f(
                    bisectorPoint.x + n + counter * sin(bisectorAngle),
                    bisectorPoint.y + m + counter * cos(bisectorAngle));

                  foundOutlinePoint = true;
                }
              }
            }
          }

          if (inLimitsTwo)
          {
            // Sweep the neighbors of the tip of the ray in order to find
            // a valid non-zero value point, aka the outline point
            // that is being sought
            for (int m = -1; m < 2; m++)
            {
              for (int n = -1; n < 2; n++)
              {
                if (inImage->at<uchar>(
                    bisectorPoint.y + m - counter * cos(bisectorAngle),
                    bisectorPoint.x + n - counter * sin(bisectorAngle)) != 0)
                {
                  outlinePoint = cv::Point2f(
                    bisectorPoint.x + n - counter * sin(bisectorAngle),
                    bisectorPoint.y + m - counter * cos(bisectorAngle));

                  foundOutlinePoint = true;
                }
              }
            }
          }

          // An outline point could not be found
          if (!inLimitsOne && !inLimitsTwo)
          {
            break;
          }
        }

        if (foundOutlinePoint)
        {
          // The distance between the outline point found, and the middle point
          // of the segment that connects the pair points
          float outlineBisectorPointDist = sqrt(
            pow(bisectorPoint.x - outlinePoint.x, 2) +
            pow(bisectorPoint.y - outlinePoint.y, 2));

          // If the curve that the pair points lie on approximates
          // a straight line, do not connect the two pair points
          if (pairsDistance >=
            Parameters::Outline::AB_to_MO_ratio * outlineBisectorPointDist)
          {
            continue;
          }

          // The major axis of the elliptical curve connecting the pair points
          float majorAxis = pairsDistance / 2;

          // The minor axis of the elliptical curve connecting the pair points
          float minorAxis = outlineBisectorPointDist / 2 < pairsDistance / 4 ?
            outlineBisectorPointDist / 2 : pairsDistance / 4;

          // The angle that the line that connects the first pair point and the
          // outline point found forms with the horizontal axis
          float pairFirstToOutlinePointAngle = atan2(
            outlinePoint.y - pairs[i].first.x,
            outlinePoint.x - pairs[i].first.y);

          // The angle that the line that connects the second pair point and the
          // outline point found forms with the horizontal axis
          float pairSecondToOutlinePointAngle = atan2(
            outlinePoint.y - pairs[i].second.x,
            outlinePoint.x - pairs[i].second.y);

          // Map the angles to the standard polar system
          if (pairFirstToOutlinePointAngle > 0)
          {
            pairFirstToOutlinePointAngle = M_PI
              - std::abs(pairFirstToOutlinePointAngle);
          }
          else
          {
            pairFirstToOutlinePointAngle = - M_PI
              + std::abs(pairFirstToOutlinePointAngle);
          }

          if (pairSecondToOutlinePointAngle > 0)
          {
            pairSecondToOutlinePointAngle = M_PI
              - std::abs(pairSecondToOutlinePointAngle);
          }
          else
          {
            pairSecondToOutlinePointAngle = - M_PI
              + std::abs(pairSecondToOutlinePointAngle);
          }


          // The angle between the line that connects the two pair points
          // and the horizontal axis
          float defaultAngle = atan2(
            pairs[i].first.x - pairs[i].second.x,
            pairs[i].first.y - pairs[i].second.y);

          // The angle between the line that connects the two pair points
          // and the horizontal axis. In order for the arc to accurately
          // connect the two pair points, we need to know its orientation.
          // It's one thing connecting point A to point B with an arc,
          // and another connecting point B to point A with an arc.
          // In other words, the arc has to have a starting point and an ending
          // point in order for the total connected curve to have the same
          // curvature orientation
          float pairsAngle;

          // Both pair points are above the X axis
          if (pairFirstToOutlinePointAngle > 0 &&
            pairSecondToOutlinePointAngle > 0)
          {
            if (pairFirstToOutlinePointAngle < pairSecondToOutlinePointAngle)
            {
              pairsAngle = defaultAngle + M_PI;
            }
            else
            {
              pairsAngle = defaultAngle;
            }
          }

          // Both pair points are below the X axis
          if (pairFirstToOutlinePointAngle < 0 &&
            pairSecondToOutlinePointAngle < 0)
          {
            if (pairFirstToOutlinePointAngle > pairSecondToOutlinePointAngle)
            {
              pairsAngle = defaultAngle;
            }
            else
            {
              pairsAngle = defaultAngle + M_PI;
            }
          }

          // Pair point 2 is above the X axis while pair point 1 below it
          if (pairFirstToOutlinePointAngle < 0 &&
            pairSecondToOutlinePointAngle > 0)
          {
            if (-pairFirstToOutlinePointAngle
              + pairSecondToOutlinePointAngle < M_PI)
            {
              pairsAngle = defaultAngle + M_PI;
            }
            else
            {
              pairsAngle = defaultAngle;
            }
          }

          // Pair point 1 is above the X axis while pair point 2 below it
          if (pairFirstToOutlinePointAngle > 0 &&
            pairSecondToOutlinePointAngle < 0)
          {
            if (pairFirstToOutlinePointAngle
              - pairSecondToOutlinePointAngle < M_PI)
            {
              pairsAngle = defaultAngle;
            }
            else
            {
              pairsAngle = defaultAngle + M_PI;
            }
          }

          // Draw the elliptical curve on the inImageDrawnOnce image
          // size arg 1: length of the major axis. always pairsDistance / 2
          // size arg 2: length of the minor axis.
          cv::ellipse(
            inImageDrawnOnce,
            bisectorPoint,
            cv::Size(majorAxis, minorAxis),
            pairsAngle * 180 / M_PI,
            0,
            180,
            cv::Scalar(255, 255, 255));

          for (unsigned int rows = 0; rows < inImage->rows; rows++)
          {
            for (unsigned int cols = 0; cols < inImage->cols; cols++)
            {
              if (inImageDrawnOnce.at<unsigned char>(rows, cols) != 0 &&
                inImage->at<unsigned char>(rows, cols) == 0)
              {
                addedArcs.at<uchar>(rows, cols) = 255;
              }
            }
          }
        }
      }

      *inImage += addedArcs;
    }

    #ifdef DEBUG_TIME
    Timer::tick("connectPairs");
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_connect_pairs)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Connection after";
      msgs.push_back(msg);
      cv::Mat tmp;
      inImage->copyTo(tmp);
      imgs.push_back(tmp);
    }
    if (Parameters::Debug::show_connect_pairs)  // Debug
    {
      Visualization::multipleShow("connectPairs function", imgs, msgs, 1200, 1);
    }
    #endif
  }



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
    place not been detected due to low contrast in a rois's borders.
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
  void EdgeDetection::denoiseEdges(cv::Mat* img)
  {
    if (img->depth() != CV_8U)
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "EdgeDetection::denoiseEdges: Inappropriate image depth.");

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("denoiseEdges", "computeEdges");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("Sector #1", "denoiseEdges");
    #endif

    #ifdef DEBUG_SHOW
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Initial Edges";
      msgs.push_back(msg);
      cv::Mat tmp;
      img->copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // Perform edge contamination:
    // remove all non-zero pixels iteratively adjacent to the image's borders
    cv::Mat contaminatedEdges;
    img->copyTo(contaminatedEdges);

    applyEdgeContamination(&contaminatedEdges);

    #ifdef DEBUG_TIME
    Timer::tick("Sector #1");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("Sector #2", "denoiseEdges");
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After edge contamination";
      msgs.push_back(msg);
      cv::Mat tmp;
      contaminatedEdges.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // Eliminate closed shapes
    for (unsigned int i = 0 ; i < img->rows ; i++)
    {
      contaminatedEdges.at<unsigned char>(i, 0) = 0;
      contaminatedEdges.at<unsigned char>(i, img->cols - 1) = 0;
    }
    for (unsigned int i = 0 ; i < img->cols ; i++)
    {
      contaminatedEdges.at<unsigned char>(0, i) = 0;
      contaminatedEdges.at<unsigned char>(img->rows - 1, i) = 0;
    }

    // Apply thinning to the contaminated original edges
    cv::Mat thinnedImage;
    contaminatedEdges.copyTo(thinnedImage);

    Morphology::thinning(contaminatedEdges, &thinnedImage, 100);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After thinning";
      msgs.push_back(msg);
      cv::Mat tmp;
      thinnedImage.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // By pruning the thinned image,
    // all pixels that are open-ended are eliminated,
    // leaving thus behind only shapes whose ends are connected, aka the
    // closed shapes
    cv::Mat thinnedClosedLines;
    thinnedImage.copyTo(thinnedClosedLines);
    Morphology::pruningStrictIterative(&thinnedClosedLines, 1000);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After pruning";
      msgs.push_back(msg);
      cv::Mat tmp;
      thinnedClosedLines.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // The thinnedOpenLines image now features only the thinned,
    // open-ended shapes
    cv::Mat thinnedOpenLines = thinnedImage - thinnedClosedLines;

    // We want to preserve the original (unthinned) outline of the closed shapes
    // and connect the ends of open-ended shapes which are thinned to this point.
    // So, first, we need to identify the closed shapes in the edge contaminated
    // image. These will be stored in the closedLines image.
    cv::Mat closedLines;
    thinnedClosedLines.copyTo(closedLines);

    bool finished = false;

    while (!finished)
    {
      cv::Mat marker;
      closedLines.copyTo(marker);

      for (int rows = 1; rows < img->rows - 1; rows++)
      {
        for (int cols = 1; cols < img->cols - 1; cols++)
        {
          if (closedLines.at<unsigned char>(rows, cols) == 0
            && contaminatedEdges.at<unsigned char>(rows, cols) == 255)
          {
            for (int r = -1; r < 2; r++)
            {
              for (int c = -1; c < 2; c++)
              {
                if (contaminatedEdges.at<unsigned char>(rows + r, cols + c) != 0
                  && closedLines.at<unsigned char>(rows + r, cols + c) != 0)
                {
                  closedLines.at<unsigned char>(rows, cols) = 255;
                }
              }
            }
          }
        }
      }

      // The operation has finished when all the original closed shapes have
      // been found
      if (cv::countNonZero(closedLines - marker) == 0)
      {
        finished = true;
      }
    }

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : The origninal closed shapes";
      msgs.push_back(msg);
      cv::Mat tmp;
      closedLines.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Without closed shapes";
      msgs.push_back(msg);
      cv::Mat tmp;
      thinnedOpenLines.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("Sector #2");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("Sector #3");
    #endif

    // In the image that features only open-ended shapes, find their end points.
    // if they are eligible for connection,
    // these points will be connected with each other

    std::vector<std::set<unsigned int> > lines;
    std::vector<std::pair<GraphNode, GraphNode> > farPts;

    identifyCurvesAndEndpoints(&thinnedOpenLines, &lines, &farPts);

    // Because the thinnedOpenLines image was wiped clean via the execution
    // of the identifyCurvesAndEndpoints method, re-paint the lines found on it
    for (unsigned int i = 0 ; i < lines.size() ; i++)
    {
      for (std::set<unsigned int>::iterator it = lines[i].begin() ;
        it != lines[i].end() ; it++)
      {
        thinnedOpenLines.data[*it] = 255;
      }
    }


    // Connect the end points of open shapes
    connectPairs(&thinnedOpenLines, farPts, 1);

    // Since what could be connected was connected, the rest of the open-ended
    // shapes are not needed.
    Morphology::pruningStrictIterative(&thinnedOpenLines, 1000);

    #ifdef DEBUG_TIME
    Timer::tick("Sector #3");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("Sector #4");
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After connection of distant edges";
      msgs.push_back(msg);
      cv::Mat tmp;
      thinnedOpenLines.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // Re-enable the closed shapes
    *img = thinnedOpenLines + closedLines;

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After re-insertion of closed shapes";
      msgs.push_back(msg);
      cv::Mat tmp;
      img->copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // Extract only the outer border of closed shapes
    OutlineDiscovery::getShapesClearBorderSimple(img);

    #ifdef DEBUG_TIME
    Timer::tick("Sector #4");
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After clear borders";
      msgs.push_back(msg);
      cv::Mat tmp;
      img->copyTo(tmp);
      imgs.push_back(tmp);
    }
    if (Parameters::Debug::show_denoise_edges)  // Debug
    {
      Visualization::multipleShow("denoiseEdges function", imgs, msgs,
        Parameters::Debug::show_denoise_edges_size, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("denoiseEdges");
    #endif
  }

  /**
    @brief Detects edges depending on a edge detection selection method.
    @param[in] inImage [const cv::Mat&] The image whose edges one wishes
    to detect.
    @param[out] outImage [cv::Mat*] The image of edges, in format CV_8UC1
    @param[in] detectionMethod [const int&] The edge detection method
    selector.
    @return void
   **/
  void EdgeDetection::detectEdges(const cv::Mat& inImage, cv::Mat* outImage,
    const int& detectionMethod)
  {
    switch (detectionMethod)
    {
      case CANNY :
        {
          applyCanny(inImage, outImage);
          break;
        }
      case SCHARR :
        {
          applyScharr(inImage, outImage);
          break;
        }
      case SOBEL :
        {
          applySobel(inImage, outImage);
          break;
        }
      case LAPLACIAN :
        {
          applyLaplacian(inImage, outImage);
          break;
        }
      case MIXED :
        {
          if (Parameters::Edge::mixed_edges_toggle_switch == 0)
          {
            applyScharr(inImage, outImage);
            Parameters::Edge::mixed_edges_toggle_switch = 1;
          }
          else if (Parameters::Edge::mixed_edges_toggle_switch == 1)
          {
            applySobel(inImage, outImage);
            Parameters::Edge::mixed_edges_toggle_switch = 0;
          }

          break;
        }
    }
  }

  /**
    @brief Identifies in which curve a point lies on and returns the
    curve's two end-points.
    @param[in] img [cv::Mat*] The input binary image
    @param[in] x_ [const int&] The x coordinate of the point
    @param[in] y_ [const int&] The y coordinate of the point
    @param[out] ret [std::set<unsigned int>&] The points that represent the
    curve on which the point lies on
    @return edgePoints [std::pair<GraphNode, GraphNode>*] The curve's pair of
    end points
   **/
  std::pair<GraphNode, GraphNode> EdgeDetection::identifyCurveAndEndpoints(
    cv::Mat* img, const int& x_, const int& y_, std::set<unsigned int>* ret)
  {
    #ifdef DEBUG_TIME
    Timer::start("identifyCurveAndEndpoints", "identifyCurvesAndEndpoints");
    #endif

    std::vector<unsigned int> current, next;
    std::set<unsigned int> currs;
    std::vector<GraphNode*> nodes;
    std::vector<GraphNode*> originCurr, originNext;
    GraphNode* father = new GraphNode(x_, y_);

    ret->insert(x_ * img->cols + y_);
    current.push_back(x_ * img->cols + y_);
    originCurr.push_back(father);
    nodes.clear();
    nodes.push_back(father);
    father->dist = 0;
    unsigned int stepCounter = 0;
    while (current.size() != 0)
    {
      next.clear();
      originNext.clear();
      for (unsigned int i = 0 ; i < current.size() ; i++)
      {
        std::vector<unsigned int> currNext;
        int x = current[i] / img->cols;
        int y = current[i] % img->cols;
        img->at<unsigned char>(x, y) = 0;
        unsigned int counter = 0;
        if (img->at<unsigned char>(x - 1, y - 1) != 0 &&
          currs.find((x-1) * img->cols + y - 1) == currs.end() &&
          ret->find((x-1) * img->cols + y - 1) == ret->end())
        {
          next.push_back((x-1) * img->cols + y - 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x-1) * img->cols + y - 1);
          counter++;
        }
        if (img->at<unsigned char>(x - 1, y) != 0 &&
          currs.find((x-1) * img->cols + y) == currs.end() &&
          ret->find((x-1) * img->cols + y) == ret->end())
        {
          next.push_back((x-1) * img->cols + y);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x-1) * img->cols + y);
          counter++;
        }
        if (img->at<unsigned char>(x - 1, y + 1) != 0 &&
          currs.find((x-1) * img->cols + y + 1) == currs.end() &&
          ret->find((x-1) * img->cols + y + 1) == ret->end())
        {
          next.push_back((x-1) * img->cols + y + 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x-1) * img->cols + y + 1);
          counter++;
        }

        if (img->at<unsigned char>(x, y - 1) != 0 &&
          currs.find((x) * img->cols + y - 1) == currs.end() &&
          ret->find((x) * img->cols + y - 1) == ret->end())
        {
          next.push_back((x) * img->cols + y - 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x) * img->cols + y - 1);
          counter++;
        }
        if (img->at<unsigned char>(x, y + 1) != 0 &&
          currs.find((x) * img->cols + y + 1) == currs.end() &&
          ret->find((x) * img->cols + y + 1) == ret->end())
        {
          next.push_back((x) * img->cols + y + 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x) * img->cols + y + 1);
          counter++;
        }

        if (img->at<unsigned char>(x + 1, y - 1) != 0 &&
          currs.find((x+1) * img->cols + y - 1) == currs.end() &&
          ret->find((x+1) * img->cols + y - 1) == ret->end())
        {
          next.push_back((x+1) * img->cols + y - 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x+1) * img->cols + y - 1);
          counter++;
        }
        if (img->at<unsigned char>(x + 1, y) != 0 &&
          currs.find((x+1) * img->cols + y) == currs.end() &&
          ret->find((x+1) * img->cols + y) == ret->end())
        {
          next.push_back((x+1) * img->cols + y);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x+1) * img->cols + y);
          counter++;
        }
        if (img->at<unsigned char>(x + 1, y + 1) != 0 &&
          currs.find((x+1) * img->cols + y + 1) == currs.end() &&
          ret->find((x+1) * img->cols + y + 1) == ret->end())
        {
          next.push_back((x+1) * img->cols + y + 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x+1) * img->cols + y + 1);
          counter++;
        }

        if (counter == 0)  // This is edge
        {
          GraphNode* temp = new GraphNode(x, y);
          nodes.push_back(temp);

          originCurr[i]->connections.push_back(temp);
          temp->connections.push_back(originCurr[i]);

          temp->dist = stepCounter - originCurr[i]->dist;

          originCurr[i]->dists.push_back(temp->dist);
          temp->dists.push_back(temp->dist);
        }
        else if (counter >= 2)  // This is joint
        {
          GraphNode* temp = new GraphNode(x, y);
          nodes.push_back(temp);
          originCurr[i]->connections.push_back(temp);
          temp->connections.push_back(originCurr[i]);
          temp->dist = stepCounter - originCurr[i]->dist;
          originCurr[i]->dists.push_back(temp->dist);
          temp->dists.push_back(temp->dist);
          for (unsigned int j = 0 ; j < currNext.size() ; j++)
          {
            originNext[currNext[j]] = temp;
          }
        }

        currNext.clear();
      }
      stepCounter++;
      current.swap(next);
      currs.clear();
      for (unsigned int k = 0 ; k < current.size() ; k++)
      {
        currs.insert(current[k]);
      }
      originCurr.swap(originNext);
    }

    std::pair<GraphNode, GraphNode> edgePoints;

    // Find larger dist between nodes
    int maxDist = 0;

    for (unsigned int i = 0 ; i < nodes.size(); i++)
    {
      std::vector<GraphNode*> current, next;
      std::map<GraphNode*, int> dists;
      std::set<GraphNode*> visited;

      current.push_back(nodes[i]);
      visited.insert(nodes[i]);
      for (unsigned int kk = 0 ; kk < nodes.size() ; kk++)
      {
        dists.insert(std::pair<GraphNode*, int>(nodes[i], 0));
      }
      while (current.size() != 0)
      {
        next.clear();
        for (unsigned int c = 0 ; c < current.size() ; c++)
        {
          for (unsigned int n = 0 ; n < current[c]->connections.size(); n++)
          {
            if (visited.find(current[c]->connections[n]) == visited.end())
            {
              visited.insert(current[c]->connections[n]);
              dists[current[c]->connections[n]] +=
                dists[current[c]] + current[c]->dists[n];
              next.push_back(current[c]->connections[n]);
            }
          }
        }
        current.swap(next);
      }
      for (unsigned int d = 0 ; d < nodes.size(); d++)
      {
        if (dists[nodes[d]] > maxDist)
        {
          maxDist = dists[nodes[d]];
          edgePoints.first.x = nodes[i]->x;
          edgePoints.first.y = nodes[i]->y;
          edgePoints.second.x = nodes[d]->x;
          edgePoints.second.y = nodes[d]->y;
        }
      }
    }

    // Delete pointers inside nodes
    for (unsigned int d = 0 ; d < nodes.size(); d++)
    {
      delete nodes[d];
    }

    #ifdef DEBUG_TIME
    Timer::tick("identifyCurveAndEndpoints");
    #endif

    return edgePoints;
  }

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
  void EdgeDetection::identifyCurvesAndEndpoints(cv::Mat* image,
    std::vector<std::set<unsigned int> >* lines,
    std::vector<std::pair<GraphNode, GraphNode> >* endPoints)
  {
    #ifdef DEBUG_TIME
    Timer::start("identifyCurvesAndEndpoints", "denoiseEdges");
    #endif

    bool hasFinished = false;

    while (!hasFinished)
    {
      hasFinished = true;
      for (unsigned int i = 1 ; i < image->rows - 1; i++)
      {
        for (unsigned int j = 1 ; j < image->cols - 1; j++)
        {
          // A point is potentially located along a curve only if its value
          // is non-zero; otherwise it is not a point!
          if (image->at<unsigned char> (i, j) != 0)
          {
            // Locate the indices of the points that constitute the curve on
            // which point (i, j) is located, along with the end points of it.
            std::set<unsigned int> ret;
            std::pair<GraphNode, GraphNode> pts =
              identifyCurveAndEndpoints(image, i, j, &ret);

            if (ret.size() > Parameters::Outline::minimum_curve_points)
            {
              // Push the indices of the points constituting the curve on which
              // point (i, j) is located, into the overall curves' indices vector.
              lines->push_back(ret);

              // Push the end-points of the curve into the overall vector of
              // end-points
              endPoints->push_back(pts);
            }

            hasFinished = false;
            break;
          }
        }

        if (!hasFinished)
        {
          break;
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("identifyCurvesAndEndpoints");
    #endif
  }

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton
