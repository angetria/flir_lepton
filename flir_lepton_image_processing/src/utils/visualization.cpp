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

#include "utils/defines.h"
#include "utils/visualization.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton_rpi2
{
namespace flir_lepton_image_processing
{
  /**
    @brief Shows multiple images in one window
    @param[in] title [const std::string&] The window's title
    @param[in] imgs [const std::vector<cv::Mat>&] The images to be shown
    @param[in] titles [const std::vector<std::string>&] The titles for each
    image
    @param[in] maxSize [const int&] The maximum size of the window
    @param[in] ms [const int&] How many seconds the showing lasts
    @return [cv::Mat] An image featuring multiple images
   **/
  cv::Mat Visualization::multipleShow(
    const std::string& title,
    const std::vector<cv::Mat>& imgs,
    const std::vector<std::string>& titles,
    const int& maxSize,
    const int& ms)
  {
    unsigned int rows = 0, cols = 0;
    unsigned int sqdim = sqrt(imgs.size());
    rows = sqdim;
    cols = imgs.size() / rows + (((imgs.size() % rows) != 0) ? 1 : 0);

    unsigned int winCols = (static_cast<float>(maxSize)) / cols;
    float scale = (static_cast<float>(winCols)) / imgs[0].cols;

    unsigned int finalRows, finalCols;
    finalRows = rows * (static_cast<float>(imgs[0].rows) * scale);
    finalCols = winCols * cols;

    cv::Mat big(rows * imgs[0].rows, cols * imgs[0].cols, CV_8UC3);

    // Draw images
    for (unsigned int im = 0 ; im < imgs.size() ; im++)
    {
      unsigned int startRow, startCol;
      startRow = im / cols * imgs[im].rows;
      startCol = im % cols * imgs[im].cols;
      for (unsigned int i = 0 ; i < imgs[im].rows ; i++)
      {
        for (unsigned int j = 0 ; j < imgs[im].cols ; j++)
        {
          if (imgs[im].channels() == 1)
          {
            big.at<cv::Vec3b>(startRow + i, startCol + j)[0] =
              imgs[im].at<unsigned char>(i, j);
            big.at<cv::Vec3b>(startRow + i, startCol + j)[1] =
              imgs[im].at<unsigned char>(i, j);
            big.at<cv::Vec3b>(startRow + i, startCol + j)[2] =
              imgs[im].at<unsigned char>(i, j);
          }
          else if (imgs[im].channels() == 3)
          {
            big.at<cv::Vec3b>(startRow + i, startCol + j) =
              imgs[im].at<cv::Vec3b>(i, j);
          }
        }
      }

      cv::putText(big, titles[im].c_str(),
        cvPoint(startCol + 10, startRow + 20),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 255, 0), 1, CV_AA);
    }

    for (unsigned int i = 1 ; i <= rows - 1 ; i++)
    {
      cv::line(big,
        cv::Point2f(0 , i * imgs[0].rows - 1),
        cv::Point2f(cols * imgs[0].cols - 1, i * imgs[0].rows - 1),
        CV_RGB(255, 255, 255), 2, 8);
    }

    for (unsigned int i = 1 ; i <= cols - 1 ; i++)
    {
      cv::line(big,
        cv::Point2f(i * imgs[0].cols - 1 , 0),
        cv::Point2f(i * imgs[0].cols - 1, rows * imgs[0].rows),
        CV_RGB(255, 255, 255), 2, 8);
    }


    // Final resize
    cv::Mat resized(finalRows, finalCols, CV_8UC3);
    cv::resize(big, resized, cv::Size(finalCols, finalRows));

    if (ms > 0)
    {
      show(title, resized, ms);
    }

    return resized;
  }



  /**
    @brief Scales an image from its original format to CV_8UC1
    @param[in] inImage [const cv::Mat&] The image to show
    @param[in] ms [const int&] How many ms the showing lasts
    @return void
   **/
  cv::Mat Visualization::scaleImageForVisualization(
    const cv::Mat& inImage,
    const int& method)
  {
    cv::Mat outImage;

    double min;
    double max;
    cv::minMaxIdx(inImage, &min, &max);

    if (method == 0)
    {
      if (max != min)
      {
        outImage = (inImage - min) * 255 / (max - min);
        outImage = cv::abs(outImage);
        outImage.convertTo(outImage, CV_8UC1);
      }
      else
      {
        cv::convertScaleAbs(inImage, outImage, 255 / max);
      }
    }
    else if (method == 1)
    {
      cv::convertScaleAbs(inImage, outImage, 255 / max);
    }
    else if (method == 2)
    {
      cv::equalizeHist(scaleImageForVisualization(inImage, 0), outImage);
    }

    return outImage;
  }



  /**
    @brief Overrides the cv::imshow function.
    @param[in] windowTitle [const std::string&] The window title
    @param[in] inImage [const cv::Mat&] The image to show
    @param[in] ms [const int&] How many ms the showing lasts
    @return void
   **/
  void Visualization::show(
    const std::string& windowTitle,
    const cv::Mat& inImage,
    const int& ms)
  {
    cv::imshow(windowTitle, inImage);
    cv::waitKey(ms);
  }



  /**
    @brief Depicts the contents of a RoisConveyor on an image
    @param[in] windowTitle [const std::string&] The window title
    @param[in] inImage [const cv::Mat&] The image to show
    @param[in] conveyor [const RoisConveyor&] The rois conveyor
    @param[in] ms [const int&] How many ms the showing lasts
    @param[in] msgs [const std::vector<std::string>&] Message to show to
    each keypoint
    @param[in] hz [const float&] If positive holds the Hz
    @return void
   **/
  cv::Mat Visualization::showRois(
    const std::string& windowTitle,
    const cv::Mat& inImage,
    const RoisConveyor& conveyor,
    const int& ms,
    const std::vector<std::string>& msgs,
    const float& hz)
  {
    cv::Mat img;
    if (inImage.type() != CV_8UC1 || inImage.type() != CV_8UC3)
    {
      img = scaleImageForVisualization(inImage,
        Parameters::Image::scale_method);
    }
    else
    {
      inImage.copyTo(img);
    }

    // Construct a keypoints vector to feed into the cv::drawKeypoints method
    std::vector<cv::KeyPoint> keypointsVector;
    for (int i = 0; i < conveyor.size(); i++)
    {
      keypointsVector.push_back(conveyor.rois[i].keypoint);
    }

    cv::drawKeypoints(img, keypointsVector, img, CV_RGB(0, 255, 0),
      cv::DrawMatchesFlags::DEFAULT);

    for (unsigned int i = 0; i < conveyor.size(); i++)
    {
      for (unsigned int j = 0; j < conveyor.rois[i].outline.size(); j++)
      {
        img.at<unsigned char>(conveyor.rois[i].outline[j].y,
          3 * conveyor.rois[i].outline[j].x + 2) = 0;

        img.at<unsigned char>(conveyor.rois[i].outline[j].y,
          3 * conveyor.rois[i].outline[j].x + 1) = 255;

        img.at<unsigned char>(conveyor.rois[i].outline[j].y,
          3 * conveyor.rois[i].outline[j].x + 0) = 0;
      }
    }

    for (int i = 0; i < conveyor.size(); i++)
    {
      for (int j = 0; j < 4; j++)
      {
        cv::line(img, conveyor.rois[i].rectangle[j],
          conveyor.rois[i].rectangle[(j + 1) % 4], CV_RGB(255, 0, 0), 1, 8);
      }

      if (msgs.size() == conveyor.size())
      {
        cv::putText(img, msgs[i].c_str(),
          cvPoint(conveyor.rois[i].keypoint.pt.x - 20,
            conveyor.rois[i].keypoint.pt.y - 20),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 50, 50), 1, CV_AA);
      }
    }

    if (hz > 0)
    {
      cv::putText(img,
        (TOSTR(hz)+std::string("Hz")).c_str(),
        cvPoint(20, 20),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);
    }

    if (ms >= 0)
    {
      Visualization::show(windowTitle.c_str(), img, ms);
    }

    return img;
  }



  /**
    @brief Depicts the keypoints and bounding boxes
    @param[in] windowTitle [const std::string&] The window title
    @param[in] inImage [const cv::Mat&] The image to show
    @param[in] ms [const int&] How many ms the showing lasts
    @param[in] keypoints [const std::vector<cv::KeyPoint>&] The keypoints
    @return void
   **/
  cv::Mat Visualization::showKeypoints(
    const std::string& windowTitle,
    const cv::Mat& inImage,
    const int& ms,
    const std::vector<cv::KeyPoint>& keypoints)
  {
    cv::Mat img;
    if (inImage.depth() != CV_8U)
    {
      img = scaleImageForVisualization(inImage, Parameters::Image::scale_method);
    }
    else
    {
      inImage.copyTo(img);
    }

    cv::drawKeypoints(img, keypoints, img, CV_RGB(255, 0, 0),
      cv::DrawMatchesFlags::DEFAULT);

    if (ms >= 0)
    {
      Visualization::show(windowTitle.c_str(), img, ms);
    }

    return img;
  }



  /**
    @brief Overrides the cv::imshow function. Provides image scaling from
    the image's original forma to CV_8UC1 format
    @param[in] windowTitle [const std::string&] The window title
    @param[in] inImage [const cv::Mat&] The image to show
    @param[in] ms [const int&] How many ms the showing lasts
    @return void
   **/
  void Visualization::showScaled(
    const std::string& windowTitle,
    const cv::Mat& inImage,
    const int& ms)
  {
    cv::imshow(windowTitle,
      scaleImageForVisualization(inImage, Parameters::Image::scale_method));
    cv::waitKey(ms);
  }

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton_rpi2
