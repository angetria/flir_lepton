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

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_UTILS_DEFINES_H
#define FLIR_LEPTON_IMAGE_PROCESSING_UTILS_DEFINES_H

#include <iostream>
// #include <fstream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// #include <image_transport/image_transport.h>
#include "std_msgs/String.h"

#include "utils/timer.h"

#define DEBUG_SHOW
#define DEBUG_TIME

// Transforms a float number to string
#define TOSTR(x) static_cast<std::ostringstream&>(\
  (std::ostringstream() << std::dec << x)).str()

// Takes the file name from an absolute path
#define LPATH(x) x.substr(x.find_last_of("/") + 1 , \
  x.size() - x.find_last_of("/") - 1)

// Shortcut to std::string
#define STR(x) std::string(x)

// The string identifier of the thermal detector package.
// It must comply with the sub-namespace set in the package's launchers
#define PKG_NAME "flir_lepton_image_processing"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton
{
namespace flir_lepton_image_processing
{

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_UTILS_DEFINES_H
