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

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_UTILS_PARAMETERS_H
#define FLIR_LEPTON_IMAGE_PROCESSING_UTILS_PARAMETERS_H

#include "utils/defines.h"
#include <dynamic_reconfigure/server.h>
#include <flir_lepton_image_processing/thermal_cfgConfig.h>

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton
{
namespace flir_lepton_image_processing
{
  /**
    @struct Parameters
    @brief Provides flexibility by parameterizing variables needed by the
    hole detector package
   **/
  struct Parameters
  {
    //  Blob detection - specific parameters
    struct Blob
    {
      static int min_threshold;
      static int max_threshold;
      static int threshold_step;
      static int min_area;
      static int max_area;
      static double min_convexity;
      static double max_convexity;
      static double min_inertia_ratio;
      static double max_circularity;
      static double min_circularity;
      static bool filter_by_color;
      static bool filter_by_circularity;
    };

    //  Debug-specific parameters
    struct Debug
    {
      //  Show the thermal image that arrives in the thermal node
      static bool show_thermal_image;

      //  In the terminal's window, show the probabilities of candidate rois
      static bool show_probabilities;

      static bool show_find_rois;
      static int show_find_rois_size;

      static bool show_denoise_edges;
      static int show_denoise_edges_size;

      static bool show_connect_pairs;
      static int show_connect_pairs_size;

      static bool show_get_shapes_clear_border;
      static int show_get_shapes_clear_border_size;
    };

    //  Parameters specific to the Thermal node
    struct Thermal
    {
      // The thermal detection method
      // If set to 0 process the binary image acquired from temperatures MultiArray
      // If set to 1 process the sensor/Image from thermal sensor
      static int detection_method;

      //  The probability extraction method
      //  0 for Gaussian function
      //  1 for Logistic function
      static int probability_method;
      static float min_thermal_probability;

      //  Gausian variables
      static float optimal_temperature;
      static float tolerance;

      //  Logistic variables
      static float low_acceptable_temperature;
      static float high_acceptable_temperature;

      static float left_tolerance;
      static float right_tolerance;

      //  Low and High acceptable temperatures for thresholding
      static float low_temperature;
      static float high_temperature;
    };

    //  Thermal image parameters
    struct ThermalImage
    {
      //  Thermal image width and height
      static int WIDTH;
      static int HEIGHT;
    };

    //  Edge detection specific parameters
    struct Edge
    {
      //  canny parameters
      static int canny_ratio;
      static int canny_kernel_size;
      static int canny_low_threshold;
      static int canny_blur_noise_kernel_size;

      //  The opencv edge detection method:
      //  0 for the Canny edge detector
      //  1 for the Scharr edge detector
      //  2 for the Sobel edge detector
      //  3 for the Laplacian edge detector
      //  4 for mixed Scharr / Sobel edge detection
      static int edge_detection_method;

      //  Threshold parameters
      static int denoised_edges_threshold;

      //  When mixed edge detection is selected, this toggle switch
      //  is needed in order to shift execution from one edge detector
      //  to the other.
      //  1 for the Scharr edge detector,
      //  2 for the Sobel edge detector
      static int mixed_edges_toggle_switch;
    };

    //  Image representation specific parameters
    struct Image
    {
      //  The thermal sensor's horizontal field of view
      static float horizontal_field_of_view;

      //  The thermal sensor's vertical field of view
      static float vertical_field_of_view;

      //  Depth and RGB images' representation method.
      //  0 if image used is used as obtained from the image sensor
      //  1 through wavelet analysis
      static int image_representation_method;

      //  Method to scale the CV_32F images to CV_8UC1
      static int scale_method;

      //  Term criteria for segmentation purposes
      static int term_criteria_max_iterations;
      static double term_criteria_max_epsilon;
    };



    //  Outline discovery specific parameters
    struct Outline
    {
      //  The detection method used to obtain the outline of a blob
      //  0 for detecting by means of brushfire
      //  1 for detecting by means of raycasting
      static int outline_detection_method;

      //  When using raycast instead of brushfire to find the (approximate here)
      //  outline of blobs, raycast_keypoint_partitions dictates the number of
      //  rays, or equivalently, the number of partitions in which the blob is
      //  partitioned in search of the blob's borders
      static int raycast_keypoint_partitions;

      //  Loose ends connection parameters
      static int AB_to_MO_ratio;
      static int minimum_curve_points;
    };
  };

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_UTILS_PARAMETERS_H
