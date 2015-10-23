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

#include "utils/parameters.h"

/**
  @brief The namespaces for this package
 **/
namespace flir_lepton
{
namespace flir_lepton_image_processing
{
  //////////////////// Blob detection - specific parameters ////////////////////

  int Parameters::Blob::min_threshold = 0;
  int Parameters::Blob::max_threshold = 200;
  int Parameters::Blob::threshold_step = 100;
  int Parameters::Blob::min_area = 550;
  int Parameters::Blob::max_area = 300000;
  double Parameters::Blob::min_convexity = 0;
  double Parameters::Blob::max_convexity = 100;
  double Parameters::Blob::min_inertia_ratio = 0;
  double Parameters::Blob::max_circularity = 1.0;
  double Parameters::Blob::min_circularity = 0.3;
  bool Parameters::Blob::filter_by_color = 0;
  bool Parameters::Blob::filter_by_circularity = 1;


  ///////////////////////// Debug-specific parameters //////////////////////////


  // Show the thermal image that arrives in the thermal node
  bool Parameters::Debug::show_thermal_image = false;

  // In the terminal's window, show the probabilities of candidate holes
  bool Parameters::Debug::show_probabilities = false;

  bool Parameters::Debug::show_find_rois = false;
  int Parameters::Debug::show_find_rois_size = 1000;

  bool Parameters::Debug::show_denoise_edges = false;
  int Parameters::Debug::show_denoise_edges_size = 900;

  bool Parameters::Debug::show_connect_pairs = false;
  int Parameters::Debug::show_connect_pairs_size = 1200;

  bool Parameters::Debug::show_get_shapes_clear_border = false;
  int Parameters::Debug::show_get_shapes_clear_border_size  = 1200;

  ////////////////// Parameters specific to the Thermal node ////////////////////

  // The thermal detection method
  // If set to 0 process the binary image acquired from temperatures MultiArray
  // If set to 1 process the sensor/Image from thermal sensor
  int Parameters::Thermal::detection_method = 0;

  // The probability extraction method
  // 0 for Gaussian function
  // 1 for Logistic function
  int Parameters::Thermal::probability_method = 1;
  float Parameters::Thermal::min_thermal_probability = 0.3;

  // Gausian variables
  float Parameters::Thermal::optimal_temperature = 35;
  float Parameters::Thermal::tolerance = 10;

  // Logistic variables
  float Parameters::Thermal::low_acceptable_temperature = 32;
  float Parameters::Thermal::high_acceptable_temperature = 38;

  float Parameters::Thermal::left_tolerance = 4;
  float Parameters::Thermal::right_tolerance = 8;

  ///////////// Parameters of acceptable temperature for threshold /////////////
  float Parameters::Thermal::low_temperature = 28;
  float Parameters::Thermal::high_temperature = 40;

  ////////////////////// Parameters of the thermal image ///////////////////////
  int Parameters::ThermalImage::WIDTH = 80;
  int Parameters::ThermalImage::HEIGHT = 60;

  ///////////////////// Edge detection specific parameters /////////////////////

  // canny parameters
  int Parameters::Edge::canny_ratio = 3;
  int Parameters::Edge::canny_kernel_size = 3;
  int Parameters::Edge::canny_low_threshold = 50;
  int Parameters::Edge::canny_blur_noise_kernel_size = 3;

  // The opencv edge detection method:
  // 0 for the Canny edge detector
  // 1 for the Scharr edge detector
  // 2 for the Sobel edge detector
  // 3 for the Laplacian edge detector
  // 4 for mixed Scharr / Sobel edge detection
  int Parameters::Edge::edge_detection_method = 2;

  // Threshold parameters
  int Parameters::Edge::denoised_edges_threshold = 10;

  // When mixed edge detection is selected, this toggle switch
  // is needed in order to shift execution from one edge detector
  // to the other.
  // 1 for the Scharr edge detector,
  // 2 for the Sobel edge detector
  int Parameters::Edge::mixed_edges_toggle_switch = 1;

  ////////////////// Image representation specific parameters //////////////////

  // The depth sensor's horizontal field of view in rads
  float Parameters::Image::horizontal_field_of_view =
    static_cast<float>(58) / 180 * M_PI;

  // The depth sensor's vertical field of view in rads
  float Parameters::Image::vertical_field_of_view =
    static_cast<float>(45) / 180 * M_PI;

  // Depth and RGB images' representation method.
  // 0 if image used is used as obtained from the image sensor
  // 1 through wavelet analysis
  int Parameters::Image::image_representation_method = 0;

  // Method to scale the CV_32F image to CV_8UC1
  int Parameters::Image::scale_method = 0;

  // Term criteria for segmentation purposes
  int Parameters::Image::term_criteria_max_iterations = 5;
  double Parameters::Image::term_criteria_max_epsilon = 1;


  /////////////////// Outline discovery specific parameters ////////////////////

  // The detection method used to obtain the outline of a blob
  // 0 for detecting by means of brushfire
  // 1 for detecting by means of raycasting
  int Parameters::Outline::outline_detection_method = 0;

  // When using raycast instead of brushfire to find the (approximate here)
  // outline of blobs, raycast_keypoint_partitions dictates the number of
  // rays, or equivalently, the number of partitions in which the blob is
  // partitioned in search of the blob's borders
  int Parameters::Outline::raycast_keypoint_partitions = 32;

  // Loose ends connection parameters
  int Parameters::Outline::AB_to_MO_ratio = 4;
  int Parameters::Outline::minimum_curve_points = 50;

}  // namespace flir_lepton_image_processing
}  // namespace flir_lepton
