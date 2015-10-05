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
 * Authors: Alexandros Philotheou, Manos Tsardoulias,
 * Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#include "processing_node/process.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Default constructor. Initiates communications, loads parameters.
    @return void
   **/
  Process::Process(void)
  {
    // Acquire the names of topics which the process node will be having
    // transactionary affairs with
    getTopicNames();

    // Subscribe to the thermal image published by flir lepton camera.
    thermalImageSubscriber_ = nodeHandle_.subscribe(thermalImageTopic_, 1,
      &Process::inputThermalImageCallback, this);

    // Advertise the candidate Rois found by the process node.
    candidateRoisPublisher_ = nodeHandle_.advertise
      <flir_lepton_image_processing::CandidateRoisVectorMsg>(
      candidateRoisTopic_, 1);

    // Advertise the candidate Rois Alert message found by the process node.
    candidateRoisAlertPublisher_ = nodeHandle_.advertise
      <flir_lepton_image_processing::ThermalAlertVector>(
      candidateRoisAlertTopic_, 1);

    // The dynamic reconfigure (process) parameter's callback
    server.setCallback(boost::bind(&Process::parametersCallback, this, _1, _2));

    ROS_INFO_NAMED(PKG_NAME, "[Process node] Initiated");
  }



  /**
    @brief Default destructor
    @return void
   **/
  Process::~Process(void)
  {
    ROS_INFO_NAMED(PKG_NAME, "[Process node] Terminated");
  }


  /**
    @brief Callback for the thermal image received by the camera.
    The thermal image message is unpacked in a cv::Mat image.
    Thermal rois are then located inside this image.
    @param msg [const flir_lepton_ros_comm::FlirLeptonMsg&]
    The thermal image message
    @return void
   **/
  void Process::inputThermalImageCallback(
    const flir_lepton_ros_comm::FlirLeptonMsg& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("FlirLeptonImageCallback", "", true);
    #endif

    ROS_INFO_NAMED(PKG_NAME, "Process node callback");

    RoisConveyor rois;
    cv::Mat thermalImage;

    // Detection method = 0 --> process the binary image acquired from temperatures
    // Detection method = 1 --> process the sensor image
    switch (Parameters::Thermal::detection_method)
    {
      case 0:
        // Obtain the thermal message and extract the temperature information.
        // Convert this information to cv::Mat in order to be processed.
        // It's format will be CV_8UC1
        thermalImage = MessageConversions::convertFloat32MultiArrayToMat
          (msg.temperatures);

        // Apply double threshold(up and down) in the temperature image.
        // This way we get rid of unwanted temperature regions(black).
        cv::inRange(
        thermalImage, cv::Scalar(Parameters::Thermal::low_temperature),
        cv::Scalar(Parameters::Thermal::high_temperature), thermalImage);

        #ifdef DEBUG_SHOW
        if (Parameters::Debug::show_thermal_image)
        {
          Visualization::showScaled("Thermal temperature image", thermalImage, 1);
        }
        #endif

        // Locate potential Rois in the thermal image
        rois = RoiDetector::findRois(thermalImage);
        break;
      case 1:
        // Obtain the thermal image. Since the image is in a format of
        // sensor_msgs::Image, it has to be transformed into a cv format in order
        // to be processed. Its cv format will be CV_8UC1.
        MessageConversions::extractImageFromMessage(msg.thermalImage, &thermalImage,
          sensor_msgs::image_encodings::TYPE_8UC1);

        #ifdef DEBUG_SHOW
        if (Parameters::Debug::show_thermal_image)
        {
          Visualization::showScaled("Thermal sensor image", thermalImage, 1);
        }
        #endif

        // Locate potential Rois in the thermal image
        rois = RoiDetector::findRois(thermalImage);
        break;
      default:
        ROS_ERROR_STREAM("Incorrect detection method, value = " <<
          Parameters::Thermal::detection_method);
    }

    // Find the average temperature of each region of interest that is found
    // and it's probability.
    findRoisProbability(
      &rois, msg.temperatures, Parameters::Thermal::probability_method);

    // Create the candidate rois message
    flir_lepton_image_processing::CandidateRoisVectorMsg thermalCandidateRoisMsg;

    // Pack information about rois found and the thermal image
    // inside a message.
    MessageConversions::createCandidateRoisVectorMessage(rois,
      thermalImage,
      &thermalCandidateRoisMsg,
      sensor_msgs::image_encodings::TYPE_8UC1,
      msg.thermalImage);

    // Fill the thermal message to be sent
    flir_lepton_image_processing::ThermalAlert thermalMsg;
    flir_lepton_image_processing::ThermalAlertVector thermalVectorMsg;
    flir_lepton_image_processing::GeneralAlertInfo alert;

    // Finally find the yaw and pitch of each candidate roi found and
    // publish it if a roi exists.
    std::vector<RoiConveyor>::iterator iter = rois.rois.begin();

    while (iter != rois.rois.end())
    {
      // The sensor's vertical and horizontal field of view
      float hfov = Parameters::Image::horizontal_field_of_view;
      float vfov = Parameters::Image::vertical_field_of_view;

      // The frame's width and height
      int width = Parameters::ThermalImage::WIDTH;
      int height = Parameters::ThermalImage::HEIGHT;

      // The thermal roi's coordinates relative to the center of the frame
      float x = static_cast<float>(width) / 2 - iter->keypoint.pt.x;

      float y = iter->keypoint.pt.y - static_cast<float>(height) / 2;

      // The keypoint's yaw and pitch
      float yaw = atan(2 * x / width * tan(hfov / 2));
      float pitch = atan(2 * y / height * tan(vfov / 2));

      // If the probability of a roi is small discart this roi
      if (iter->roiProbability < Parameters::Thermal::min_thermal_probability)
      {
        iter = rois.rois.erase(iter);
        continue;
      }
      else
      {
        // Fill the GeneralAlertInfo message
        alert.yaw = yaw;
        alert.pitch = pitch;
        alert.probability = iter->roiProbability;
      }

      // Pass GeneralAlertInfo message to ThermalAlert and fill it's temperature
      thermalMsg.info = alert;
      thermalMsg.temperature = iter->roiTemperature;

      // Fill the ThermalAlertVector message
      thermalVectorMsg.header = msg.header;
      thermalVectorMsg.alerts.push_back(thermalMsg);

      ++iter;
    }

    // Publish the thermal message
    candidateRoisAlertPublisher_.publish(thermalVectorMsg);

    // Publish the thermal message
    candidateRoisPublisher_.publish(thermalCandidateRoisMsg);

    #ifdef DEBUG_TIME
    Timer::tick("FlirLeptonImageCallback");
    Timer::printAllMeansTree();
    #endif

    return;
  }

  /**
    @brief Acquire the names of topics which the process node will be having
    transactionary affairs with.
    @param void
    @return void
   **/
  void Process::getTopicNames()
  {
    // The namespace dictated in the launch file.
    std::string ns = nodeHandle_.getNamespace();

    // Read the name of the topic from where the process node acquires the
    // thermal image and store it in a private member variable.
    if (nodeHandle_.getParam(
        ns + "/thermal_camera_node/subscribed_topics/thermal_image_topic",
        thermalImageTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
        "[Process Node] Subscribed to the input thermal image");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[Process Node] Could not subscribe to Flir-Lepton image topic");
    }

    // Read the name of the topic to which the process node will be publishing
    // information about the candidate Rois found and store it in a private
    // member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_camera_node/published_topics/candidate_rois_topic",
        candidateRoisTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
        "[Process Node] Advertising to the candidate Rois topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[Process Node] Could not find topic in order to publish the thermal Rois");
    }

    // Read the name of the topic to which the process node will be publishing
    // information about the candidate Rois Alert message  found and store it
    // in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_camera_node/published_topics/candidate_rois_alert_topic",
        candidateRoisAlertTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
        "[Process Node] Advertising to the candidate Rois alert topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[Process Node] Could not find topic in order to publish the thermal Rois alert");
    }
  }


  /**
    @brief This function finds for each region of interest found it's
    probability based on the keypoint's average temperature.
    Then fills the probability and the temperature in the RoisConveyor struct.
    @param[out] rois [const RoisConveyor&] The regions of interest found.
    @param[in] temperatures [const Float32MultiArray&] The multiArray with
    the temperatures of the image.
    @param[in] method [const int&] Denotes the probabilities extraction
    method.
    @return void
   **/
  void Process::findRoisProbability(RoisConveyor* rois,
    const std_msgs::Float32MultiArray& temperatures, const int& method)
  {
    // The width and height of the input temperature multiarray
    int width = temperatures.layout.dim[1].size;
    int height = temperatures.layout.dim[0].size;

    // For each roi find its probability
    for (unsigned int i = 0; i < rois->size(); i++)
    {
      float average = 0;

      // Find the keypoint coordinates
      float temperatureX = rois->rois[i].keypoint.pt.x;
      float temperatureY = rois->rois[i].keypoint.pt.y;

      // Convert them to int, in order to access the point in MultiArray
      int tempX = static_cast <int> (std::floor(temperatureX));
      int tempY = static_cast <int> (std::floor(temperatureY));

      // Find the average temperature around the keypoint

      // Check if the keypoint is on the edges of the image
      if (tempX > 0 && tempX < 60 && tempY > 0 && tempY < 80)
      {
        int counter = 0;

        for (unsigned int k = (tempY - 1); k < (tempY + 2); k++)
        {
          for (unsigned int o = (tempX - 1); o < (tempX + 2); o++)
          {
            average += temperatures.data[k * width + o];
            counter++;
          }
        }
        average = average / counter;
      }
      else
      {
        // If it is on the edges it take the temperature of the keypoint it self
        average = temperatures.data[tempY * width + tempX];
      }

      // Fill the average temperature vector
      rois->rois[i].roiTemperature = average;

      // Apply gaussian function on the average temperature found
      if (method == 0)
      {
        float probability = exp(
          - pow((average - Parameters::Thermal::optimal_temperature), 2)
          / (2 * pow(Parameters::Thermal::tolerance , 2)));

        // Push the probability in the vector
        rois->rois[i].roiProbability = probability;
      }
      // Apply logistic function on the average temperature found
      else if (method == 1)
      {
        float probability = 1 / (1 + exp(-Parameters::Thermal::left_tolerance *
            (average - Parameters::Thermal::low_acceptable_temperature)))
            - 1/(1 + exp(- Parameters::Thermal::right_tolerance *
            (average - Parameters::Thermal::high_acceptable_temperature)));

        // Push the probability in the vector
        rois->rois[i].roiProbability = probability;
      }
    }
  }

  /**
    @brief The function called when a parameter is changed
    @param[in] config [const flir_lepton_image_processing::thermal_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void Process::parametersCallback(
    const flir_lepton_image_processing::thermal_cfgConfig& config,
    const uint32_t& level)
  {
    ROS_INFO_NAMED(PKG_NAME, "[Thermal node] Parameters callback called");

    //////////////////// Blob detection - specific parameters //////////////////

    Parameters::Blob::min_threshold =
      config.min_threshold;
    Parameters::Blob::max_threshold =
      config.max_threshold;
    Parameters::Blob::threshold_step =
      config.threshold_step;

    if (Parameters::Image::image_representation_method == 0)
    {
      Parameters::Blob::min_area =
        config.min_area;
      Parameters::Blob::max_area =
        config.max_area;
    }

    Parameters::Blob::min_convexity =
      config.min_convexity;
    Parameters::Blob::max_convexity =
      config.max_convexity;
    Parameters::Blob::min_inertia_ratio =
      config.min_inertia_ratio;
    Parameters::Blob::max_circularity =
      config.max_circularity;
    Parameters::Blob::min_circularity =
      config.min_circularity;
    Parameters::Blob::filter_by_color =
      config.filter_by_color;
    Parameters::Blob::filter_by_circularity =
      config.filter_by_circularity;


    ////////////////////////////// Debug parameters ////////////////////////////

    // Show the thermal image that arrives in the thermal node
    Parameters::Debug::show_thermal_image =
     config.show_thermal_image;

    Parameters::Thermal::detection_method =
      config.detection_method;

    Parameters::Debug::show_find_rois =
      config.show_find_rois;
    Parameters::Debug::show_find_rois_size =
      config.show_find_rois_size;

    Parameters::Debug::show_denoise_edges =
      config.show_denoise_edges;
    Parameters::Debug::show_denoise_edges_size =
      config.show_denoise_edges_size;

    Parameters::Debug::show_connect_pairs =
      config.show_connect_pairs;
    Parameters::Debug::show_connect_pairs_size =
      config.show_connect_pairs_size;

    Parameters::Debug::show_get_shapes_clear_border  =
      config.show_get_shapes_clear_border;
    Parameters::Debug::show_get_shapes_clear_border_size =
      config.show_get_shapes_clear_border_size;


    //------------------- Edge detection specific parameters -------------------

    // Canny parameters
    Parameters::Edge::canny_ratio =
      config.canny_ratio;
    Parameters::Edge::canny_kernel_size =
      config.canny_kernel_size;
    Parameters::Edge::canny_low_threshold =
      config.canny_low_threshold;
    Parameters::Edge::canny_blur_noise_kernel_size =
      config.canny_blur_noise_kernel_size;

    Parameters::Edge::edge_detection_method =
      config.edge_detection_method;

    // Threshold parameters
    Parameters::Edge::denoised_edges_threshold =
      config.denoised_edges_threshold;


    // Method to scale the CV_32FC1 image to CV_8UC1
    Parameters::Image::scale_method = config.scale_method;


    //----------------- Outline discovery specific parameters ------------------

    // The detection method used to obtain the outline of a blob
    // 0 for detecting by means of brushfire
    // 1 for detecting by means of raycasting
    Parameters::Outline::outline_detection_method =
      config.outline_detection_method;

    // When using raycast instead of brushfire to find the (approximate here)
    // outline of blobs, raycast_keypoint_partitions dictates the number of
    // rays, or equivalently, the number of partitions in which the blob is
    // partitioned in search of the blob's borders
    Parameters::Outline::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;

    //-------------------- Loose ends connection parameters --------------------

    Parameters::Outline::AB_to_MO_ratio = config.AB_to_MO_ratio;

    if (Parameters::Image::image_representation_method == 0)
    {
      Parameters::Outline::minimum_curve_points =
        config.minimum_curve_points;
    }

    //-------------------- Probability extraction Parameters -------------------

    // The interpolation method for noise removal
    // 0 for averaging the pixel's neighbor values
    // 1 for brushfire near
    // 2 for brushfire far
    Parameters::Thermal::probability_method = config.probability_method;

    //-------------------- Gausian variables ---------------------
    Parameters::Thermal::optimal_temperature = config.optimal_temperature;

    // As the standard deviation(tolerance) is higher we consider
    // more temperatures as valid. We handle the tolerance.
    Parameters::Thermal::tolerance = config.tolerance;

    //-------------------- Logistic variables ---------------------

    // These temperatures have 50% probability.
    // Inside their range the probability grows.
    Parameters::Thermal::low_acceptable_temperature =
      config.low_acceptable_temperature;
    Parameters::Thermal::high_acceptable_temperature =
      config.high_acceptable_temperature;

    // These variables handle the tolerance of the temperatures.
    // left is for the left side of the destributions curve
    // right is for the right side of the destributions curve
    Parameters::Thermal::left_tolerance = config.left_tolerance;
    Parameters::Thermal::right_tolerance = config.right_tolerance;
  }

}  // namespace pandora_vision
