/*********************************************************************
 * *
 * * Software License Agreement (BSD License)
 * *
 * *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
 * *  All rights reserved.
 * *
 * *  Redistribution and use in source and binary forms, with or without
 * *  modification, are permitted provided that the following conditions
 * *  are met:
 * *
 * *   * Redistributions of source code must retain the above copyright
 * *     notice, this list of conditions and the following disclaimer.
 * *   * Redistributions in binary form must reproduce the above
 * *     copyright notice, this list of conditions and the following
 * *     disclaimer in the documentation and/or other materials provided
 * *     with the distribution.
 * *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 * *     contributors may be used to endorse or promote products derived
 * *     from this software without specific prior written permission.
 * *
 * *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * *  POSSIBILITY OF SUCH DAMAGE.
 * *
 * * Author: Konstantinos Panayiotou, Angelos Triantafyllidis,
 * *  Tsirigotis Christos
 * * Maintainer: Konstantinos Panayiotou
 * * Email: klpanagi@gmail.com
 * *
 * *********************************************************************/

#include "flir_lepton_hw_iface.h"
#include "utils/utils.h"

/* ---< SPI interface related >--- */
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <ros/package.h>
/* ------------------------------- */


namespace flir_lepton
{
  FlirLeptonHWIface::FlirLeptonHWIface(
    const std::string& ns):
    nh_(ns),
    image_encoding_("mono8"),
    MAX_RESETS_ERROR(750),
    MAX_RESTART_ATTEMPS_EXIT(5)
  {
    loadParameters();
    rawBuffer_ = flirSpi_.makeFrameBuffer();

    calibMap_ = Utils::loadThermalCalibMap(calibFileUri_);

    openDevice();
    temper_publisher_ = nh_.advertise<flir_lepton_ros_comm::TemperaturesMsg>(
      temper_topic_, 1);
    image_publisher_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 1);
  }



  FlirLeptonHWIface::~FlirLeptonHWIface()
  {
    closeDevice();
    delete[] rawBuffer_;
  }



  void FlirLeptonHWIface::loadParameters(void)
  {
    int param;
    std::string calibFileName;

    /* ----------- Load Parameters ------------ */
    // Load calibration dataset file name. Datasets are stored under the
    // flir_lepton_auxiliary package, into the datasets directory.
    // (flir_lepton_auxiliary/datasets)
    nh_.param<std::string>("dataset_file_name", calibFileName,
      "dataset_spline");
    // Search for flir_lepton_auxiliary package absolute path, using rospack.
    std::string datasetPath = ros::package::getPath("flir_lepton_auxiliary") +
      "/datasets/";
    calibFileUri_ = datasetPath + calibFileName;

    ROS_INFO("Temperature calibration dataset file: [%s]",
      calibFileUri_.c_str());

    nh_.param<std::string>("flir_urdf/camera_optical_frame", frame_id_,
      "/flir_optical_frame");
    nh_.param<int32_t>("thermal_image/height", param, 60);
    imageHeight_ = param;
    nh_.param<int32_t>("thermal_image/width", param, 80);
    imageWidth_ = param;
    nh_.param<std::string>("published_topics/flir_image_topic", image_topic_,
      "/flir_lepton/image");
    nh_.param<std::string>("published_topics/flir_temper_topic",
      temper_topic_, "flir_lepton/temperatures");
    /* ----------------------------------------- */

    flirSpi_.configFlirSpi(nh_);
  }


  void FlirLeptonHWIface::FlirSpi::configFlirSpi(
    const ros::NodeHandle& nh)
  {
    int param;
    mode = SPI_MODE_3;
    nh.param<int32_t>("iface/bits", param, 8);
    bits = param;
    nh.param<int32_t>("iface/speed", param, 16000000);
    speed = param;
    nh.param<int32_t>("iface/delay", param, 0);
    delay = param;
    nh.param<int32_t>("iface/packet_size", param, 164);
    packet_size = param;
    nh.param<int32_t>("iface/packets_per_frame", param, 60);
    packets_per_frame = param;
    packet_size_uint16 = packet_size / 2;
    frame_size_uint16 = packet_size_uint16 * packets_per_frame;
    nh.param<std::string>("iface/device_port", devicePort, "/dev/spidev0.0");
  }



  uint8_t* FlirLeptonHWIface::FlirSpi::makeFrameBuffer(void)
  {
    return new uint8_t[packet_size * packets_per_frame];
  }



  void FlirLeptonHWIface::run(void)
  {
    readFrame(&rawBuffer_);
    now_ = ros::Time::now();
    frameData_.clear();
    uint16_t minValue, maxValue;
    processFrame(rawBuffer_, frameData_, minValue, maxValue);

    // Sensor_msgs/Image
    sensor_msgs::Image thermalImage;

    // Custom message
    flir_lepton_ros_comm::TemperaturesMsg temperMsg;

    // Create the Image message
    craftImageMsg(frameData_, &thermalImage, minValue, maxValue);

    // Create the custom message
    craftTemperMsg(frameData_, temperMsg, minValue, maxValue);

    /* --------< Publish Messages >-------- */
    image_publisher_.publish(thermalImage);
    temper_publisher_.publish(temperMsg);
    /* ------------------------------------ */
  }



  void FlirLeptonHWIface::readFrame(uint8_t** frame_buffer)
  {
    int packet_number = -1;
    int resets = 0;
    int restarts = 0;

    for (uint16_t i = 0; i < flirSpi_.packets_per_frame; i++)
    {
      // flir sends discard packets that we need to resolve
      read(flirSpi_.handler, &(*frame_buffer)[flirSpi_.packet_size * i],
        sizeof(uint8_t) * flirSpi_.packet_size);
      packet_number = (*frame_buffer)[i * flirSpi_.packet_size + 1];
      if (packet_number != i)
      {
        // if it is a drop packet, reset i
        i = -1;
        resets += 1;
        // sleep for 1ms
        ros::Duration(0.01).sleep();

        // If resets reach this value, we assume an error on communication with
        // flir-lepton sensor. Perform communication restart
        if (resets == MAX_RESETS_ERROR) //Reach 750 sometimes
        {
          restarts ++;
          ROS_ERROR("[Flir-Lepton]: Error --> resets numbered at [%d]", resets);
          closeDevice();
          ros::Duration(2.0).sleep();
          openDevice();
          resets = 0;
        }

        // If true we assume an exit status.. Kill process and exit
        if (restarts > MAX_RESTART_ATTEMPS_EXIT)
        {
          ROS_FATAL("[Flir-Lepton]: Cannot communicate with sensor. Exiting...");
          ros::shutdown();
          exit(1);
        }
      }
    }
  }



  void FlirLeptonHWIface::processFrame(
    uint8_t* frame_buffer, std::vector<uint16_t>& rawData,
    uint16_t& minValue, uint16_t& maxValue)
  {
    int row, column;
    uint16_t value;
    minValue = -1;
    maxValue = 0;
    uint16_t* frame_buffer_16 = (uint16_t*) frame_buffer;
    uint16_t temp;
    uint16_t diff;
    float scale;
    std::vector<int> v;

    for (int i = 0; i < flirSpi_.frame_size_uint16; i++)
    {
      //Discard the first 4 bytes. it is the header.
      if (i % flirSpi_.packet_size_uint16 < 2) continue;

      temp = frame_buffer[i*2];
      frame_buffer[i*2] = frame_buffer[i*2+1];
      frame_buffer[i*2+1] = temp;
      value = frame_buffer_16[i];
      rawData.push_back(value);
      if (value > maxValue) maxValue = value;
      if (value < minValue) minValue = value;
    }
  }



  void FlirLeptonHWIface::craftImageMsg(
    const std::vector<uint16_t>& rawData,
    sensor_msgs::Image* thermalImage, uint16_t minValue, uint16_t maxValue)
  {
    thermalImage->header.stamp = now_;
    thermalImage->header.frame_id = frame_id_;
    thermalImage->height = imageHeight_;
    thermalImage->width = imageWidth_;
    thermalImage->encoding = image_encoding_;
    thermalImage->is_bigendian = 0;
    thermalImage->step = imageWidth_ * sizeof(uint8_t);

    for (int i = 0; i < imageWidth_; i++) {
      for (int j = 0; j < imageHeight_; j++) {
        uint8_t value = Utils::signalToImageValue(
          rawData.at(i * imageHeight_ + j), minValue, maxValue);
        thermalImage->data.push_back(value);
      }
    }
  }


  void FlirLeptonHWIface::craftTemperMsg(
    const std::vector<uint16_t>& rawData,
    flir_lepton_ros_comm::TemperaturesMsg& temperMsg, uint16_t minValue,
    uint16_t maxValue)
  {
     float temperSum = 0.0;
     temperMsg.header.stamp = now_;
     temperMsg.header.frame_id = frame_id_;
     scene_tempers_.clear();

     // Vector containing the temperatures in image after calibration and vector
     // with signal raw values
     for (int i = 0; i < imageHeight_; i++) {
       for (int j = 0; j < imageWidth_; j++) {

         float value = Utils::signalToTemperature(
           rawData.at(i * imageWidth_ + j), calibMap_);
         temperMsg.temperatures.data.push_back(value);
         scene_tempers_.push_back(value);
         temperSum += value;
       }
     }

     scene_avgTemper_ = temperSum / (imageHeight_ * imageWidth_);

     temperMsg.temperatures.layout.dim.push_back(
       std_msgs::MultiArrayDimension());
     temperMsg.temperatures.layout.dim[0].size = 60;

     temperMsg.temperatures.layout.dim.push_back(std_msgs::MultiArrayDimension());
     temperMsg.temperatures.layout.dim[1].size = 80;
  }


  void FlirLeptonHWIface::openDevice(void)
  {
    flirSpi_.handler = open(flirSpi_.devicePort.c_str(), O_RDWR);
    if (flirSpi_.handler < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't open SPI device");
      exit(1);
    }

    statusValue_ = ioctl(flirSpi_.handler, SPI_IOC_WR_MODE, &flirSpi_.mode);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI-mode (WR)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(flirSpi_.handler, SPI_IOC_RD_MODE, &flirSpi_.mode);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI-mode (RD)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(flirSpi_.handler, SPI_IOC_WR_BITS_PER_WORD, &flirSpi_.bits);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI bitsperWord (WD)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(flirSpi_.handler, SPI_IOC_RD_BITS_PER_WORD, &flirSpi_.bits);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI bitsperWord (RD)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(flirSpi_.handler, SPI_IOC_WR_MAX_SPEED_HZ, &flirSpi_.speed);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI speed (WD)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(flirSpi_.handler, SPI_IOC_RD_MAX_SPEED_HZ, &flirSpi_.speed);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI speed (RD)...ioctl failed");
      exit(1);
    }
    ROS_WARN("[Flir-Lepton]: Opened SPI Port");

    //ROS_INFO("[SPI Device information]:");
    //std::cout << "  * Device Port: " << flirSpi_.devicePort << std::endl;
  }



  void FlirLeptonHWIface::closeDevice(void)
  {
    statusValue_ = close(flirSpi_.handler);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Could not close SPI device");
      exit(1);
    }
    ROS_WARN("[Flir-Lepton]: Closed SPI Port");
  }

}  // namespace flir_lepton_rpi2
