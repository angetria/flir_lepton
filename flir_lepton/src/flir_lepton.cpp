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

#include "flir_lepton/flir_lepton.h"
#include "flir_lepton/utils.h"

/* ---< SPI interface related >--- */
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
/* ------------------------------- */


namespace flir_lepton_rpi2
{
namespace flir_lepton
{
  FlirLeptonHardwareInterface::FlirLeptonHardwareInterface(
    const std::string& ns):
    nh_(ns),
    device_("/dev/spidev0.0"),
    image_encoding_("mono8"),
    MAX_RESETS_ERROR(750),
    MAX_RESTART_ATTEMPS_EXIT(5)
  {
    loadParameters();
    frame_buffer_ = flirSpi_.makeFrameBuffer();

    calibMap_ = Utils::loadThermalCalibMap(calibFileUri_);

    openDevice();
    fusedMsg_publisher_ = nh_.advertise<flir_lepton_ros_comm::FlirLeptonMsg>(
      fusedMsg_topic_, 1);
    image_publisher_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 1);
  }



  FlirLeptonHardwareInterface::~FlirLeptonHardwareInterface()
  {
    closeDevice();
    delete[] frame_buffer_;
  }



  void FlirLeptonHardwareInterface::loadParameters(void)
  {
    int param;
    /* ----------- Load Parameters ------------ */
    nh_.param<std::string>("dataset/spline_interpolated_data", calibFileUri_,
      "/home/pandora/pandora_ws/src/rpi_hardware_interface/data" \
      "/flir_lepton/dataset_spline_interp.pandora");

    nh_.param<std::string>("flir_urdf/camera_optical_frame", frame_id_,
      "/flir_optical_frame");
    nh_.param<int32_t>("thermal_image/height", param, 60);
    imageHeight_ = param;
    nh_.param<int32_t>("thermal_image/width", param, 80);
    imageWidth_ = param;
    nh_.param<std::string>("published_topics/flir_image_topic", image_topic_,
      "/rpi2/thermal/image");
    nh_.param<std::string>("published_topics/flir_fused_topic",
      fusedMsg_topic_, "/rpi2/thermal/fused_msg");
    /* ----------------------------------------- */

    flirSpi_.configFlirSpi(nh_);
  }



  void FlirLeptonHardwareInterface::FlirSpi::configFlirSpi(
    const ros::NodeHandle& nh)
  {
    int param;
    mode = SPI_MODE_3;
    nh.param<int32_t>("flir_spi/bits", param, 8);
    bits = param;
    nh.param<int32_t>("flir_spi/speed", param, 24000000);
    speed = param;
    nh.param<int32_t>("flir_spi/delay", param, 0);
    delay = param;
    nh.param<int32_t>("flir_spi/packet_size", param, 164);
    packet_size = param;
    nh.param<int32_t>("flir_spi/packets_per_frame", param, 60);
    packets_per_frame = param;
    packet_size_uint16 = packet_size / 2;
    frame_size_uint16 = packet_size_uint16 * packets_per_frame;
  }



  uint8_t* FlirLeptonHardwareInterface::FlirSpi::makeFrameBuffer(void)
  {
    return new uint8_t[packet_size * packets_per_frame];
  }



  void FlirLeptonHardwareInterface::run(void)
  {
    readFrame(&frame_buffer_);
    now_ = ros::Time::now();
    thermal_signals_.clear();
    uint16_t minValue, maxValue;
    processFrame(frame_buffer_, &thermal_signals_, &minValue, &maxValue);

    // Sensor_msgs/Image
    sensor_msgs::Image thermalImage;

    // Custom message
    flir_lepton_ros_comm::FlirLeptonMsg fusedMsg;

    // Create the Image message
    craftImageMsg(thermal_signals_, &thermalImage, minValue, maxValue);

    // Create the custom message
    craftFusedMsg(thermal_signals_, &fusedMsg, minValue, maxValue);

    /* --------< Publish Messages >-------- */
    image_publisher_.publish(thermalImage);
    fusedMsg_publisher_.publish(fusedMsg);
    /* ------------------------------------ */
  }



  void FlirLeptonHardwareInterface::readFrame(uint8_t** frame_buffer)
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



  void FlirLeptonHardwareInterface::processFrame(
    uint8_t* frame_buffer, std::vector<uint16_t>* thermal_signals,
    uint16_t* minValue, uint16_t* maxValue)
  {
    int row, column;
    uint16_t value;
    *minValue = -1;
    *maxValue = 0;
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
      thermal_signals->push_back(value);
      if (value > *maxValue) *maxValue = value;
      if (value < *minValue) *minValue = value;
    }
  }



  void FlirLeptonHardwareInterface::craftImageMsg(
    const std::vector<uint16_t>& thermal_signals,
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
          thermal_signals.at(i * imageHeight_ + j), minValue, maxValue);
        thermalImage->data.push_back(value);
      }
    }
  }



  void FlirLeptonHardwareInterface::craftFusedMsg(
    const std::vector<uint16_t>& thermal_signals,
    flir_lepton_ros_comm::FlirLeptonMsg* flirMsg, uint16_t minValue, uint16_t maxValue)
  {
    float temperSum = 0;
    flirMsg->header.stamp = now_;
    flirMsg->header.frame_id = frame_id_;

    craftImageMsg(thermal_signals, &flirMsg->thermalImage, minValue, maxValue);

    scene_tempers_.clear();
    // Vector containing the temperatures in image after calibration and vector
    // with signal raw values
    for (int i = 0; i < imageHeight_; i++) {
      for (int j = 0; j < imageWidth_; j++) {
        flirMsg->rawValues.data.push_back(thermal_signals.at(
            i * imageWidth_ + j));

        float value = Utils::signalToTemperature(
          thermal_signals.at(i * imageWidth_ + j), calibMap_);
        flirMsg->temperatures.data.push_back(value);
        scene_tempers_.push_back(value);
        temperSum += value;
      }
    }
    scene_avgTemper_ = temperSum / (imageHeight_ * imageWidth_);

    flirMsg->temperatures.layout.dim.push_back(std_msgs::MultiArrayDimension());
    flirMsg->temperatures.layout.dim[0].size = 60;

    flirMsg->temperatures.layout.dim.push_back(std_msgs::MultiArrayDimension());
    flirMsg->temperatures.layout.dim[1].size = 80;
  }



  void FlirLeptonHardwareInterface::openDevice(void)
  {
    flirSpi_.handler = open(device_.c_str(), O_RDWR);
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
  }



  void FlirLeptonHardwareInterface::closeDevice(void)
  {
    statusValue_ = close(flirSpi_.handler);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Could not close SPI device");
      exit(1);
    }
    ROS_WARN("[Flir-Lepton]: Closed SPI Port");
  }

}  // namespace flir_lepton
}  // namespace flir_lepton_rpi2
