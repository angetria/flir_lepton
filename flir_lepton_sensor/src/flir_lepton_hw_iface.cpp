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
#include "flir_lepton_utils.h"

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
  namespace flir_lepton_sensor
  {
    FlirLeptonHWIface::FlirLeptonHWIface(
      const std::string& ns):
      nh_(ns),
      imageEncoding_("mono8"),
      MAX_RESETS_ERROR(750),
      MAX_RESTART_ATTEMPS_EXIT(5)
    {
      loadParameters();

      calibMap_ = Utils::loadThermalCalibMap(calibFileUri_);

      openDevice();
      temperPublisher_ = nh_.advertise<flir_lepton_msgs::TemperaturesMsg>(
        temperTopic_, 1);
      imagePublisher_ = nh_.advertise<sensor_msgs::Image>(imageTopic_, 1);
      batchPublisher_ = nh_.advertise<flir_lepton_msgs::FlirLeptonBatchMsg>(
        batchTopic_, 1);

      flirDataFrame_.allocateBuffers();
      allocateFrameData();

      /* -------------- Initialize topic message containers ----------------- */
      thermalImage_.header.frame_id = frameId_;
      thermalImage_.height = IMAGE_HEIGHT;
      thermalImage_.width = IMAGE_WIDTH;
      thermalImage_.encoding = imageEncoding_;
      thermalImage_.is_bigendian = 0;
      thermalImage_.step = IMAGE_WIDTH * sizeof(uint8_t);

      temperMsg_.header.frame_id = frameId_;
      temperMsg_.values.layout.dim.push_back(std_msgs::MultiArrayDimension());
      temperMsg_.values.layout.dim[0].size = IMAGE_HEIGHT;
      temperMsg_.values.layout.dim.push_back(std_msgs::MultiArrayDimension());
      temperMsg_.values.layout.dim[1].size = IMAGE_WIDTH;

      batchMsg_.header.frame_id = frameId_;
      /* -------------------------------------------------------------------- */
    }


    FlirLeptonHWIface::~FlirLeptonHWIface()
    {
      closeDevice();
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

      nh_.param<std::string>("flir_urdf/camera_optical_frame", frameId_,
        "/flir_optical_frame");
      nh_.param<std::string>("published_topics/flir_image_topic", imageTopic_,
        "/flir_lepton/image");
      nh_.param<std::string>("published_topics/flir_temper_topic",
        temperTopic_, "flir_lepton/temperatures");
      nh_.param<std::string>("published_topics/flir_batch_topic",
        batchTopic_, "flir_lepton/batch");
      /* ----------------------------------------- */

      flirSpi_.configFlirSpi(nh_);

      nh_.param<int32_t>("iface/packet_size", param, 164);
      flirDataFrame_.packetSize = param;
      flirDataFrame_.packetSize16 = param / 2;
      nh_.param<int32_t>("iface/packets_per_frame", param, 60);
      flirDataFrame_.packetsPerFrame = param;

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
      nh.param<std::string>("iface/device_port", devicePort, "/dev/spidev0.0");
    }


    /*!
     *  @brief Allocate the uint16_t buffer memory
     */
    void FlirLeptonHWIface::FlirDataFrame::allocateBuffers(void)
    {
      frameBuffer = new uint8_t[packetSize * packetsPerFrame];
      cleanDataBuffer = new uint16_t[IMAGE_HEIGHT * IMAGE_WIDTH];
      frameData = new uint16_t[IMAGE_HEIGHT * IMAGE_WIDTH];
    }


    void FlirLeptonHWIface::allocateFrameData(void)
    {
      lastFrame_ = new uint16_t[IMAGE_HEIGHT * IMAGE_WIDTH];
    }


    void FlirLeptonHWIface::run(void)
    {
      readFrame();
      now_ = ros::Time::now();
      processFrame();
      uint16_t maxVal = flirDataFrame_.maxVal;
      uint16_t minVal = flirDataFrame_.minVal;

      //Batch msg creation
      fillBatchMsg();

      /* --------< Publish Messages >-------- */
      imagePublisher_.publish(thermalImage_);
      temperPublisher_.publish(temperMsg_);
      batchPublisher_.publish(batchMsg_);
      /* ------------------------------------ */
    }


    void FlirLeptonHWIface::readFrame(void)
    {
      int packetNumber = -1;
      int resets = 0;
      int restarts = 0;
      // Pointer to FlirDataFrame struct member buffer.
      uint8_t* rawBuffer = flirDataFrame_.frameBuffer;
      uint16_t* cleanData = flirDataFrame_.cleanDataBuffer;
      uint16_t* frameData = flirDataFrame_.frameData;


      uint16_t packetSize = flirDataFrame_.packetSize;
      uint16_t packetsPerFrame = flirDataFrame_.packetsPerFrame;
      uint16_t packetSize16 = packetSize / 2;
      uint16_t frameSize16 = packetSize16 * packetsPerFrame;
      uint16_t maxVal, minVal;
      uint16_t value, temp;

      /* ------------------ Read raw frame data from spi -------------------- */
      for (uint16_t i = 0; i < packetsPerFrame; i++)
      {
        read(flirSpi_.handler, &rawBuffer[packetSize * i],
          sizeof(uint8_t) * packetSize);

        // flir sends discard packets that we need to resolve
        packetNumber = rawBuffer[i * packetSize + 1];
        if (packetNumber != i)
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
      /* -------------------------------------------------------------------- */

      // Cast to uint16_t in (2 bytes).
      uint16_t* rawBuffer16 = (uint16_t*) rawBuffer;
      uint16_t cleanDataCount = 0;
      maxVal = 0;
      minVal = -1;

      // Process this acquired from spi port, frame and create the data vector
      for (int i = 0; i < frameSize16; i++)
      {
        //Discard the first 4 bytes. it is the header.
        if (i % packetSize16 < 2) continue;

        temp = rawBuffer[i * 2];
        rawBuffer[i * 2] = rawBuffer[i * 2 + 1];
        rawBuffer[i * 2 + 1] = temp;
        value = rawBuffer16[i];

        cleanData[cleanDataCount] = value;
        cleanDataCount++;

        if (value > maxVal) {maxVal = value;}
        if (value < minVal) {minVal = value;}
      }

      // Copy cleanData to class frameData buffer
      memcpy(frameData, cleanData, IMAGE_HEIGHT * IMAGE_WIDTH * sizeof(uint16_t));
      flirDataFrame_.minVal = minVal;
      flirDataFrame_.maxVal = maxVal;
    }


    /*!
     *  @brief Process the last obtained from flir lepton VoSPI frame.
     */
    void FlirLeptonHWIface::processFrame(void)
    {
      uint8_t imageVal;
      uint16_t temp;
      float temperVal;
      float temperSum = 0;
      uint16_t minVal, maxVal;

      /* ==================================================================== */
      memcpy(lastFrame_, flirDataFrame_.frameData,
        IMAGE_HEIGHT * IMAGE_WIDTH * sizeof(uint16_t));
      minVal = flirDataFrame_.minVal;
      maxVal = flirDataFrame_.maxVal;
      /* ==================================================================== */

      // Clear previous acquired frame temperature values
      temperMsg_.values.data.clear();
      thermalImage_.data.clear();

      for (int i = 0; i < IMAGE_WIDTH; i++) {
        for (int j = 0; j < IMAGE_HEIGHT; j++) {
          // Thermal image creation
          imageVal = Utils::signalToImageValue(
            lastFrame_[i * IMAGE_HEIGHT + j], minVal, maxVal);
          thermalImage_.data.push_back(imageVal);

          // Scene frame Temperatures vector creation.
          temperVal = Utils::signalToTemperature(
            lastFrame_[i * IMAGE_HEIGHT + j], calibMap_);
          temperMsg_.values.data.push_back(temperVal);
          temperSum += temperVal;
        }
      }

      thermalImage_.header.stamp = now_;
      temperMsg_.header.stamp = now_;

      frameAvgTemper_ = temperSum / (IMAGE_HEIGHT * IMAGE_WIDTH);
    }


    void FlirLeptonHWIface::fillBatchMsg(void)
    {
      batchMsg_.header.stamp = now_;
      batchMsg_.temperatures = temperMsg_;
      batchMsg_.thermalImage = thermalImage_;
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

  }  // namespace flir_lepton_sensor
}  // namespace flir_lepton_rpi2
