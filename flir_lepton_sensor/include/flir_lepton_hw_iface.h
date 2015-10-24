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
 * * Author: Konstantinos Panayiotou, Agnelos Triantafyllidis,
 * *  Tsirigotis Christos
 * * Maintainer: Konstantinos Panayiotou
 * * Email: klpanagi@gmail.com
 * *
 * *********************************************************************/

#ifndef FLIR_LEPTON_FLIR_LEPTON_HW_IFACE_H
#define FLIR_LEPTON_FLIR_LEPTON_HW_IFACE_H

/* ---< Containers >--- */
#include <vector>
#include <map>
#include <cstring>
/* -------------------- */

#include <stdlib.h>
#include <stdint.h>


/* ---< ROS related >--- */
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "flir_lepton_msgs/TemperaturesMsg.h"
#include "flir_lepton_msgs/FlirLeptonBatchMsg.h"
#include "flir_lepton_msgs/FlirLeptonRawMsg.h"
/* --------------------- */


namespace flir_lepton
{
  namespace flir_lepton_sensor
  {
    class FlirLeptonHWIface
    {
      private:
        struct FlirSpi
        {
          std::string devicePort;
          uint8_t mode;
          uint8_t bits;
          uint32_t speed;
          uint16_t delay;
          uint16_t packet_size;
          uint16_t packets_per_frame;
          uint16_t packet_size_uint16;
          uint16_t frame_size_uint16;
          int handler;

          void configFlirSpi(const ros::NodeHandle& nh);
          uint8_t* makeFrameBuffer(void);
        };

        /* ------< Published Topics >------ */
        std::string imageTopic_;
        std::string temperTopic_;
        std::string batchTopic_;
        /* -------------------------------- */

        /* -------< ROS Publishers >------- */
        ros::Publisher imagePublisher_;
        ros::Publisher temperPublisher_;
        ros::Publisher batchPublisher_;
        ros::Publisher raw_publisher_;
        /* -------------------------------- */
        ros::NodeHandle nh_;
        ros::Time now_;

        int statusValue_;
        FlirSpi flirSpi_;  // SPI interface container

        uint8_t* rawBuffer_;
        std::vector<uint16_t> frameData_;  // Raw signal values

        /* -----< Thermal Image Characteristics >----- */
        std::string frame_id_;
        std::string image_encoding_;
        uint16_t imageHeight_;
        uint16_t imageWidth_;
        /* ------------------------------------------- */

        /* ------< Scene Temperature Values >--------- */
        std::vector<float> frameTempers_;
        float frameAvgTemper_;
        /* ------------------------------------------- */

        // Raw sensor signal values to absolute thermal values map
        std::map<uint16_t, float> calibMap_;
        std::string calibFileUri_;

        int MAX_RESTART_ATTEMPS_EXIT;
        int MAX_RESETS_ERROR;


        /*!
         * @brief Loads parameters from parameter server
         */
        void loadParameters(void);

        /*!
         * @brief Opens SPI device port for communication with flir lepton camera
         */
        void openDevice(void);


        /*!
         * @brief Closes the SPI device communication port
         */
        void closeDevice(void);


        /*!
         * @brief Reads a frame from flir lepton thermal camera
         */
        void readFrame(uint8_t** rawBuffer);


        /*!
         * @brief Exports thermal signal values from an obtained VoSPI frame
         */
        void processFrame(uint16_t& minValue, uint16_t& maxValue);



        void craftTemperMsg(flir_lepton_msgs::TemperaturesMsg& temperMsg);


        /*!
         * @brief Fills Thermal image ros message
         */
        void craftImageMsg(
          sensor_msgs::Image& thermalImage, uint16_t minValue,
          uint16_t maxValue);


        void craftBatchMsg(
          flir_lepton_msgs::FlirLeptonBatchMsg& batchMsg,
          flir_lepton_msgs::TemperaturesMsg& temperMsg,
          sensor_msgs::Image& thermalImage);

      public:

        /*!
         * @brief Default constructor
         */
        FlirLeptonHWIface(const std::string& ns);


        /*!
         * @brief Default Destructor
         */
        virtual ~FlirLeptonHWIface();


        /*!
         * @brief Reads a thermal scene frame and publishes the image
         *  on the relevant topic
         */
        void run(void);

    };

  }  // namespace flir_lepton_sensor
}  // namespace flir_lepton

#endif  // FLIR_LEPTON_FLIR_LEPTON_H
