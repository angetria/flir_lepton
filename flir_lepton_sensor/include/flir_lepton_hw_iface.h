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
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
/* --------------------- */


namespace flir_lepton
{
  namespace flir_lepton_sensor
  {
    #define IMAGE_HEIGHT 60
    #define IMAGE_WIDTH 80
    #define MAX_RESTART_ATTEMPS_EXIT 5 
    #define MAX_RESETS_ERROR 750

 
    class FlirLeptonHWIface
    {
      private:
        /*!
         *  @brief Flir Lepton VoSPI struct.
         */
        struct FlirSpi
        {
          std::string devicePort;
          uint8_t mode;
          uint8_t bits;
          uint32_t speed;
          uint16_t delay;
          int handler;

          void configSpiParams(const ros::NodeHandle& nh);
        };


        /*!
         * @brief Flir Lepton data frame acquisition struct.
         */
        struct FlirDataFrame
        {
          uint16_t packetSize;
          uint16_t packetsPerFrame;
          uint16_t packetSize16;
          uint16_t frameSize16;

          uint8_t* frameBuffer;
          uint16_t* cleanDataBuffer;
          uint16_t* frameData;  // Clean flir data frame.
          //std::vector<uint16_t> dataV;
          uint16_t minVal;
          uint16_t maxVal;
          void allocateBuffers(void);
        };


        /* ------< Published Topics >------ */
        std::string grayTopic_;
        std::string rgbTopic_;
        std::string temperTopic_;
        std::string batchTopic_;
        /* -------------------------------- */

        /* -------< ROS Publishers >------- */
        ros::Publisher grayPublisher_;
        ros::Publisher rgbPublisher_;
        ros::Publisher temperPublisher_;
        ros::Publisher batchPublisher_;
        ros::Publisher raw_publisher_;
        /* -------------------------------- */
        ros::NodeHandle nh_;
        ros::Time now_;

        FlirSpi flirSpi_;  // SPI interface container
        float vospiFps_;
        FlirDataFrame flirDataFrame_;
        uint16_t* lastFrame_;

        /* -----< Thermal Image Characteristics >----- */
        std::string frameId_;
        /* ------------------------------------------- */

        /* ------< Scene Temperature Values >--------- */
        float frameAvgTemper_;
        /* ------------------------------------------- */

        /* -------------< Publishing Messages >-------------- */
        flir_lepton_msgs::TemperaturesMsg temperMsg_;
        flir_lepton_msgs::FlirLeptonBatchMsg batchMsg_;
        sensor_msgs::Image grayImage_;
        sensor_msgs::Image rgbImage_;
        /* -------------------------------------------------- */

        // Raw sensor signal values to absolute thermal values map
        std::map<uint16_t, float> calibMap_;
        std::string calibFileUri_;

        bool pubGray_;
        bool pubRgb_;

        // Threads
        boost::thread ioThread_;
        boost::mutex mtxLock_;


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


        void initThreadedIO(void);


        /*!
         * @brief Reads a frame from flir lepton thermal camera
         */
        void readFrame(void);


        /*!
         * @brief Exports thermal signal values from an obtained VoSPI frame
         */
        void processFrame(void);


        /*!
         *  @brief Allocate the uint16_t buffer memory
         */
        void allocateFrameData(void);


        void fillBatchMsg(void);


        float calcVoSPIfps(
          boost::posix_time::ptime& start, boost::posix_time::ptime& stop);


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
