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
 *
 * *********************************************************************/

#ifndef FLIR_LEPTON_UTILS_H
#define FLIR_LEPTON_UTILS_H

#include <boost/utility.hpp>
#include <stdint.h>
#include <map>
#include <cstring>

#include "colormaps.h"


namespace flir_lepton
{
  namespace flir_lepton_sensor
  {
    class Utils : private boost::noncopyable
    {
      public:
        /*!
         * @brief Converts signal values to raw_image values.
         * @param signal signal value
         * @param minVal Minimun signal value captured on a thermal image frame.
         * @param maxVal Maximum signal value captured on a thermal image frame.
         * @return raw_image value.
         */
        static uint8_t signalToImageValue(
          uint16_t signal, uint16_t minVal, uint16_t maxVal);


        /*!
         * @brief Convert a signal value to absolute temperature value.
         * @param signalValue Signal value obtained from flir-lepton sensor.
         */
        static float signalToTemperature(uint16_t signalValue,
          std::map<uint16_t, float>& tempMap);


        /*!
         * @brief Fill an std::map that contains the calibration dataset
         */
        static std::map<uint16_t, float> loadThermalCalibMap(
          const std::string& calibFileUri);

        static void toColormap(uint8_t val, uint8_t& red, uint8_t& green,
          uint8_t& blue);

          
    };

  }  // namespace flir_lepton_sensor
}  // namespace flir_lepton

#endif


