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

#include "flir_lepton/utils.h"
#include <sstream>
#include <iostream>
#include <fstream>

namespace flir_lepton
{
  namespace flir_lepton_utils
  {

    uint8_t Utils::signalToImageValue(
      uint16_t signal, uint16_t minVal, uint16_t maxVal)
    {
      uint16_t imageValue = 0;
      uint16_t diff = 0;
      float scale = 0.0;
      diff = maxVal - minVal;
      scale = 255.0/diff;
      imageValue = (float)(signal-minVal) * scale;
      return (uint8_t)(imageValue);
    }


    float Utils::signalToTemperature(uint16_t signalValue,
      std::map<uint16_t, float>& tempMap)
    {
      // The input signalValue is the keyword of the map with temperatures
      std::map<uint16_t, float>::iterator search = tempMap.find(signalValue);
      float temp_celcius = 0.0;

      if(search != tempMap.end())
      {
        // Pass the value from the map
        temp_celcius = search->second;
      }
      return temp_celcius;
    }


    std::map<uint16_t, float> Utils::loadThermalCalibMap(
      const std::string& calibFileUri)
    {
      /* ---< Open a file input stream to read the dataset >--- */
      std::ifstream file(calibFileUri.c_str());
      std::string line;

      std::map<uint16_t, float> calibMap;

      // Value to be stored in map and its name in the file that we read
      float value = 0;
      uint16_t keyword = 0;
      std::string value_s;
      std::string keyword_s;
      std::istringstream ss;

      // If counter even -> read keyword of map from file
      // If counter odd -> read value of map from file
      int counter =2;

      if(file.is_open())
      {
        while(std::getline(file, line))
        {
          if(counter % 2 == 0)
          {
            // Read the keyword of map and convert it to uint16_t
            keyword_s = line;
            std::istringstream ss(keyword_s);
            ss >> keyword;

            if(ss.fail())
            {
              std::cout << "Failed to read thermal-signal dataset\n";
              exit(1);
            }
          }
          else
          {
            value_s = line;

            std::istringstream ss(value_s);
            // Convert string to float for map
            ss >> value;

            if(ss.fail())
            {
              std::cout << "Failed to read thermal-signal dataset\n";
              exit(1);
            }
            // Fill the map with the next pair
            calibMap[keyword] = value;
          }
          counter++;
        }
        file.close();
      }
      else
      {
        std::cout << "Failed to open file\n";
        exit(1);
      }
      return calibMap;
    }

  }
}
