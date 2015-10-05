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
 *********************************************************************/

#ifndef FLIR_LEPTON_IMAGE_PROCESSING_UTILS_TIMER_H
#define FLIR_LEPTON_IMAGE_PROCESSING_UTILS_TIMER_H

#ifndef DEFINES
#define DEFINES
#define PKG_NAME "flir_lepton_image_processing"
#endif  // DEFINES

#include <iostream>
#include <cstdlib>
#include <string>
#include <map>
#include <set>
#include <fstream>
#include <ros/ros.h>
#include <sys/time.h>

class Timer
{
  typedef std::map<std::string, double> Msd;
  typedef std::map<float, std::string> Mfs;
  typedef std::map<float, std::string>::iterator MfsIt;
  typedef std::map<std::string, uint64_t> Msul;
  typedef std::map<std::string, double>::iterator MsdIt;
  typedef std::map<std::string, double>::const_iterator MsdCIt;
  typedef std::pair<std::string, double> Psd;
  typedef std::pair<std::string, uint64_t> Psul;

  static Msd times;
  static Msd max_time;
  static Msd min_time;
  static Msd mean_time;
  static Msd sum_time;
  static Msul count;
  static struct timeval msTime;
  static std::map<std::string, std::set<std::string> > timer_tree;

  static std::string top_node;

  Timer(void){}

  static void printMsInternal(double t);
  static void printSecInternal(double t);
  static void printMinInternal(double t);
  static void printHoursInternal(double t);
  static void printLiteralInternal(double t);

  public:
  static void start(std::string timerId, std::string father = "", bool top = false);
  static double stop(std::string timerId);
  static double mean(std::string timerId);
  static void tick(std::string timerId);
  static void printMs(std::string timerId);
  static void printSec(std::string timerId);
  static void printMin(std::string timerId);
  static void printHours(std::string timerId);
  static void printLiteral(std::string timerId);
  static void printLiteralMean(std::string timerId, std::string identation = "");
  static void printAll(void);
  static void printAllMeans(void);
  static void printAllMeansTree(void);
  static void printIterativeTree(std::string node, std::string identation);
};

#endif  // FLIR_LEPTON_IMAGE_PROCESSING_UTILS_TIMER_H
