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

#include "utils/timer.h"

std::map<std::string, double> Timer::times = std::map<std::string, double>();
std::map<std::string, double> Timer::max_time = std::map<std::string, double>();
std::map<std::string, double> Timer::min_time = std::map<std::string, double>();
std::map<std::string, double> Timer::mean_time = std::map<std::string, double>();
std::map<std::string, double> Timer::sum_time = std::map<std::string, double>();

std::map<std::string, uint64_t> Timer::count =
std::map<std::string, uint64_t>();

std::map<std::string, std::set<std::string> > Timer::timer_tree =
std::map<std::string, std::set<std::string> >();

std::string Timer::top_node = "";

struct timeval Timer::msTime = timeval();

void Timer::printMsInternal(double t)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, t << " ms");
}

void Timer::printSecInternal(double t)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, t / 1000.0 << " sec");
}

void Timer::printMinInternal(double t)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, t / 1000.0 / 60.0 << " minutes");
}

void Timer::printHoursInternal(double t)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, t / 1000.0 / 60.0 / 60.0 << " hours");
}

void Timer::printLiteralInternal(double t)
{
  int sec = t / 1000.0;
  if (sec >= 1)
  {
    t -= sec * 1000;
    int min = sec / 60.0;
    if (min >= 1)
    {
      sec -= min * 60;
      int hours = min / 60.0;
      if (hours >= 1)
      {
        min -= hours * 60;
        ROS_INFO_STREAM_NAMED(PKG_NAME,
          min << " hours " <<hours << " minutes "
          << sec << " sec " << t << " ms");
      }
      else
      {  // Mins
        ROS_INFO_STREAM_NAMED(PKG_NAME,
          min << " minutes " << sec << " sec " << t << " ms\n");
      }
    }
    else
    {  // Sec
      ROS_INFO_STREAM_NAMED(PKG_NAME, sec << " sec " << t << " ms");
    }
  }
  else
  {  // Ms
    ROS_INFO_STREAM_NAMED(PKG_NAME, t << " ms");
  }
}

void Timer::start(std::string timerId, std::string father, bool top)
{
  MsdIt it = times.find(timerId);
  if (it == times.end())
  {
    timer_tree.insert(std::pair<std::string, std::set<std::string> >
      (timerId, std::set<std::string>()));
    gettimeofday(&msTime, NULL);
    double ms = static_cast<double>(msTime.tv_sec) * 1000 +
      static_cast<double>(msTime.tv_usec) / 1000;
    times.insert(Psd(timerId, ms));
    count.insert(Psul(timerId, 0));
    mean_time.insert(Psul(timerId, 0));
    sum_time.insert(Psul(timerId, 0));
    max_time.insert(Psd(timerId, -1.0));
    min_time.insert(Psd(timerId, 1000000000000000.0));
  }
  else
  {
    gettimeofday(&msTime, NULL);
    double ms = static_cast<double>(msTime.tv_sec) * 1000 +
      static_cast<double>(msTime.tv_usec) / 1000;
    it->second = ms;
    mean_time.insert(Psul(timerId, 0));
    sum_time.insert(Psul(timerId, 0));
    max_time.insert(Psd(timerId, -1.0));
    min_time.insert(Psd(timerId, 1000000000000000.0));
  }
  if (father != "")
  {
    timer_tree[father].insert(timerId);
  }
  if (top)
  {
    top_node = timerId;
  }
}

double Timer::stop(std::string timerId)
{
  MsdIt it = times.find(timerId);
  if (it == times.end())
  {
    ROS_INFO_STREAM_NAMED(PKG_NAME, "Invalid timer id : " << timerId << "\n");
    return -1;
  }
  else
  {
    gettimeofday(&msTime , NULL);
    double ms = static_cast<double>(msTime.tv_sec) * 1000 +
      static_cast<double>(msTime.tv_usec) / 1000;
    return ms - it->second;
  }
}

double Timer::mean(std::string timerId)
{
  MsdIt it = times.find(timerId);
  if (it == times.end())
  {
    ROS_INFO_STREAM_NAMED(PKG_NAME, "Invalid timer id : " << timerId << "\n");
    return -1;
  }
  else
  {
    return mean_time[timerId];
  }
}

void Timer::tick(std::string timerId)
{
  MsdIt it = times.find(timerId);
  if (it == times.end())
  {
    ROS_INFO_STREAM_NAMED(PKG_NAME, "Invalid timer id : " << timerId << "\n");
  }
  else
  {
    count[timerId]++;
    gettimeofday(&msTime , NULL);
    double ms = static_cast<double>(msTime.tv_sec) * 1000 +
      static_cast<double>(msTime.tv_usec) / 1000;
    ms -= it->second;
    times[timerId] = ms;
    sum_time[timerId] += ms;
    mean_time[timerId] = sum_time[timerId] / count[timerId];
    if (min_time[timerId] > ms)
      min_time[timerId] = ms;
    if (max_time[timerId] < ms)
      max_time[timerId] = ms;
  }
}

void Timer::printMs(std::string timerId)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, "Timer " << timerId << " : ");
  printMsInternal(stop(timerId));
}

void Timer::printSec(std::string timerId)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, "Timer " << timerId << " : ");
  printSecInternal(stop(timerId));
}

void Timer::printMin(std::string timerId)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, "Timer " << timerId << " : ");
  printMinInternal(stop(timerId));
}

void Timer::printHours(std::string timerId)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, "Timer " << timerId << " : ");
  printHoursInternal(stop(timerId));
}

void Timer::printLiteral(std::string timerId)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, "Timer " << timerId << " : ");
  printLiteralInternal(stop(timerId));
}

void Timer::printLiteralMean(std::string timerId, std::string identation)
{
  MsdIt it = times.find(timerId);
  if (it == times.end())
  {
    ROS_INFO_STREAM_NAMED(PKG_NAME, "Invalid timer id : " << timerId << "\n");
  }
  else
  {
    ROS_INFO_STREAM_NAMED(PKG_NAME, identation << timerId <<
      "' [" << times[timerId] << " - " <<
      min_time[timerId] << " , " << mean_time[timerId] << " , " <<
      max_time[timerId] << " - " << count[timerId] << "]");
  }
}

void Timer::printAll(void)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, "Timers available :");
  for (MsdCIt it = times.begin() ; it != times.end() ; it++)
  {
    ROS_INFO_STREAM_NAMED(PKG_NAME, "\t");
    printLiteral(it->first);
  }
}
void Timer::printAllMeans(void)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME, "Timers available :");
  Mfs tms;
  for (MsdCIt it = mean_time.begin() ; it != mean_time.end() ; it++)
  {
    tms.insert(std::pair<float, std::string>(it->second, it->first));
  }
  for (MfsIt it = tms.begin() ; it != tms.end() ; it++)
  {
    ROS_INFO_STREAM_NAMED(PKG_NAME, "\t"; printLiteralMean(it->second));
  }
}

void Timer::printAllMeansTree(void)
{
  ROS_INFO_STREAM_NAMED(PKG_NAME,
    "Timers available : [curr - min , mean , max - ticks]\n");
  Mfs tms;
  for (MsdCIt it = mean_time.begin() ; it != mean_time.end() ; it++)
  {
    tms.insert(std::pair<float, std::string>(it->second, it->first));
  }
  printIterativeTree(top_node, "");
}

void Timer::printIterativeTree(std::string node, std::string identation)
{
  printLiteralMean(node, identation);
  for (std::set<std::string>::iterator it = timer_tree[node].begin() ; it !=
    timer_tree[node].end() ; it++)
  {
    printIterativeTree(*it, identation + "|-");
  }
}
