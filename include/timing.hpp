/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the Wrapped IgH EtherCAT master userspace program 
 * for control applications.
 *
 *  The Wrapped IgH EtherCAT master userspace program for control application
 *  in userspace is free software; you canredistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the 
 * Free Software Foundation; version 2 of the License.
 *
 *  The Wrapped IgH EtherCAT master userspace program for control application
 *  is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 *  PURPOSE.  
 *  See the  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the Wrapped IgH EtherCAT master userspace program for control application. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/*******************************************************************************
 * \file  timing.hpp
 * \brief Contains timing measurement functions for convenience
 *******************************************************************************/
#pragma once
#include <vector>
#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cstdint>
#include <ctime>
#include <ratio>
#include <fstream>
#include <string>
#include <iostream>

#define NUMBER_OF_SAMPLES 1E6
/**
 *  \class   Timing
 *  \brief   Contains Timing measurement related functions.
 */
class Timing{
    public:
      std::chrono::high_resolution_clock::time_point timer_start_;
      std::chrono::high_resolution_clock::time_point last_start_time_;
      std::chrono::duration<long,std::micro> time_span_;
      std::vector<long> timing_info_ = std::vector<long>(NUMBER_OF_SAMPLES);
      uint32_t counter_ = 0;
  /**
   * @brief Gets the current time and assings to timer_start_ member.
   */
  void GetTime();
  /**
   * @brief Measures time difference from last call to function GetTime() 
   * and writes is to time_span member
   * 
   */
  void MeasureTimeDifference();
  /**
   * @brief Outputs timing information to loop_timing_info.txt file.
   * 
   */
  void OutInfoToFile();
};