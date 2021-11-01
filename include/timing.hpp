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

class Timing{
    public:
      std::chrono::high_resolution_clock::time_point timer_start_;
      std::chrono::high_resolution_clock::time_point last_start_time_;
      std::chrono::duration<long,std::micro> time_span;
      std::vector<long> timing_info_ = std::vector<long>(NUMBER_OF_SAMPLES);
      uint32_t counter_ = 0;
  void GetTime();
  void MeasureTimeDifference();
  void OutInfoToFile();
};