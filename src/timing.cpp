#include "timing.hpp"

void Timing::GetTime()
{
    timer_start_ = std::chrono::high_resolution_clock::now();
}
void Timing::MeasureTimeDifference()
{
    time_span_ = std::chrono::duration_cast<std::chrono::microseconds>(timer_start_ - last_start_time_);
    // timing_info_.insert(timing_info_.begin(), time_span.count());
    last_start_time_ = timer_start_;
    counter_++;
}
void Timing::OutInfoToFile()
{
    std::ofstream outFile("loop_timing_info.txt");
    for (const auto &e : timing_info_) outFile << e << "\n";
}