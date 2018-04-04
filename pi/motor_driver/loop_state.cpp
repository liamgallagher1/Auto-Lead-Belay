#include <fstream>
#include <math.h>
#include <string>
#include <time.h>

#include "loop_state.hpp"

using namespace std;

// Hours from UTC
static int EST = -5;

void write_loops_to_file(
    const std::vector<LoopState>& history, 
    const std::string& file_name)
{
  time_t raw_time;
  time(&raw_time);
  struct tm * est_time;
  est_time = gmtime(&raw_time);
  int day = est_time->tm_yday;
  int hour = (abs(est_time->tm_hour + EST)) % 24;
  int min = est_time->tm_min;

  string full_file_name = string("logs/") + file_name + to_string(day) + string("_") + 
    to_string(hour) + string("_") + to_string(min) + string(".csv");

  ofstream output_file(full_file_name); //, std::ofstream::out);
  output_file << "Time s, time ns, loop count, la_raw_adc, la_amp_adc, " << 
    "sm_raw_adc, sm_amp_adc, lm_raw_adc, lm_amp_adc, la_current, sm_current, " <<
    "lm_current, sm_count, lm_count, sm_pos_r, sm_vel_est_rs, lm_pos_r, " <<
    "lm_vel_est_rs, la_duty_cycle, sm_duty_cycle, lm_duty_cycle\n";

  for (unsigned int i = 0; i < history.size(); ++i) {
    LoopState now = history[i];
    output_file << now.loop_time.tv_sec << ", " << now.loop_time.tv_nsec << ", " << 
      now.loop_count << ", " << now.la_raw_adc << ", " << 
      now.la_amp_adc << ", " << now.sm_raw_adc << ", " << now.sm_amp_adc << ", " << 
      now.lm_raw_adc << ", " << now.lm_amp_adc << ", " <<  
      now.la_current << ", " << now.sm_current << ", " << 
      now.lm_current << ", " << now.sm_count << ", " << now.lm_count << ", " << 
      now.sm_pos_r << ", " << now.sm_vel_est_rs << ", " << now.la_duty_cycle << ", " << 
      now.sm_duty_cycle << ", " << now.lm_duty_cycle << "\n";
  }
  
  output_file.close();
}


//void write_loops_stamps_to_file(
//    const std::vector<LoopState>& history, 
//    const std::vector<TimeStamp>& all_stamps,
//    const std::string& file_name)
//{
//  time_t raw_time;
//  time(&raw_time);
//  struct tm * est_time;
//  est_time = gmtime(&raw_time);
//  int day = est_time->tm_yday;
//  int hour = (abs(est_time->tm_hour + EST)) % 24;
//  int min = est_time->tm_min;
//
//  string full_file_name = string("logs/") + file_name + to_string(day) + string("_") + 
//    to_string(hour) + string("_") + to_string(min) + string(".csv");
//
//  ofstream output_file(full_file_name); //, std::ofstream::out);
//  
//  output_file << "Stamp time s, Stamp time ns, motor count\nTime s, time ns, Motor count, pos radians, vel est, accel est, U of t, raw adc, amplified adc\n";
// 
//  // output states
//  for (unsigned int i = 0; i < all_stamps.size(); ++i) {
//    output_file << all_stamps[i].second.tv_sec << ", ";
//  }
//  output_file << "\n";
//  for (unsigned int i = 0; i < all_stamps.size(); ++i) {
//    output_file << all_stamps[i].second.tv_nsec << ", ";
//  }
//  output_file << "\n";
//  for (unsigned int i = 0; i < all_stamps.size(); ++i) {
//    output_file << all_stamps[i].first << ", ";
//  }
//  output_file << "\n";
//  // Then output states
//
//  for (unsigned int i = 0; i < history.size(); ++i) {
//    LoopState now = history[i];
//    output_file << now.loop_time.tv_sec << ", " << now.loop_time.tv_nsec << ", " << 
//      now.motor_count << ", " << now.motor_pos_r << ", " << 
//      now.vel_est_rs << ", " << now.accel_est_rss << ", " << now.u_of_t << ", " << 
//      now.raw_adc << ", " << now.amplified_adc << "\n"; 
//  }
//  
//  output_file.close();
//}
