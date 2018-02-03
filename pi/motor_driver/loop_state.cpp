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
  output_file << "Motor count, pos radians, vel est, accel est, U of t, raw adc, amplified adc\n";

  for (unsigned int i = 0; i < history.size(); ++i) {
    LoopState now = history[i];
    output_file << now.motor_count << ", " << now.motor_pos_r << ", " << 
      now.vel_est_rs << ", " << now.accel_est_rss << ", " << now.u_of_t << ", " << 
      now.raw_adc << ", " << now.amplified_adc << "\n"; 
  }
  
  output_file.close();
}



