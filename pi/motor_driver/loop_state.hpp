#pragma once

#include <string>
#include <vector>


// Information to save on every loop
typedef struct LoopState {
  long motor_count;
  double motor_pos_r;
  double vel_est_rs;
  double accel_est_rss;

  unsigned int u_of_t;
  unsigned int raw_adc;
  unsigned int amplified_adc;
} LoopState;


// Outputs the history to a CSV file
void write_loops_to_file(
    const std::vector<LoopState>& history, 
    const std::string& file_name);
