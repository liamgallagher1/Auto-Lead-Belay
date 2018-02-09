#pragma once

#include <string>
#include <time.h>
#include <vector>

typedef std::pair<long, struct timespec> TimeStamp;

//#include <eigen3/Eigen/Dense>

// Information to save on every loop

//typedef struct LSState {
//  double vel_est_rs;
//  double accel_est_rs;
//  double 
//
//}

typedef struct LoopState {
  struct timespec loop_time;
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

// Outputs the history to a CSV file
void write_loops_stamps_to_file(
    const std::vector<LoopState>& history, 
    const std::vector<TimeStamp>& all_stamps,
    const std::string& file_name);

