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
  // Loop info
  struct timespec loop_time;
  long loop_count;

  // ADC info
  int la_raw_adc;
  int la_amp_adc;
  int sm_raw_adc;
  int sm_amp_adc;
  int lm_raw_adc;
  int lm_amp_adc;

  double la_current;
  double sm_current;
  double lm_current;
  
  long lm_count;
  long sm_count;
 
  double sm_pos_r;
  double sm_vel_est_rs;


  double lm_pos_r;
  double lm_vel_est_rs;

  // Duty cycles in percents
  float la_duty_cycle;
  float sm_duty_cycle;
  float lm_duty_cycle;

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

