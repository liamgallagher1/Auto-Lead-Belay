#pragma once
// time_functions.hpp

#include <iostream>
#include <deque>
#include <time.h>

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

// size of least squares problem
#define LS_ORDER 3
#define LS_TIMES 5
#define CPR 8196


// Calculates the difference of two times
// TODO put the function elsewhere
// TODO use analyically derived times for loop timing,
// to avoid increasing loop times
inline void timespec_diff(
    struct timespec *start, 
    struct timespec *stop,
    struct timespec *result)
{
  if ((stop->tv_nsec - start->tv_nsec) < 0) {
    result->tv_sec = stop->tv_sec - start->tv_sec - 1;
    result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
  } else {
    result->tv_sec = stop->tv_sec - start->tv_sec;
    result->tv_nsec = stop->tv_nsec - start->tv_nsec;
  }
}

// Add a number of seconds and nano seconds to init time
// output it on out time.
inline void add_time(
    struct timespec* init_time,
    long time_sec,
    long time_nsec,
    struct timespec* out_time)
{
  out_time->tv_sec = init_time->tv_sec + time_sec;
  out_time->tv_nsec = init_time->tv_nsec + time_nsec;
  // Carry the zero
  out_time->tv_sec += out_time->tv_nsec / 1000000000;
  // make it a valid time in nano seconds
  out_time->tv_nsec %= 1000000000;
}


// Returns true if query time is greater than comparison time
// Should these be pointers if I am inlining it?
inline bool past_time(
    struct timespec* query_time,
    struct timespec* comparison_time)
{
  return (query_time->tv_sec > comparison_time->tv_sec) || 
        ((query_time->tv_sec == comparison_time->tv_sec) &&
         (query_time->tv_nsec >= comparison_time->tv_nsec));
}

typedef std::pair<long, struct timespec> TimeStamp;
typedef Eigen::Matrix<double, LS_TIMES, LS_ORDER> LSMat;
typedef Eigen::Matrix<double, LS_TIMES, 1> LSVec;
typedef Eigen::Matrix<double, LS_ORDER, 1> PolyVec;

void estimate_state_stamps(
    const std::deque<TimeStamp>& stamps,
    struct timespec& curr_time,
    double& vel_estimate_rs,
    double& accel_estimate_rss)
{
  // Scale the times to be between 0 and 1 for better numerics
  struct timespec oldest = stamps[LS_TIMES - 1].second;
  struct timespec newest = stamps[0].second;
  double time_diff_s = (newest.tv_sec - oldest.tv_sec) 
           + 10E-9 * (newest.tv_nsec - oldest.tv_nsec);
  long c0 = stamps[0].first;

  LSMat ls_prob; 
  LSVec counts;

  // Setup LS matrices
  struct timespec now;
  for (unsigned int i = 0; i < LS_TIMES; ++i) {
    now = stamps[i].second;
    double time = 
              ((now.tv_sec -  oldest.tv_sec) 
    + 10E-9 * (now.tv_nsec - oldest.tv_nsec)) / time_diff_s;
    double time_pow = 1;
    long count = stamps[i].first;
    counts(i) = count - c0;
    for (unsigned int j = 0; j < LS_ORDER; ++j) {
      ls_prob(i, j) = time_pow;
      time_pow *= time;
    }
  }

  // Solve for the bestfit polynomial
  PolyVec poly = 
    (ls_prob.transpose() * ls_prob).ldlt().solve(ls_prob.transpose() * counts);
  
  // calculate the derivative polynomials, offset, as well
  PolyVec first_der;
  PolyVec second_der;
  for (unsigned int i = 0; i < LS_ORDER; ++i) {
    first_der(i) = i * poly(i); 
    second_der(i) = (i - 1) * first_der(i);
  }

  // Calculate normalized estimates at the most current time
  double time = 
              ((curr_time.tv_sec -  oldest.tv_sec) 
    + 10E-9 * (curr_time.tv_nsec - oldest.tv_nsec)) / time_diff_s;
  double time_pow = 1.0;

  double vel_normalized = 0;
  double accel_normalized = 0;
  for (unsigned int i = 1; i < LS_ORDER - 1; ++i) {
    vel_normalized += first_der(i) * time_pow;
    accel_normalized += second_der(i + 1) * time_pow;
    time_pow *= time;
  }
  vel_normalized += first_der(LS_ORDER - 1) * time_pow;
  
  vel_estimate_rs = 2 * M_PI * vel_normalized / time_diff_s / CPR; 
  accel_estimate_rss = 2 * M_PI * accel_normalized / time_diff_s / time_diff_s / CPR;
}
