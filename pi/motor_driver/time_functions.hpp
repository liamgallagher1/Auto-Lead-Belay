#pragma once
// time_functions.hpp

#include <time.h>

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

