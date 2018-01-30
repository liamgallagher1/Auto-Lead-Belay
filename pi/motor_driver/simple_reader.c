/*
gcc -Wall -pthread -o motor_driver driver.c -lpigpio
*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <pigpio.h>

#include "adc_reader.h"

#define SPI_PIN  22 // GPIO for slave select.
#define MISO_PIN 19 // Input 
#define MOSI_PIN 26 // Output
#define CLK_PIN  13 // Clock

// frequency the count is quieried
#define SAMPLING_FREQ_HZ 1000
#define PULSES_PER_REVOLUTION 2048
#define RUN_FOR_TIME_SEC 3
// defines frequency of printout
#define PRINT_FREQ_HZ 5


// Calculates the difference of two times
// TODO put the function elsewhere
void timespec_diff(
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



int main(int argc, char *argv[])
{
  // TODO can we run this twice?
  if (gpioInitialise() < 0) {
    printf("Failed GPIO INIT");
    return 1;
  }
  
  // janky until we actually do multiple inputs
  int only_miso_pin = MISO_PIN;
  ADC_Reader* reader = init_adc_reader(SPI_PIN, &only_miso_pin, 1, MOSI_PIN, CLK_PIN);


  double loop_wait_time_sec = (1.0 / SAMPLING_FREQ_HZ);
  long loop_wait_time_nsec = (long) round(loop_wait_time_sec * 1E9);
  long print_loop_freq_iters = (long) round(1.0 / (PRINT_FREQ_HZ * loop_wait_time_sec));

  printf("Sampling freq HZ and NSEC: %i \t %li \n", SAMPLING_FREQ_HZ, loop_wait_time_nsec);
  printf("Print freq HZ and num iters: %d \t %li \n", PRINT_FREQ_HZ, print_loop_freq_iters);

  // Chill for a second
  sleep(1);
  
  // Use timespec to wait for precise amounts of time
  // Time that we started the prgram after init,
  struct timespec start_time;
  // Time that we started this loop,
  struct timespec curr_loop_time;
  // Current time, if we are waiting for the loops period to expire
  struct timespec curr_time, time_diff;
  clock_gettime(CLOCK_REALTIME, &start_time);
  clock_gettime(CLOCK_REALTIME, &curr_loop_time);
 
  long num_iters = 0;

  int raw_reading;
  int amplified_reading;

  // Sampling loop, do for ten seconds
  // TODO better difference operator
  while(curr_loop_time.tv_sec - start_time.tv_sec < RUN_FOR_TIME_SEC) {
    num_iters++;

    // pull readings from the reader thing
    last_readings(reader, &raw_reading, &amplified_reading);

    printf("Raw: %d\t Amp: %d\n", raw_reading, amplified_reading);

    // Wait till the time for this loop expires
    do{
      clock_gettime(CLOCK_REALTIME, &curr_time);
      // Needs to do a slightly more difficult difference operation
      timespec_diff(&curr_loop_time, &curr_time, &time_diff);
    } while(time_diff.tv_nsec < loop_wait_time_nsec);
    curr_loop_time = curr_time;
  }





  free_adc_reader(reader);
  gpioTerminate();
  return 0;
}

