#include <deque>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <pigpio.h>

// source is in C
extern "C"
{
#include "adc_reader.h"
#include "queue.h"
}
#include "circular_buffer.hpp"
#include "loop_state.hpp"
#include "rotary_encoder.hpp"
#include "time_functions.hpp"

using namespace std;

// TODO this should go
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))


// TODO make these static ints
#define DIR_PIN 8
#define PWM_PIN 25

#define ENCODER_A_PIN 23
#define ENCODER_B_PIN 24

// TODO fix pins
#define SPI_PIN  7 // GPIO for slave select.
#define MISO_PIN 9 // Input 
#define MOSI_PIN 10 // Output
#define CLK_PIN  11 // Clock

// duty cycle range used by pwm
#define PWM_RANGE 10000
// ideal frequency of PWM output in HZ
#define PWM_FREQ_HZ 4000
// frequency the count is quieried
#define SAMPLING_FREQ_HZ 400
#define PULSES_PER_REVOLUTION 2048
#define RUN_FOR_TIME_SEC 15 

#define EXPECTED_STAMPS 10E6

// defines frequency of printout
#define PRINT_FREQ_HZ 5
// Order of the estimator
#define VEL_ESTIMATOR_ORDER 12
#define ACCEL_ESTIMATOR_ORDER 12

#define STAMP_SIZE 1000

// Chrip input math
//static double CHIRP_AMP = 0.5;
//static double CHIRP_OFFSET = 0.5;
//static double INIT_FREQ_HZ = 1.0 / 20;
//static double FINAL_FREQ_HZ = 5;
//static int CHIRP_TIME_S = 100;
//// constant defining output rate of change
//// u = A sin(2 * pi * (init_freq * t + K/2 * t^2))
//static double CHIRP_K = (FINAL_FREQ_HZ - INIT_FREQ_HZ) / CHIRP_TIME_S;

static double RADS_PER_COUNT = 2.0 * M_PI / 4 / PULSES_PER_REVOLUTION;

// Quadrature Encoder gets 4 counts per pulse
//static int COUNTS_PER_REVOLUTION = 4 * PULSES_PER_REVOLUTION;


int main(int argc, char *argv[])
{
  if (gpioInitialise() < 0) {
    cout << "Failed GPIO Init" << endl;
    return 1;
  }
  
  

  // Encoder state and initalization
  Pi_Renc_t* renc;
  renc = Pi_Renc(ENCODER_A_PIN, ENCODER_B_PIN, STAMP_SIZE);
  // TODO unclear if this is still wise
  // Init staps deque before starting the encoder
  for (unsigned int i = 0; i < STAMP_SIZE; ++i) {
    renc->stamps_us->push_front(TimeStamp(0, 0));
  }
  
  //
  // ADC state and initalization
  // janky until we actually do multiple inputs
  int only_miso_pin = MISO_PIN;
  ADC_Reader* reader = init_adc_reader(SPI_PIN, &only_miso_pin, 1, MOSI_PIN, CLK_PIN);
  // Configure motor pins to output  
  gpioSetMode(DIR_PIN,  PI_OUTPUT);
  gpioSetMode(PWM_PIN, PI_OUTPUT);
 
  // Turn on forward direction
  gpioWrite(DIR_PIN, 0);

  // The asked for frequency isn't necessarily the set one
  // because of sample rate considerations
  int real_freq_hz = gpioSetPWMfrequency(PWM_PIN, PWM_FREQ_HZ);
  cout << "Real PWM Frequency: " << real_freq_hz << endl;

  // Set range controlling fully on vs fully off behavior
  int real_range = gpioSetPWMrange(PWM_PIN, PWM_RANGE);
  cout << "Real pwm range: " <<  real_range << endl;

  // Set PWM output to 50% duty cycle
  //unsigned int half_on = (unsigned int) round(PWM_RANGE * CHIRP_OFFSET);
  //int set_pwm = gpioPWM(PWM_PIN, half_on);
  //cout << "Set pwm?: " <<  set_pwm << endl;
  int set_pwm;

  double loop_wait_time_sec = (1.0 / SAMPLING_FREQ_HZ);
  long loop_wait_time_nsec = (long) round(loop_wait_time_sec * 1E9);
  long print_loop_freq_iters = (long) round(1.0 / (PRINT_FREQ_HZ * loop_wait_time_sec));

  cout << "Sampling freq HZ and NSEC: " <<  SAMPLING_FREQ_HZ << "\t"  << loop_wait_time_nsec << endl;
  cout << "Print freq HZ and num iters: " << PRINT_FREQ_HZ << "\t" << print_loop_freq_iters << endl;

  // Chill for a second ?
  //sleep(1);
  
  long num_iters = 0;

  double motor_pos_rad = renc->main_motor_count * RADS_PER_COUNT;
  int raw_adc_reading;
  int amplified_adc_reading;
  
  // What percentage of iterations have extra time to spare?
  // Want it to be 99.9999 or so
  double num_iters_with_time = 0;
  // How much time has been spend doing the work in the loop, 
  // not waiting
  long long total_inner_loop_time_ns = 0;
  // And what is the slowest loop time
  long max_inner_loop_time_ns = 0;

  // history
  vector<LoopState> history;
  history.reserve((RUN_FOR_TIME_SEC + 1) * SAMPLING_FREQ_HZ); 
  vector<TimeStamp> all_stamps;
  all_stamps.reserve(EXPECTED_STAMPS);
  long num_stamps = 0;
  long prev_num_stamps = 0;

  // Use timespec to wait for precise amounts of time
  // Time that we started the prgram after init,
  struct timespec start_time;
  // Time to finish this jawn
  struct timespec finish_time;
  // Time that we started this loop,
  struct timespec curr_loop_time;
  // Current time, if we are waiting for the loops period to expire
  struct timespec curr_time, time_diff;
  // Time to start next loop. Always add to it the proper number of nanoseconds.
  struct timespec next_loop_time;  
 
  clock_gettime(CLOCK_REALTIME, &curr_time);
  clock_gettime(CLOCK_REALTIME, &start_time);
  add_time(&start_time, RUN_FOR_TIME_SEC, 0, &finish_time);
  clock_gettime(CLOCK_REALTIME, &curr_loop_time);
  add_time(&curr_loop_time, 0, loop_wait_time_nsec, &next_loop_time); 
  
  cout << "Going into inner loop " << endl;
  sleep(1);

  while(!past_time(&curr_loop_time, &finish_time) ) {
    // Get the current motor position in radians
    long prev_count = renc->main_motor_count;
    motor_pos_rad = renc->main_motor_count * RADS_PER_COUNT;
   
    // TODO occasional numerical conditioning considerations by keeping 
    // motor position small

    // close queue lock
    // estimate 
    // estimate_state_stamps(stamps, curr_loop_time, ls_vel_est_rs, ls_accel_est_rss);
    // open queue lock

    // Get ADC readings
    last_readings(reader, &raw_adc_reading, &amplified_adc_reading);

    // Calculate and execute output.
    //double time_s = (num_iters % (SAMPLING_FREQ_HZ * CHIRP_TIME_S)) / loop_wait_time_sec; 
    //// calculate chirp output
    //float u_of_t = CHIRP_OFFSET + CHIRP_AMP * sin(2 * M_PI * 
    //   (INIT_FREQ_HZ * time_s + CHIRP_K * time_s * time_s));  
    //// CLAMP
    //u_of_t = fmin(u_of_t, 1.0);
    //u_of_t = fmax(u_of_t, 0.0);
    //unsigned int pwm_in = (unsigned int) round(u_of_t * PWM_RANGE);
    //gpioPWM(PWM_PIN, pwm_in);
 
    // Record the current state
    LoopState state;
    state.loop_time = curr_time;
    state.prev_loop_comp_time = time_diff;
    state.motor_count = prev_count;
    state.motor_pos_r = motor_pos_rad;
//    state.vel_est_rs = vel_est_rs;
//    state.accel_est_rss = accel_est_rss;
    //state.u_of_t = pwm_in;
    state.raw_adc = raw_adc_reading;
    state.amplified_adc = amplified_adc_reading;
    history.push_back(state);

    // TODO check sanity
    // Add time stamps to stamp history
    num_stamps = renc->stamps_us->add_count(); 
    long num_stamps_to_add = num_stamps - prev_num_stamps;   
    // TODO make safe to more stamps than queue size?
    for (int i = MIN(num_stamps_to_add, STAMP_SIZE) - 1; i >= 0; --i) {
      all_stamps.push_back((*renc->stamps_us)[i]); 
    }
    prev_num_stamps = num_stamps;

    num_iters++;
    // Print if its time to do so
    // TODO clock the operation?
    if ((num_iters % print_loop_freq_iters) == 0) {
      cout << "Motor pos: " << motor_pos_rad << endl; 
      //cout << "Est: " << ls_vel_est_rs << ", " << ls_accel_est_rss << endl;
    }    

    // Loop timing code
    // TODO cut some of this crud
    clock_gettime(CLOCK_REALTIME, &curr_time);
    timespec_diff(&curr_loop_time, &curr_time, &time_diff);
    total_inner_loop_time_ns += time_diff.tv_nsec;
    max_inner_loop_time_ns = MAX(time_diff.tv_nsec, max_inner_loop_time_ns);

    // Wait till the time for this loop expires
    unsigned int did_n_times = 0;
    do{
      clock_gettime(CLOCK_REALTIME, &curr_time);
      did_n_times++;
    } while(!past_time(&curr_time, &next_loop_time));
   
    // Check if that we had time to spare
    if (did_n_times > 1) {
      num_iters_with_time++; 
    }
    
    add_time(&next_loop_time, 0, loop_wait_time_nsec, &next_loop_time);
    curr_loop_time = curr_time;
  }


  timespec_diff(&start_time, &curr_time, &time_diff);
  printf("Time: %li\n Expected time: %f\n", time_diff.tv_sec, num_iters * loop_wait_time_sec); 

  printf("maximum inner loop time, min freq: %li \t %f\n", max_inner_loop_time_ns, 1000000000 / (double) max_inner_loop_time_ns);
  total_inner_loop_time_ns /= num_iters;

  printf("Average inner loop time, freq: %li \t %f\n",(long) total_inner_loop_time_ns,  1000000000 / (double) total_inner_loop_time_ns); 
  printf("Iters with time: %f\n", num_iters_with_time / (double) num_iters);

  // Turn off PWM?
  set_pwm = gpioWrite(PWM_PIN, 0);
  printf("PWM OFF? %d\n", set_pwm);
  set_pwm = gpioWrite(DIR_PIN, 0);
  printf("DIR OFF? %d\n", set_pwm);
  // Don't forget to kill the encoder count process before exiting
  Pi_Renc_cancel(renc);
  free_adc_reader(reader);
  gpioTerminate();
  cout << "Writing to file? " << (argc == 2 || argc == 3) << endl;
  if (argc == 2) {
    write_loops_to_file(history, string("chirp_sys_id"));
  } else if (argc == 3) {
    write_loops_stamps_to_file(history, all_stamps, string("timestamp_test"));
  }

  return 0;
}

