#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <pigpio.h>

// All of my defined source is in C
extern "C"
{
#include "adc_reader.h"
#include "rotary_encoder.h"
#include "queue.h"
}
#include "loop_state.hpp"
#include "time_functions.hpp"

using namespace std;

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
#define SAMPLING_FREQ_HZ 4000
#define PULSES_PER_REVOLUTION 2048
#define RUN_FOR_TIME_SEC 10
// defines frequency of printout
#define PRINT_FREQ_HZ 10
// Order of the estimator
#define VEL_ESTIMATOR_ORDER 12
#define ACCEL_ESTIMATOR_ORDER 10

// Chrip input math
static double CHIRP_AMP = 0.5;
static double CHIRP_OFFSET = 0.5;
static double INIT_FREQ_HZ = 1.0 / 20;
static double FINAL_FREQ_HZ = 5;
static int CHIRP_TIME_S = 100;
// constant defining output rate of change
// u = A sin(2 * pi * (init_freq * t + K/2 * t^2))
static double CHIRP_K = (FINAL_FREQ_HZ - INIT_FREQ_HZ) / CHIRP_TIME_S;

// TODO this should go
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

// Quadrature Encoder gets 4 counts per pulse
static int COUNTS_PER_REVOLUTION = 4 * PULSES_PER_REVOLUTION;

// Velocity filter parameters, numerator and denomenator terms of IIR filter
// implicit a_1 = 1.0 filter term included unecessarily 
static double vel_estimator_b[VEL_ESTIMATOR_ORDER] = {
  41.4895856544645, -370.492408422197, 1473.26031862357, -3424.09533031117, 5126.02498600793, -5126.02498600793, 
  3424.09533031117, -1473.26031862357, 370.492408422196, -41.4895856544644
};

static double vel_estimator_a[VEL_ESTIMATOR_ORDER] = { 
  1, -7.62789647849086, 25.3113840106525, -47.5666759564674, 55.0255254778286, -39.5730525988382, 
  16.6342418997334, -3.17748822853527, -0.139626741142248, 0.113588615291234
};

static double accel_estimator_b[ACCEL_ESTIMATOR_ORDER] = {
  9.3759, -75.3733, 276.0036, -605.9827,  881.1613, -881.1613,  605.9827, -276.0036, 75.3733,  -9.3759
};

static double accel_estimator_a[ACCEL_ESTIMATOR_ORDER] = {
    1.0000, -6.9395, 20.9819, -35.9672, 37.9638, -24.8748, 9.4597, -1.5716, -0.1093, 0.0570
};

static double RADS_PER_COUNT = 2.0 * M_PI / 4 / PULSES_PER_REVOLUTION;
// Motor count modified by ISR
volatile long main_motor_count = 0; // tics


// Now only use the callback to modify the main motor count
void callback(int dir)
{
  if (dir == 1) {
    main_motor_count++;
  } else {
    main_motor_count--;
  }
}


int main(int argc, char *argv[])
{
  if (gpioInitialise() < 0) {
    cout << "Failed GPIO Init" << endl;
    return 1;
  }
  // Encoder state and initalization
  Pi_Renc_t* renc;
  renc = Pi_Renc(ENCODER_A_PIN, ENCODER_B_PIN, callback);
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
  unsigned int half_on = (unsigned int) round(PWM_RANGE * CHIRP_OFFSET);
  int set_pwm = gpioPWM(PWM_PIN, half_on);
  cout << "Set pwm?: " <<  set_pwm << endl;

  double loop_wait_time_sec = (1.0 / SAMPLING_FREQ_HZ);
  long loop_wait_time_nsec = (long) round(loop_wait_time_sec * 1E9);
  long print_loop_freq_iters = (long) round(1.0 / (PRINT_FREQ_HZ * loop_wait_time_sec));

  cout << "Sampling freq HZ and NSEC: " <<  SAMPLING_FREQ_HZ << "\t"  << loop_wait_time_nsec << endl;
  cout << "Print freq HZ and num iters: " << PRINT_FREQ_HZ << "\t" << print_loop_freq_iters << endl;

  // Chill for a second ?
  //sleep(1);
  
   
  long num_iters = 0;

  int queue_size = MAX(VEL_ESTIMATOR_ORDER, ACCEL_ESTIMATOR_ORDER) + 1;
  // Initalize circular buffer of previous encoding measurements estimates
  Queue* prev_accel_ests_rss = createQueue(queue_size + 1);
  Queue* prev_vel_ests_rs =    createQueue(queue_size + 1);
  Queue* prev_pos_obs_rad =    createQueue(queue_size + 1);
  
  double motor_pos_rad = main_motor_count * RADS_PER_COUNT;
  int raw_adc_reading;
  int amplified_adc_reading;
  
  for (unsigned int i = 0; i < VEL_ESTIMATOR_ORDER; ++i) {
    enqueue(prev_vel_ests_rs, 0.0);
    enqueue(prev_pos_obs_rad, motor_pos_rad);
    enqueue(prev_accel_ests_rss, 0.0);
  }

  // What percentage of iterations have extra time to spare?
  // Want it to be 99.9999 or so
  double num_iters_with_time = 0;
  // How much time has been spend doing the work in the loop, 
  // not waiting
  long long total_inner_loop_time_ns = 0;
  // And wants the slowest loop time
  long max_inner_loop_time_ns = 0;

  // history
  vector<LoopState> history;
  history.reserve((RUN_FOR_TIME_SEC + 1) * SAMPLING_FREQ_HZ); 


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
  
  clock_gettime(CLOCK_REALTIME, &start_time);
  add_time(&start_time, RUN_FOR_TIME_SEC, 0, &finish_time);
  clock_gettime(CLOCK_REALTIME, &curr_loop_time);
  add_time(&curr_loop_time, 0, loop_wait_time_nsec, &next_loop_time); 
  
  cout << "Going into inner loop " << endl;
  sleep(1);

  // Sampling loop, do for ten seconds
  // TODO better difference operator
  while(!past_time(&curr_loop_time, &finish_time)) {
    // Get the current motor position in radians
    long prev_count = main_motor_count;
    motor_pos_rad = main_motor_count * RADS_PER_COUNT;
   
    // TODO occasional numerical conditioning considerations by keeping 
    // motor position small
    
   // Do IIR filter calculatorions
    double vel_est_rs = vel_estimator_b[0] * motor_pos_rad;
    for (unsigned int i = 1; i < VEL_ESTIMATOR_ORDER; ++i) {
      // double check the sanity of this
      // Handle numerator, or previous position obseration terms
      vel_est_rs += vel_estimator_b[i] * at(prev_pos_obs_rad, VEL_ESTIMATOR_ORDER - i);
      // Handle denomenator, or previous velocity estimate terms
      vel_est_rs -= vel_estimator_a[i] * at(prev_vel_ests_rs, VEL_ESTIMATOR_ORDER - i);
    }

    double accel_est_rss = vel_estimator_b[0] * vel_est_rs;
    for (unsigned int i = 1; i < VEL_ESTIMATOR_ORDER; ++i) {
      // Handle numerator, or previous velocity estimation terms
      accel_est_rss += vel_estimator_b[i] * at(prev_vel_ests_rs,
          VEL_ESTIMATOR_ORDER - i);
      // Handle denomenator, or previous acceleration estimate terms
      accel_est_rss -= vel_estimator_a[i] * at(prev_accel_ests_rss,
          VEL_ESTIMATOR_ORDER - i);
    }

    // Cycle circular queues 
    dequeue(prev_accel_ests_rss);
    enqueue(prev_accel_ests_rss, accel_est_rss);

    dequeue(prev_vel_ests_rs);
    enqueue(prev_vel_ests_rs, vel_est_rs);

    dequeue(prev_pos_obs_rad);
    enqueue(prev_pos_obs_rad, motor_pos_rad);

    // Get ADC readings
    last_readings(reader, &raw_adc_reading, &amplified_adc_reading);

    // Calculate and execute output.
    double time_s = (num_iters % (SAMPLING_FREQ_HZ * CHIRP_TIME_S)) / loop_wait_time_sec; 
    // calculate chirp output
    float u_of_t = CHIRP_OFFSET + CHIRP_AMP * sin(2 * M_PI * 
       (INIT_FREQ_HZ * time_s + CHIRP_K * time_s * time_s));  
    // CLAMP
    u_of_t = fmin(u_of_t, 1.0);
    u_of_t = fmax(u_of_t, 0.0);
    unsigned int pwm_in = (unsigned int) round(u_of_t * PWM_RANGE);
    gpioPWM(PWM_PIN, pwm_in);
 
    // Record the current state
    LoopState state;
    state.loop_time = curr_time;
    state.motor_count = prev_count;
    state.motor_pos_r = motor_pos_rad;
    state.vel_est_rs = vel_est_rs;
    state.accel_est_rss = accel_est_rss;
    state.u_of_t = pwm_in;
    state.raw_adc = raw_adc_reading;
    state.amplified_adc = amplified_adc_reading;
    history.push_back(state);

    num_iters++;
    // Print if its time to do so
    // TODO clock the operation?
    if ((num_iters % print_loop_freq_iters) == 0) {
      cout << "Motor pos: " << motor_pos_rad << "\t vel: " << vel_est_rs << 
        "\t accel: " << accel_est_rss << "\t adc_raw: " << raw_adc_reading << 
        "\t amped_reading: " << amplified_adc_reading << "\t pwm in: " << pwm_in << endl; 
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
  cout << "Writing to file? " << (argc == 2) << endl;
  if (argc == 2) {
    write_loops_to_file(history, string("chirp_sys_id"));
  }

  return 0;
}

