#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <pigpio.h>

#include "rotary_encoder.h"
#include "queue.h"

#define DIR_PIN 9
#define PWM_PIN 11

#define ENCODER_A_PIN 27
#define ENCODER_B_PIN 22

// duty cycle range used by pwm
#define PWM_RANGE 2000
// ideal frequency of PWM output in HZ
#define PWM_FREQ_HZ 8000
// frequency the count is quieried
#define SAMPLING_FREQ_HZ 1000
#define PULSES_PER_REVOLUTION 2048
#define RUN_FOR_TIME_SEC 10
// defines frequency of printout
#define PRINT_FREQ_HZ 5

// Order of the estimator
#define VEL_ESTIMATOR_ORDER 6

// Quadrature Encoder gets 4 counts per pulse
//static int COUNTS_PER_REVOLUTION = 4 * PULSES_PER_REVOLUTION;

// Velocity filter parameters, numerator and denomenator terms of IIR filter
// implicit a_1 = 1.0 filter term included unecessarily 
static float vel_estimator_b[VEL_ESTIMATOR_ORDER] = {
         0.020396475318968,  -0.058825123415598,   0.089540672078545,  -0.089540672078545,   0.058825123415598, -0.020396475318968};
static float vel_estimator_a[VEL_ESTIMATOR_ORDER] = { 
         1.0, -3.696756079437830, 4.983060887903212, -2.753504247774058, 0.345531791427655, 0.121682695559447};

// How many radians does each tic correspond to?
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
  return;
}

int main(int argc, char *argv[])
{
  if (gpioInitialise() < 0) {
    printf("Failed GPIO INIT");
    return 1;
  }
  // Encoder state and initalization
  Pi_Renc_t* renc;
  renc = Pi_Renc(ENCODER_A_PIN, ENCODER_B_PIN, callback);

  // Configure pins to output  
  gpioSetMode(DIR_PIN,  PI_OUTPUT);
  gpioSetMode(PWM_PIN, PI_OUTPUT);
 
  // Turn on forward direction
  gpioWrite(DIR_PIN, 1);

  // The asked for frequency isn't necessarily the set one
  // because of sample rate considerations
  // int real_freq_hz = gpioSetPWMfrequency(PWM_PIN, PWM_FREQ_HZ);
  // printf("Real PWM Frequency: %d\n", real_freq_hz);

  // Set range controlling fully on vs fully off behavior
  // gpioSetPWMrange(PWM_PIN, PWM_RANGE);

  // Set PWM output to 50% duty cycle
  // unsigned int half_on = PWM_RANGE / 2;
  // int set_pwm = gpioPWM(PWM_PIN, half_on);
  // printf("Set pwm?: %d\n", set_pwm);

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

  // Initalize circular buffer of previous encoding measurements estimates
  Queue* prev_vel_ests_rs = createQueue(VEL_ESTIMATOR_ORDER + 1);
  Queue* prev_pos_obs_rad = createQueue(VEL_ESTIMATOR_ORDER + 1);
  float motor_pos_rad = main_motor_count * RADS_PER_COUNT;
  for (unsigned int i = 0; i < VEL_ESTIMATOR_ORDER; ++i) {
    enqueue(prev_vel_ests_rs, 0.0);
    enqueue(prev_pos_obs_rad, motor_pos_rad);
  }

  // What percentage of iterations have extra time to spare?
  // Want it to be 99.9999 or so
  double num_iters_with_time = 0;

  // Sampling loop, do for ten seconds
  // TODO better difference operator
  while(curr_loop_time.tv_sec - start_time.tv_sec < RUN_FOR_TIME_SEC) {
    // Get the current motor position in radians
    motor_pos_rad = main_motor_count * RADS_PER_COUNT;
   
    // TODO occasional numerical conditioning considerations by keeping 
    // motor position small
    
   // Do IIR filter calculatorions
    float vel_est_rs = vel_estimator_b[0] * motor_pos_rad;
    for (unsigned int i = 1; i < VEL_ESTIMATOR_ORDER; ++i) {
      // double check the sanity of this
      // Handle numerator, or previous position obseration terms
      vel_est_rs += vel_estimator_b[i] * at(prev_pos_obs_rad, VEL_ESTIMATOR_ORDER - i);
      // Handle denomenator, or previous velocity estimate terms
      vel_est_rs -= vel_estimator_a[i] * at(prev_vel_ests_rs, VEL_ESTIMATOR_ORDER - i);
    }
    
    // Cycle circular queues 
    dequeue(prev_vel_ests_rs);
    enqueue(prev_vel_ests_rs, vel_est_rs);

    dequeue(prev_pos_obs_rad);
    enqueue(prev_pos_obs_rad, motor_pos_rad);

    num_iters++;
    // Print if its time to do so
    // TODO probably really screws up the timeing on these iterations
    // TODO clock the operation?
    if ((num_iters % print_loop_freq_iters) == 0) {
      printf("Motor pos: %f\t vel: %f \t%li \n", motor_pos_rad, vel_est_rs, main_motor_count);
    }

    // Wait till the time for this loop expires
    // TODO add automatic check that the sampling time isn't too fast
    // TODO better robust wait method?
    unsigned int did_n_times = 0;
    do{
      clock_gettime(CLOCK_REALTIME, &curr_time);
      // Needs to do a slightly more difficult difference operation
      timespec_diff(&curr_loop_time, &curr_time, &time_diff);
      did_n_times++;
    } while(time_diff.tv_nsec < loop_wait_time_nsec);
   
    // Check if that we had time to spare
    if (did_n_times > 1) {
      num_iters_with_time++; 
    }
    curr_loop_time = curr_time;
  }


  timespec_diff(&start_time, &curr_time, &time_diff);
  printf("Time: %li\n Expected time: %f\n", time_diff.tv_sec, num_iters * loop_wait_time_sec); 

  printf("Iters with time: %f\n", num_iters_with_time / (double) num_iters);

  // Don't forget to kill the encoder count process before exiting
  Pi_Renc_cancel(renc);
  gpioTerminate();
  return 0;
}

