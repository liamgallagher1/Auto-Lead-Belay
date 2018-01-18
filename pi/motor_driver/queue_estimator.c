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
#define SAMPLING_FREQ_HZ 500
#define PULSES_PER_REVOLUTION 2048
#define RUN_FOR_TIME_SEC 10

// defines frequency of printout
#define PRINT_FREQ_HZ 5.0

// Assuming quadrature encoder
static int COUNTS_PER_REVOLUTION = PULSES_PER_REVOLUTION * 4;

// How many radians does each tic correspond to?
static double RADS_PER_COUNT = M_2_PI / (PULSES_PER_REVOLUTION * 4);

// Simple low pass filter parameters for estimating velocity and acceleration
static double vel_alpha = 0.05;
static double accel_alpha = 0.3;

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
  int real_freq_hz = gpioSetPWMfrequency(PWM_PIN, PWM_FREQ_HZ);
  printf("Real PWM Frequency: %d\n", real_freq_hz);

  // Set range controlling fully on vs fully off behavior
  gpioSetPWMrange(PWM_PIN, PWM_RANGE);

  // Set PWM output to 50% duty cycle
  unsigned int half_on = PWM_RANGE / 2;
  int set_pwm = gpioPWM(PWM_PIN, half_on);
  printf("Set pwm?: %d\n", set_pwm);

  double loop_wait_time_sec = round(1.0 / SAMPLING_FREQ_HZ);
  long loop_wait_time_nsec = (long) (loop_wait_time_sec * 1E9);
  long print_loop_freq = (long) round(1.0 / (PRINT_FREQ_HZ * loop_wait_time_sec));

  printf("Actual sampling freq: %f\n", 1.0 /loop_wait_time_nsec);

  // Chill for a second
  sleep(1);
  
  // Use timespec to wait for precise amounts of time
  // Time that we started the prgram after init,
  struct timespec start_time;
  // Time that we started this loop,
  struct timespec curr_loop_time;
  // Current time, if we are waiting for the loops period to expire
  struct timespec curr_time;
  clock_gettime(CLOCK_REALTIME, &start_time);
  clock_gettime(CLOCK_REALTIME, &curr_loop_time);
 
  // TODO remove this if using the circular buffer?
  // Motor count on the previous loop
  long prev_motor_count = main_motor_count;
  // Motor sate in rads, rads per sec, rads per sec per sec
  double prev_motor_rad = prev_motor_count * M_2_PI / COUNTS_PER_REVOLUTION; 
  // Init to zero.   
  double prev_motor_omega_rs = 0;
  double prev_motor_alpha_rss = 0;

  long num_iters = 0;

  // Initalize circular buffer of previous encoding measurements estimates
  Queue* prev_2_meas = createQueue(2);
  Queue* prev_2_est = createQueue(2);
  for (unsigned int i = 0; i < 2; ++i) {
    enqueue(prev_2_meas, prev_motor_rad);
    enqueue(prev_2_ests, prev_motor_rad);
  }
  
  // Filter constants
  // Motor position estimate comes from second order low pass filter
  // which uses the previous measurements and estimates, internally
  float curr_mes_p = 0.947923447467778;
  float prev_mes_1_p =  0.021091003775550; 
  float prev_mes_2_p = -0.926832443692228;

  float prev_est_1_p = 0.083062101288895;
  float prev_est_2_p = 0.874755891160005;

  // Sampling loop, do for ten seconds
  // TODO consider single precision floats to save time?
  while(curr_loop_time.tv_sec - start_time.tv_sec < RUN_FOR_TIME_SEC) {
    // Estimate the motor position using a second order lowpass filter
    float curr_encoder_measure_rad = main_motor_count * RADS_FOR_COUNT;

    float curr_encoder_est_rad = curr_meas_p * curr_encoder_measure_rad +
      prev_mes_0_p * at(prev_2_meas, 0) + prev_mes_1_p * at(prev_2_meas, 1) +
      prev_est_0_p * at(prev_2_ests, 0) + prev_est_1_p * at(prev_2_ests, 1);

    // Update the previous values using simple lowpass filters
    // TODO use second or third order filters here, justifying need for circular buffers
    prev_motor_rad = prev_motor_count * M_2_PI / COUNTS_PER_REVOLUTION;
    prev_motor_omega_rs = vel_alpha * motor_omega_instant_rs + (1.0 - vel_alpha) * prev_motor_omega_rs;
    prev_motor_alpha_rss = accel_alpha * motor_alpha_instant_rss + (1.0 - accel_alpha) * prev_motor_alpha_rss;

    enqueue(prev_2_measures, prev_motor_rad);
    enqueue(prev_2_ests, curr_motor_est_rad);

    num_iters++;
    if (!(num_iters % print_loop_freq)) {
      printf("Motor pos: %f\t vel: %f \t accel: %f\n", prev_motor_rad, prev_motor_omega_rs, prev_motor_alpha_rss);
    }

    // Wait till the time for this loop expires
    // TODO add automatic check that the sampling time isn't too fast
    do{
      clock_gettime(CLOCK_REALTIME, &curr_time);
    } while(curr_time.tv_nsec - curr_loop_time.tv_nsec < loop_wait_time_nsec);
  }

  printf("Time: %li\n", start_time.tv_sec); 
  // Don't forget to kill the encoder count process before exiting
  Pi_Renc_cancel(renc);
  gpioTerminate();
  return 0;
}

