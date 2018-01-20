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

// Order of the estimator
#define VEL_ESTIMATOR_ORDER 6

// Velocity filter parameters, numerator and denomenator terms of IIR filter
// implicit a_1 = 1.0 filter term included unecessarily 
static float vel_estimator_b[VEL_ESTIMATOR_ORDER] = {
         0.020396475318968,  -0.058825123415598,   0.089540672078545,  -0.089540672078545,   0.058825123415598, -0.020396475318968};
static float vel_estimator_a[VEL_ESTIMATOR_ORDER] = { 
         1.0, -3.696756079437830, 4.983060887903212, -2.753504247774058, 0.345531791427655, 0.121682695559447};

// Assuming quadrature encoder
static int COUNTS_PER_REVOLUTION = PULSES_PER_REVOLUTION * 4;

// How many radians does each tic correspond to?
static double RADS_PER_COUNT = M_2_PI / (PULSES_PER_REVOLUTION * 4);

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
 
  long num_iters = 0;

  // Initalize circular buffer of previous encoding measurements estimates
  Queue* prev_vel_ests_rs = createQueue(VEL_ESTIMATOR_ORDER);
  Queue* prev_pos_obs_rad = createQueue(VEL_ESTIMATOR_ORDER);
  float motor_pos_rad = main_motor_count * RADS_PER_COUNT;
  for (unsigned int i = 0; i < VEL_ESTIMATOR_ORDER; ++i) {
    enqueue(prev_vel_ests_rs, 0.0);
    enqueue(prev_pos_obs_rad, motor_pos_rad);
  }

  // Sampling loop, do for ten seconds
  while(curr_loop_time.tv_sec - start_time.tv_sec < RUN_FOR_TIME_SEC) {
    // Get the current motor position in radians
    motor_pos_rad = main_motor_count * RADS_PER_COUNT;
   
    // TODO occasional numerical conditioning considerations by keeping 
    // motor position small
    
    // Do IIR filter calculatorions
    float vel_est_rs = vel_estimator_b[0] * motor_pos_rad;
    for (unsigned int i = 1; i < VEL_ESTIMATOR_ORDER; ++i) {
      vel_est_rs += vel_estimator_b[i] * at(prev_vel_ests_rs, i - 1);
      vel_est_rs -= vel_estimator_a[i] * at(prev_pos_obs_rad, i - 1);
    }
    
    // Cycle circular queues 
    dequeue(prev_vel_ests_rs);
    enqueue(prev_vel_ests_rs, vel_est_rs);

    dequeue(prev_pos_obs_rad);
    enqueue(prev_pos_obs_rad, motor_pos_rad);

    num_iters++;
    // Print if its time to do so
    // TODO probably really screws up the timeing on these iterations
    if (!(num_iters % print_loop_freq)) {
      printf("Motor pos: %f\t vel: %f \t", motor_pos_rad, vel_est_rs);
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

