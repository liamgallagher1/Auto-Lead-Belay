#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <pigpio.h>

#include "adc_reader.h"
#include "rotary_encoder.h"
#include "queue.h"

#define DIR_PIN 17
#define PWM_PIN 27

#define ENCODER_A_PIN 16
#define ENCODER_B_PIN 20

// TODO fix pins

#define SPI_PIN  22 // GPIO for slave select.
#define MISO_PIN 19 // Input 
#define MOSI_PIN 26 // Output
#define CLK_PIN  13 // Clock


// duty cycle range used by pwm
#define PWM_RANGE 2000
// ideal frequency of PWM output in HZ
#define PWM_FREQ_HZ 8000
// frequency the count is quieried
#define SAMPLING_FREQ_HZ 1000
#define PULSES_PER_REVOLUTION 2048
#define RUN_FOR_TIME_SEC 5
// defines frequency of printout
#define PRINT_FREQ_HZ 10
// Order of the estimator
#define VEL_ESTIMATOR_ORDER 12
#define ACCEL_ESTIMATOR_ORDER 10

#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))


// Quadrature Encoder gets 4 counts per pulse
static int COUNTS_PER_REVOLUTION = 4 * PULSES_PER_REVOLUTION;

// Velocity filter parameters, numerator and denomenator terms of IIR filter
// implicit a_1 = 1.0 filter term included unecessarily 
static float vel_estimator_b[VEL_ESTIMATOR_ORDER] = {
    0.1202, -0.2310, 0.4283, -0.4000, 0.3820, -0.2771, 0.2771, -0.3820, 0.4000, -0.4283, 0.2310, -0.1202
};

static float vel_estimator_a[VEL_ESTIMATOR_ORDER] = { 
    1.0000, -6.2197, 17.7840, -30.5757, 34.7733, -27.0961, 14.4329, -5.0187, 0.9795,   -0.0385,   -0.0234, 0.0035
};

static float accel_estimator_b[ACCEL_ESTIMATOR_ORDER] = {
  9.3759, -75.3733, 276.0036, -605.9827,  881.1613, -881.1613,  605.9827, -276.0036, 75.3733,  -9.3759
};

static float accel_estimator_a[ACCEL_ESTIMATOR_ORDER] = {
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
  if (gpioInitialise() < 0) {
    printf("Failed GPIO INIT");
    return 1;
  }
  // Encoder state and initalization
  Pi_Renc_t* renc;
  renc = Pi_Renc(ENCODER_A_PIN, ENCODER_B_PIN, callback);

  // ADC state and initalization
  // janky until we actually do multiple inputs
  int only_miso_pin = MISO_PIN;
  ADC_Reader* reader = init_adc_reader(SPI_PIN, &only_miso_pin, 1, MOSI_PIN, CLK_PIN);

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

  int queue_size = MAX(VEL_ESTIMATOR_ORDER, ACCEL_ESTIMATOR_ORDER) + 1;
  // Initalize circular buffer of previous encoding measurements estimates
  Queue* prev_accel_ests_rss = createQueue(queue_size + 1);
  Queue* prev_vel_ests_rs =    createQueue(queue_size + 1);
  Queue* prev_pos_obs_rad =    createQueue(queue_size + 1);
  
  float motor_pos_rad = main_motor_count * RADS_PER_COUNT;
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

    float accel_est_rss = vel_estimator_b[0] * vel_est_rs;
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


    num_iters++;
    // Print if its time to do so
    // TODO probably really screws up the timeing on these iterations
    // TODO clock the operation?
    if ((num_iters % print_loop_freq_iters) == 0) {
      printf("Motor pos: %f\t count: %li\t vel: %f \t accel: %f \t adc_raw: %d \t adc_amp: %d \n", motor_pos_rad, main_motor_count, vel_est_rs, accel_est_rss, raw_adc_reading, amplified_adc_reading);
    }


    // Loop timing code
    clock_gettime(CLOCK_REALTIME, &curr_time);
    timespec_diff(&curr_loop_time, &curr_time, &time_diff);
    total_inner_loop_time_ns += time_diff.tv_nsec;
    max_inner_loop_time_ns = MAX(time_diff.tv_nsec, max_inner_loop_time_ns);

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

  printf("maximum inner loop time, min freq: %li \t %f\n", max_inner_loop_time_ns, 1000000000 / (double) max_inner_loop_time_ns);
  total_inner_loop_time_ns /= num_iters;

  printf("Average inner loop time, freq: %li \t %f\n",(long) total_inner_loop_time_ns,  1000000000 / (double) total_inner_loop_time_ns); 
  printf("Iters with time: %f\n", num_iters_with_time / (double) num_iters);

  // Turn off PWM?
  set_pwm = gpioPWM(PWM_PIN, 0);


  // Don't forget to kill the encoder count process before exiting
  Pi_Renc_cancel(renc);
  free_adc_reader(reader);
  gpioTerminate();
  return 0;
}

