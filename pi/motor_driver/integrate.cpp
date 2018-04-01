#include <chrono>
#include <cmath>
#include <iostream>
#include <signal.h>
#include <thread>


#include <pigpio.h>

// Source is in C 
extern "C"
{
#include "rotary_encoder.h"
}
#include "time_functions.hpp"

#define SAMPLING_FREQ_HZ 400
#define PRINT_FREQ_HZ 0
#define RUN_FOR_TIME_SEC 60

// The big motor
#define MOTOR_1_PWM 18 // Only pin capable of hardware PWM on our model. 
#define MOTOR_1_DIR 15
#define MOTOR_1_FREQ 10000
#define MOTOR_1_RANGE 1000000

// The small motor
#define MOTOR_2_PWM 22
#define MOTOR_2_DIR 23
#define MOTOR_2_FREQ  4000
#define MOTOR_2_RANGE 50

// The linear actuator
#define LIN_ACT_PWM 26
#define LIN_ACT_DIR 19
#define LIN_ACT_FREQ 4000
#define LIN_ACT_RANGE 50


// Returns 0 if okay
int init_pwm(void)
{
  int motor_1_pwm = gpioHardwarePWM(MOTOR_1_PWM, MOTOR_1_FREQ, round(MOTOR_1_RANGE * 0.5));
  gpioSetMode(MOTOR_1_DIR, PI_OUTPUT);
  gpioWrite(MOTOR_1_DIR, 1); 

  int motor_2_freq = gpioSetPWMfrequency(MOTOR_2_PWM, MOTOR_2_FREQ);
  int motor_2_range = gpioSetPWMrange(MOTOR_2_PWM, MOTOR_2_RANGE);
  int motor_2_pwm = gpioPWM(MOTOR_2_PWM, round(MOTOR_2_RANGE * 0.5));
  gpioSetMode(MOTOR_2_DIR, PI_OUTPUT);
  gpioWrite(MOTOR_2_DIR, 1);

  int lin_act_freq = gpioSetPWMfrequency(LIN_ACT_PWM, LIN_ACT_FREQ);
  int lin_act_range = gpioSetPWMrange(LIN_ACT_PWM, LIN_ACT_RANGE);
  int lin_act_pwm = gpioPWM(LIN_ACT_PWM, round(LIN_ACT_RANGE * 0.3));
  gpioSetMode(LIN_ACT_DIR, PI_OUTPUT);
  gpioWrite(LIN_ACT_DIR, 1);

  cout << "ranges: " << motor_2_range << ", " << lin_act_pwm << endl;
  return motor_1_pwm && motor_2_freq && motor_2_range && motor_2_pwm && 
        lin_act_freq && lin_act_range && lin_act_pwm;
}

void exit_handler(int s)
{
  cout << "Exit given: " << s << endl;
  // TODO turn off all pwm and be safe
  // TODO consider safer ramp down
  gpioPWM(MOTOR_1_PWM, 0);
  gpioPWM(MOTOR_2_PWM, 0);
  gpioPWM(LIN_ACT_PWM, 0);
  gpioTerminate();
  exit(s);
}

void init_exit_handler(void)
{
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char *argv[])
{
  int num_iters = 0;

  if (gpioInitialise() < 0) {
    cout << "Failed GPIO Init" << endl;
    return 1;
  }
  init_exit_handler();

  int init_pwm_success = init_pwm();
  cout << init_pwm_success << endl;
//  // Encoder state and initalization
//  Pi_Renc_t* rot_encoder_1;
//  Pi_Renc_t* rot_encoder_2;
//  rot_encoder_1 = 
//    Pi_Renc(ENCODER_1_A_PIN, ENCODER_1_B_PIN, callback);
//  rot_encoder_2 = 
//    Pi_Renc(ENCODER_2_A_PIN, ENCODER_2_B_PIN, callback);


  // driver.set_duty_cycle(MOTOR_2_CHANNEL, 0.6);
  // driver.set_duty_cycle(LIN_ACT_CHANNEL, 0.9);

  /** 
  * The asked for frequency isn't necessarily the set one
  * because of sample rate considerations
  */
  double loop_wait_time_sec = (1.0 / SAMPLING_FREQ_HZ);
  // Time to wait per iteration
  long loop_wait_time_nsec = static_cast<long>(
      round(loop_wait_time_sec * 1E9));
  // Print out ever kth iteration 
  long print_loop_freq_iters = static_cast<long>(
      round(1.0 / (PRINT_FREQ_HZ * loop_wait_time_sec)));

  cout << "Sampling freq HZ and period nsec: " <<
    SAMPLING_FREQ_HZ << "\t"  << loop_wait_time_nsec << endl;
  cout << "Print freq HZ and num iters: " << 
    PRINT_FREQ_HZ << "\t" << print_loop_freq_iters << endl;


  /**
  * Use timespec to wait for precise amounts of time
  * Time that we started the prgram after initialization
  */ 
  struct timespec start_time;
  // Time to stop running;
  struct timespec finish_time;
  // Time that we started the current loop,
  struct timespec curr_loop_time;
  // Current time, if we are waiting for the loop's period to expire
  struct timespec curr_time;
  // Time to start next loop. Always add to it the proper number of nanoseconds.
  struct timespec next_loop_time;  
 
  clock_gettime(CLOCK_REALTIME, &curr_time);
  clock_gettime(CLOCK_REALTIME, &start_time);
  add_time(&start_time, RUN_FOR_TIME_SEC, 0, &finish_time);
  clock_gettime(CLOCK_REALTIME, &curr_loop_time);
  add_time(&curr_loop_time, 0, loop_wait_time_nsec, &next_loop_time); 
  
  cout << "Entering Inner Loop " << endl;
 
  int motor_1_dir = 0;
  while(!past_time(&curr_loop_time, &finish_time) ) {
    num_iters++;
    if (num_iters % 100 == 0) {
      gpioWrite(MOTOR_1_DIR, !motor_1_dir);
      gpioWrite(MOTOR_2_DIR, !motor_1_dir);
      gpioWrite(LIN_ACT_DIR, !motor_1_dir);

      motor_1_dir = !motor_1_dir;
    }

    // Print if its time to do so
    if ((num_iters % print_loop_freq_iters) == 0) {
      cout << "Print here" << endl;
    }

    // Loop timing code
    do{
      clock_gettime(CLOCK_REALTIME, &curr_time);
    } while(!past_time(&curr_time, &next_loop_time));
    
    add_time(&next_loop_time, 0, loop_wait_time_nsec, &next_loop_time);
    curr_loop_time = curr_time;
  }


  // Turn off PWM?
  // Don't forget to kill the encoder count process before exiting
  // Pi_Renc_cancel(renc);
 
  exit_handler(0);
  return 0;
}



