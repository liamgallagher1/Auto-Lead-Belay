#include <iostream>

#include <pigpio.h>
#include <thread>
#include <chrono>

// Source is in C 
extern "C"
{
#include "rotary_encoder.h"
}
#include "pwm_driver.hpp"
#include "time_functions.hpp"

#define SAMPLING_FREQ_HZ 400
#define PRINT_FREQ_HZ 0
#define RUN_FOR_TIME_SEC 6

#define PWM_FREQ_HZ 10000 // barely ultrasonic
#define PWM_I2C_ADDR 0x40
#define PWM_I2C_BUS 1

#define MOTOR_1_CHANNEL 0
#define MOTOR_2_CHANNEL 1
#define LIN_ACT_CHANNEL 2


int main(int argc, char *argv[])
{
  int num_iters = 0;

  if (gpioInitialise() < 0) {
    cout << "Failed GPIO Init" << endl;
    return 1;
  }

//  // Encoder state and initalization
//  Pi_Renc_t* rot_encoder_1;
//  Pi_Renc_t* rot_encoder_2;
//  rot_encoder_1 = 
//    Pi_Renc(ENCODER_1_A_PIN, ENCODER_1_B_PIN, callback);
//  rot_encoder_2 = 
//    Pi_Renc(ENCODER_2_A_PIN, ENCODER_2_B_PIN, callback);

// PWM Driver Initalization
  PwmDriver pwm_driver(PWM_I2C_BUS, PWM_I2C_ADDR, PWM_FREQ_HZ);
  pwm_driver.set_duty_cycle(1, 30);

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
  
  while(!past_time(&curr_loop_time, &finish_time) ) {
    num_iters++;
    pwm_driver.set_duty_cycle(-1, 60);

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
  
  pwm_driver.cancel();
  gpioTerminate();
  return 0;
}



