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
#include "adc_reader.hpp"
#include "time_functions.hpp"

#define SAMPLING_FREQ_HZ 400 
#define ADC_FREQ_HZ 400 // must be less than or equal
#define PRINT_FREQ_HZ 5
#define RUN_FOR_TIME_SEC 60

// The big motor
#define MOTOR_1_PWM 18 // Only pin capable of hardware PWM on our model. 
#define MOTOR_1_DIR 15
#define MOTOR_1_FREQ 10000
#define MOTOR_1_RANGE 1000000
#define MOTOR_1_MISO 11

// The small motor
#define MOTOR_2_PWM 22
#define MOTOR_2_DIR 23
#define MOTOR_2_FREQ  4000
#define MOTOR_2_RANGE 50
#define MOTOR_2_MISO 9

// The linear actuator
#define LIN_ACT_PWM 13
#define LIN_ACT_DIR 6
#define LIN_ACT_FREQ 4000
#define LIN_ACT_RANGE 50
#define LIN_ACT_ADC_MISO 11

// ADC
#define ADC_CS 19
#define ADC_MOSI 26
#define ADC_CLK 5

// Encoders
#define ENCODER_1_A 16
#define ENCODER_1_B 12
#define ENCODER_1_X 7

#define ENCODER_2_A 8
#define ENCODER_2_B 25
#define ENCODER_2_X 24

// Resistors
#define BOARD_2_R1 100000.0  // Nominal Resistor Values, worth identifing.
#define BOARD_2_R2 220000.0

#define BOARD_4_R5 470000.0
#define BOARD_4_R4 620000.0
#define BOARD_4_R1 100000.0
#define BOARD_4_R3 330000.0
#define BOARD_4_R6 100000.0
#define BOARD_4_R7 330000.0


volatile long encoder_count_1 = 0;
volatile long encoder_count_2 = 0;

Pi_Renc_t* rot_encoder_1;
Pi_Renc_t* rot_encoder_2;

ADC_Reader* adc_reader;

// Returns 0 if okay
int init_motors(void)
{
  int motor_1_pwm = 0; //gpioHardwarePWM(MOTOR_1_PWM, MOTOR_1_FREQ, round(MOTOR_1_RANGE * 0.5));
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

// Simple Encoder Callback
// Timestamping also an option
// This should all be C++
void encoder_callback_1(int dir)
{
  encoder_count_1 += dir;
}

void encoder_callback_2(int dir)
{
  encoder_count_2 += dir;
}

void init_encoders()
{
  // Encoder state and initalization
  rot_encoder_1 = Pi_Renc(ENCODER_1_A, ENCODER_1_B, encoder_callback_1);
  rot_encoder_2 = Pi_Renc(ENCODER_2_A, ENCODER_2_B, encoder_callback_2);
}

void cancel_encoders(void)
{
  Pi_Renc_cancel(rot_encoder_1);
  Pi_Renc_cancel(rot_encoder_2);
}

void exit_handler(int s)
{
  cout << "Exit given: " << s << endl;
  // TODO turn off all pwm and be safe
  // TODO consider safer ramp down
  gpioPWM(MOTOR_1_PWM, 0);
  gpioPWM(MOTOR_2_PWM, 0);
  gpioPWM(LIN_ACT_PWM, 0);

  // Cancel Encoder ISRs 
  cancel_encoders();
  gpioTerminate();


  // Free adc reader
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

  int init_pwm_success = !init_motors();

  init_encoders();

  int miso_pins[3] = {LIN_ACT_ADC_MISO, 1, 1};
  int channel_0[3] = {-1, -1, -1};
  int channel_1[3] = {-1, -1, -1};

  adc_reader = init_adc_reader(
    ADC_CS,
    miso_pins,
    1,
    ADC_MOSI,
    ADC_CLK,
    1); // non reapeating adc reader

  cout << "Init success?: Motors: " << init_pwm_success << 
    ", and adc: " << (adc_reader != NULL) << endl;

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

  long adc_loop_freq_iters = static_cast<long>(
      round(1.0 / (ADC_FREQ_HZ * loop_wait_time_sec)));
  
  cout << "Sampling freq HZ and period nsec: " <<
    SAMPLING_FREQ_HZ << "\t"  << loop_wait_time_nsec << endl;
  cout << "Print freq HZ and num iters: " << 
    PRINT_FREQ_HZ << "\t" << print_loop_freq_iters << endl;
  cout << "ADC freq and num iters: " << 
    (1.0 / (adc_loop_freq_iters * loop_wait_time_sec)) << ", " 
    << adc_loop_freq_iters << endl;

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
    if (num_iters % 400 == 0) {
      gpioWrite(MOTOR_1_DIR, !motor_1_dir);
      gpioWrite(MOTOR_2_DIR, !motor_1_dir);
      gpioWrite(LIN_ACT_DIR, !motor_1_dir);

      motor_1_dir = !motor_1_dir;
    }

    // Print if its time to do so
    if ((num_iters % print_loop_freq_iters) == 0) {
      cout << "Count 1 and 2: "  << encoder_count_1 << "\t " << encoder_count_2 << endl;

      cout << "ADC: " << channel_0[0] << ", " << channel_1[0] << endl;

    }
    if ((num_iters % adc_loop_freq_iters) == 0) {
      last_readings(adc_reader, channel_0, channel_1);
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



