/*
gcc -Wall -pthread -o motor_driver driver.c -lpigpio
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pigpio.h>

#include "rotary_encoder.h"


#define DIR_PIN 9
#define PWM_PIN 11

#define ENCODER_A_PIN 27
#define ENCODER_B_PIN 22

// duty cycle range used by pwm
#define PWM_RANGE 2000
// frequency of PWM output in HZ
#define PWM_FREQ_HZ 8000

void callback(int dir)
{
  static int pos = 0;
  // Increment the angular position on each encoder tic
  pos += dir;
  printf("pos=%d\n", pos);
}

int main(int argc, char *argv[])
{
  double end;

  if (gpioInitialise() < 0) return 1;
  
  // Encoder state and initalization
  Pi_Renc_t* renc;
  renc = Pi_Renc(ENCODER_A_PIN, ENCODER_B_PIN, callback);

  // Configure pins to output  
  gpioSetMode(DIR_PIN,  PI_OUTPUT);
  gpioSetMode(PWM_PIN, PI_OUTPUT);
 
  // Turn on forward direction
  gpioWrite(DIR_PIN, 1);

  // Not necessarily the real freq because of sample rate considerations
  int real_freq_hz = gpioSetPWMfrequency(PWM_PIN, PWM_FREQ_HZ);
  printf("Real freq: %d\n", real_freq_hz);
  gpioSetPWMrange(PWM_PIN, PWM_RANGE);
  unsigned int half_on = PWM_RANGE / 2;
  int set_pwm = gpioPWM(PWM_PIN, half_on);
  printf("Set pwm?: %d\n", set_pwm);

  sleep(1000);
  end = time_time();
  printf("End %f\n", end);

  Pi_Renc_cancel(renc);

  gpioTerminate();
  return 0;
}

