/*
gcc -Wall -pthread -o motor_driver driver.c -lpigpio
*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <pigpio.h>

#include "adc_reader.h"

#define DIR_PIN 17
#define PWM_PIN 27

// duty cycle range used by pwm
#define PWM_RANGE 2000
// ideal frequency of PWM output in HZ
#define PWM_FREQ_HZ 8000


int main(int argc, char *argv[])
{
  if (gpioInitialise() < 0) {
    printf("Failed GPIO INIT");
    return 1;
  }
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

  // Chill for a second
  sleep(10);

  int turned_off = gpioPWM(PWM_PIN, 0);
  gpioWrite(PWM_PIN, 0);
  gpioWrite(DIR_PIN, 0);
  printf("Turned PWM off\n");

  gpioTerminate();
  return 0;
}

