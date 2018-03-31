#pragma once 


/**  This class provides an interface to the I2C PCA9685 PWM chip.
The chip provides 16 PWM channels.

All channels use the same frequency which may be set in the
range 24 to 1526 Hz.

If used to drive servos the frequency should normally be set
in the range 50 to 60 Hz.

The duty cycle for each channel may be independently set
between 0 and 100%.

It is also possible to specify the desired pulse width in
microseconds rather than the duty cycle.  This may be more
convenient when the chip is used to drive servos.

The chip has 12 bit resolution, i.e. there are 4096 steps
between off and full on. */


class PwmDriver {
  uint8_t  _MODE1         = 0x00;
  uint8_t  _MODE2         = 0x01;
  uint8_t  _SUBADR1       = 0x02;
  uint8_t  _SUBADR2       = 0x03;
  uint8_t  _SUBADR3       = 0x04;
  uint8_t  _PRESCALE      = 0xFE;
  uint8_t  _LED0_ON_L     = 0x06;
  uint8_t  _LED0_ON_H     = 0x07;
  uint8_t  _LED0_OFF_L    = 0x08;
  uint8_t  _LED0_OFF_H    = 0x09;
  uint8_t  _ALL_LED_ON_L  = 0xFA;
  uint8_t  _ALL_LED_ON_H  = 0xFB;
  uint8_t  _ALL_LED_OFF_L = 0xFC;
  uint8_t  _ALL_LED_OFF_H = 0xFD;

  uint8_t  _RESTART = 1<<7;
  uint8_t  _AI      = 1<<5;
  uint8_t  _SLEEP   = 1<<4;
  uint8_t  _ALLCALL = 1<<0;

  uint8_t  _OCH    = 1<<3;
  uint8_t  _OUTDRV = 1<<2;

  uint _i2c_handle;
  
  double _frequency_hz;
  double _duty_cycle_percent;

public:
  PwmDriver(
    uint16_t bus,         // 1
    uint16_t address,     // 0x40
    double frequency_hz);  // 10 kHZ to be ultrasonic

  // Returns the frequency currently set in hz
  double get_frequency_hz(void) const;
  
  // Sets the PWM frequency in Hz, to be used by all channels
  void set_frequency_hz(double freq_hz);

  // Sets the width of the frequency
  void set_duty_cycle(int16_t channel, double percent);

  // Turns off the pwm for all channels. Closes it. Do not use after calling.
  void cancel(void);
};



