#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <pigpio.h>

#include "pwm_driver.hpp"

using namespace std;

PwmDriver::PwmDriver(
  uint16_t bus,         // 1
  uint16_t address,     // 0x40
  double frequency_hz)   // 10 kHZ to be ultrasonic
{
  this->_i2c_handle = i2cOpen(bus, address, 0);
  if (_i2c_handle < 0) {
    std::cout << "Bad handle, pwm driver init failed" << std::endl;
    return;
  }
  i2cWriteByteData(_i2c_handle, _MODE1, _AI | _ALLCALL);
  i2cWriteByteData(_i2c_handle, _MODE2, _OCH | _OUTDRV);


  std::chrono::nanoseconds timespan(500);
  std::this_thread::sleep_for(timespan);

  int mode = i2cReadByteData(_i2c_handle, _MODE1);
  i2cWriteByteData(_i2c_handle, _MODE1, mode & ~_SLEEP);

  std::this_thread::sleep_for(timespan);
 
  set_duty_cycle(-1, 0);
  set_frequency_hz(frequency_hz);

  cout << (_AI | _ALLCALL) << ", " << (_OCH | _OUTDRV) << ", " <<
    (mode & ~_SLEEP) << endl;
}


double PwmDriver::get_frequency_hz(void) const
{
  return this->_frequency_hz;
}

void PwmDriver::set_frequency_hz(double freq_hz)
{
  this->_frequency_hz = freq_hz;
  int prescale = static_cast<int>(round(25000000.0 / (4096.0 * freq_hz)) - 1);
  if (prescale < 3) {
    prescale = 3;
  } else if (prescale > 255) {
    prescale = 255;
  }
  int mode = i2cReadByteData(_i2c_handle, _MODE1);
  i2cWriteByteData(_i2c_handle, _MODE1, (mode & ~_SLEEP) | _SLEEP);
  i2cWriteByteData(_i2c_handle, _PRESCALE, prescale);
  i2cWriteByteData(_i2c_handle, _MODE1, mode);

  std::chrono::nanoseconds timespan(500);
  std::this_thread::sleep_for(timespan);
  
  i2cWriteByteData(_i2c_handle, _MODE1, mode | _RESTART);

  this->_frequency_hz = (25000000.0 / 4096.0) / (prescale + 1);
  cout << "freq: " << ((mode & ~_SLEEP) | _SLEEP) << ", " << prescale << ", " << mode << ", " << (mode | _RESTART) << "\t" << _frequency_hz << endl;

}

void PwmDriver::set_duty_cycle(int16_t channel, double percent)
{
  this->_duty_cycle_percent = percent;

  uint16_t steps = static_cast<uint16_t>(round(percent * (4096.0 / 100.0)));

  cout << " steps: " << steps << endl;
  int on, off = 0;
  if (steps < 0) {
    on = 0;
    off = 4096;
  } else if (steps > 4095) {
    on = 4096;
    off = 0;
  } else {
    on = 0;
    off = steps;
  }
  char buff[4];
  buff[0] = on & 0xFF;
  buff[1] = on >> 8; 
  buff[2] = off & 0xFF;
  buff[3] = off >> 8;

  cout << "buff: " << ((int) buff[0] )<< "," << ((int) buff[1]) << ", " << ((int) buff[2]) << ", " <<( (int) buff[3] )<< endl;

  int i2c_led_channel;
  if (channel >= 0 && channel <= 15) {
    i2c_led_channel = _LED0_ON_L + 4 * channel;
    std::cout << "duty cycle write: " << i2cWriteBlockData(_i2c_handle, i2c_led_channel, buff, 4) << std::endl;
  } else {
    i2c_led_channel = _ALL_LED_ON_L;
    std::cout << "duty cycle write: " << i2cWriteBlockData(_i2c_handle, i2c_led_channel, buff, 4) << std::endl;
  }
  cout << "channel: " << channel << ", " << i2c_led_channel << endl;
}

void PwmDriver::cancel(void)
{
  set_duty_cycle(-1, 0);
  i2cClose(_i2c_handle);
}

