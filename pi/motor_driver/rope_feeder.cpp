
#include "rope_feeder.hpp"

// This is the rope feeder
RopeFeeder::RopeFeeder(bool make_log, string file_name)
{
  this->make_log = make_log;
  this->output_file = file_name;

  gpioCfgClock(CLK_MICROS, 1, 0);
  if (gpioInitialise() < 0) {
      cout << "Failed GPIO Init" << endl;
      return;
   }
  init_exit_handler();
  if (make_log) {
    // Instead of dynamic memory allocation
    history.reserve(static_cast<long>(round(SAMPLING_FREQ_HZ * RUN_FOR_TIME_SEC * 1.01)));
  }
  init_encoders();
  
  adc_reader = init_adc_reader(
    ADC_CS,
    miso_pins,
    3,
    ADC_MOSI,
    ADC_CLK,
    1); // non reapeating adc reader
  
  int init_pwm_success = !init_motors();
  cout << "Init success?: Motors: " << init_pwm_success << 
    ", and adc: " << (adc_reader != NULL) << endl;
  if (!init_pwm_success || adc_reader == NULL) return;

  
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
}

