#include <chrono>
#include <cmath>
#include <iostream>
#include <signal.h>
#include <string.h>
#include <string>
#include <thread>
#include <pigpio.h>

// Source is in C 
extern "C"
{
#include "rotary_encoder.h"
}
#include "adc_reader.hpp"
#include "loop_state.hpp"
#include "time_functions.hpp"

using namespace std;

static int SAMPLING_FREQ_HZ = 1000; 
static int ADC_FREQ_HZ = 1000; // must be less than or equal
static int PRINT_FREQ_HZ = 5;
static int RUN_FOR_TIME_SEC = 120;
static int CLK_MICROS = 1; // pigpio pwm clk sample rate. 1 microsecond is the highest precision, but uses a whole core
  
  // The big motor
static int MOTOR_1_PWM = 18; // Only pin capable of hardware PWM on our model. 
static int MOTOR_1_DIR = 15;
static int MOTOR_1_FLIP_DIR = 1; // Change the sign of this to flip all motor command directions
static int MOTOR_1_FREQ = 4000;
static int MOTOR_1_RANGE = 250;
static int MOTOR_1_MISO = 9;
static double MOTOR_1_MAX_V = 12.0;
  
  // The small motor
static int MOTOR_2_PWM = 6;
static int MOTOR_2_DIR = 13;
static int MOTOR_2_FLIP_DIR = 1;
static int MOTOR_2_FREQ  = 4000;
static int MOTOR_2_RANGE = 250;
static int MOTOR_2_MISO = 11;
static double MOTOR_2_MAX_V = 12.0;
  
  // The linear actuator
static int LIN_ACT_PWM = 22;
static int LIN_ACT_DIR = 23;
static int LIN_ACT_FLIP_DIR = 1;
static int LIN_ACT_FREQ = 4000;
static int LIN_ACT_RANGE = 250;
static int LIN_ACT_ADC_MISO = 10;
static double LIN_ACT_MAX_A = 0.42;
static double LIN_ACT_MAX_V = 12.0;
static double LIN_ACT_KP_A = 0.1;
  
  // ADC
static int ADC_CS = 19;
static int ADC_MOSI = 26;
static int ADC_CLK = 5;
  
// Encoders
// small motor encoder
static int ENCODER_1_A = 16;
static int ENCODER_1_B = 12;
static int ENCODER_1_X = 7;
static int ENCODER_1_CPR = 8192;
// big motor encoder
static int ENCODER_2_A = 8;
static int ENCODER_2_B = 25;
static int ENCODER_2_X = 24;
static int ENCODER_2_CPR = 8192;

// Learned amplification constants
static double LIN_ACT_AMP = 4.5458; 
static double MOTOR_2_AMP = 3.2912;
static double MOTOR_1_OFFSET_V = -1.4119;
static double MOTOR_1_AMP  = 3.3686;
static double MOTOR_1_NO_CURRENT_V = 1.7225; 

// Rope amount constants
static double UP_RADIUS = 0.060217658443764;
static double SPOOL_R0 =  0.185789405425864;
static double SPOOL_A = -5.492535985428214E-6;

// Dynamic Global Variables. I regret their existance. 

// Encoders
volatile long sm_encoder_count = 0;
  // Encoder 1
volatile long lm_encoder_count = 0;

// Asociated angles [rad]
double sm_theta = 0.0;
double lm_theta = 0.0;

// Calculated from sm_theta [m]
double rope_out = 0.0;
// Amount of slack between the upper pulley and main spool
double up_slack = 0.0;
// Previous encoder counts to calculate the delta on the rope out
double prev_sm_theta = 0;
double prev_rope_out = 0;
bool pulling_in_slack = false;

// Dynamic Variables
//Encoder 1
Pi_Renc_t* rot_encoder_1;
Pi_Renc_t* rot_encoder_2;
ADC_Reader* adc_reader;
int miso_pins[3] = {LIN_ACT_ADC_MISO, MOTOR_2_MISO, MOTOR_1_MISO};
int channel_0[3] = {-1, -1, -1};
int channel_1[3] = {-1, -1, -1};
double raw_current_readings[3] = {-1.0, -1.0, -1.0};
double amp_current_readings[3] = {-1.0, -1.0, -1.0};
bool use_amp_readings[3] = {false, false, false};


float pwm_commands[3] = {0.0, 0.0, 0.0};
// Time that we started the current loop,
struct timespec curr_loop_time;
// Number of this iteration
long num_iters = 0;
bool make_log = false;
string output_file = string("rope_feed_test_");
vector<LoopState> history;

// Iteration information
long print_loop_freq_iters = 0;
long adc_loop_freq_iters = 0;
long loop_wait_time_nsec = 0;

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



int inner_loop(void);
int init_motors(void);
static void encoder_callback_1(int dir);
static void encoder_callback_2(int dir);
void init_encoders(void);
// Calculates current readings for each actuator
void current_calculations(
  int* channel_0_in,
  int* channel_1_in,
  double* raw_current_out,
  double* amplified_current_out,
  bool*  use_amped_est);

void add_loop_state(vector<LoopState>& history);
void cancel_encoders(void);
void exit_handler(int s);
void init_exit_handler(void);

// Set the duty cycles (percents) for each actuator
int set_la_dc(float dc);
int set_lm_dc(float dc);
int set_sm_dc(float dc);
float identification_dc(struct timespec* init_loop_time);
float get_la_dc(float desired_current_a);

// Control logic
void update_slack_count(void); 

// Gives duty cycle for upper pulley if its time to pull in slack.
// Outputs true when done
void clear_slack(float& dc, bool& done_with_pull_in);



// main function
int init(void);
int main_loop(void);
int finish_and_quit(void);
int make_logs(void);

int main(int argc, char* argv[]){
  if (argc < 2) { 
    cout << 
      "Include an argument 0 if you don't want to output a log or 1 if you want to output a log" 
      << endl; 
    return 1; 
  }
  make_log = strcmp(argv[1], "0"); 
  int init_failed = init();

  if (init_failed) return init_failed;
  cout << "Init: " << endl;
  while (!past_time(&curr_loop_time, &finish_time)) {

    main_loop();
  }

}

int init(void)
{
  gpioCfgClock(CLK_MICROS, 1, 0); 
  if (gpioInitialise() < 0) { 
    cout << "Failed GPIO Init" << endl;
    return 1;
  }
  init_exit_handler();
  vector<LoopState> history;
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

  /** 
  * The asked for frequency isn't necessarily the set one
  * because of sample rate considerations
  */
  double loop_wait_time_sec = (1.0 / SAMPLING_FREQ_HZ);
  // Time to wait per iteration
  loop_wait_time_nsec = static_cast<long>(
      round(loop_wait_time_sec * 1E9));
  // Print out ever kth iteration 
  print_loop_freq_iters = static_cast<long>(
      round(1.0 / (PRINT_FREQ_HZ * loop_wait_time_sec)));

  adc_loop_freq_iters = static_cast<long>(
      round(1.0 / (ADC_FREQ_HZ * loop_wait_time_sec)));
 
  cout << "Sampling freq HZ and period nsec: " <<
    SAMPLING_FREQ_HZ << "\t"  << loop_wait_time_nsec << endl;
  cout << "Print freq HZ and num iters: " << 
    PRINT_FREQ_HZ << "\t" << print_loop_freq_iters << endl;
  cout << "ADC freq and num iters: " << 
    (1.0 / (adc_loop_freq_iters * loop_wait_time_sec)) << ", " 
    << adc_loop_freq_iters << endl;
  
  clock_gettime(CLOCK_REALTIME, &curr_time);
  clock_gettime(CLOCK_REALTIME, &start_time);
  add_time(&start_time, RUN_FOR_TIME_SEC, 0, &finish_time);
  clock_gettime(CLOCK_REALTIME, &curr_loop_time);
  add_time(&curr_loop_time, 0, loop_wait_time_nsec, &next_loop_time); 
 
  // Wait a millisecond, you deserve a break
  std::chrono::microseconds timespan(1000);
  std::this_thread::sleep_for(timespan);

  cout << "Entering Inner Loop " << endl;
  return 0;
}

void print_state(void)
{
  cout << "\nCounts:"  << sm_encoder_count << "\t " << lm_encoder_count <<  
        "\nADC2:\t" <<
            channel_0[0] << "\t" << channel_1[0] << "\nADC2:\t" << 
            channel_0[1] << "\t" << channel_1[1] << "\nADC3:\t" << 
            channel_0[2] << "\t" << channel_1[2] << 
      "\nLinear Actu:\t" << raw_current_readings[0] << "\t" << amp_current_readings[0] << 
      "\nSmall Motor:\t" << raw_current_readings[1] << "\t" << amp_current_readings[1] << 
      "\nlarge motor:\t" << raw_current_readings[2] << "\t" << amp_current_readings[2] << endl;
}

// The main program!
int main_loop(void)
{
  num_iters++;
  // Do current calculations

  if ((num_iters % adc_loop_freq_iters) == 0) {
    last_readings(adc_reader, channel_0, channel_1);
    current_calculations(channel_0, channel_1, 
        raw_current_readings, amp_current_readings, use_amp_readings);
  }
  // TODO velocity estimation
  // Set pwms
  
      // Print if its time to do so
  if ((num_iters % print_loop_freq_iters) == 0) {
    print_state();
  }


  // Log if needed
  if (make_log) add_loop_state(history);

  // Loop timing code
  do{
    clock_gettime(CLOCK_REALTIME, &curr_time);
  } while(!past_time(&curr_time, &next_loop_time));
    
  add_time(&next_loop_time, 0, loop_wait_time_nsec, &next_loop_time);
  curr_loop_time = curr_time;
  return 0;
}


// Returns 0 if okay
int init_motors(void)
{
  //int motor_1_pwm = 0; //gpioHardwarePWM(MOTOR_1_PWM, MOTOR_1_FREQ, round(MOTOR_1_RANGE * 0.5));
  // The large motor
  int motor_1_freq = gpioSetPWMfrequency(MOTOR_1_PWM, MOTOR_1_FREQ);
  int motor_1_range = gpioSetPWMrange(MOTOR_1_PWM, MOTOR_1_RANGE);
  int motor_1_pwm = gpioPWM(MOTOR_1_PWM, 0); // round(MOTOR_1_RANGE * 0.40));
  gpioSetMode(MOTOR_1_DIR, PI_OUTPUT);
  gpioWrite(MOTOR_1_DIR, 1); 

  int motor_2_freq = gpioSetPWMfrequency(MOTOR_2_PWM, MOTOR_2_FREQ);
  int motor_2_range = gpioSetPWMrange(MOTOR_2_PWM, MOTOR_2_RANGE);
  int motor_2_pwm = gpioPWM(MOTOR_2_PWM, 0); // round(MOTOR_2_RANGE * 0.6));
  gpioSetMode(MOTOR_2_DIR, PI_OUTPUT);
  gpioWrite(MOTOR_2_DIR, 1);

  int lin_act_freq = gpioSetPWMfrequency(LIN_ACT_PWM, LIN_ACT_FREQ);
  int lin_act_range = gpioSetPWMrange(LIN_ACT_PWM, LIN_ACT_RANGE);
  int lin_act_pwm = gpioPWM(LIN_ACT_PWM, 0); //round(LIN_ACT_RANGE * 0.8));
  gpioSetMode(LIN_ACT_DIR, PI_OUTPUT);
  gpioWrite(LIN_ACT_DIR, 1);

  cout << "ranges: " << motor_2_range << ", " << lin_act_pwm << endl;
  return motor_1_freq && motor_1_range && motor_1_pwm &&
        motor_2_freq && motor_2_range && motor_2_pwm && 
        lin_act_freq && lin_act_range && lin_act_pwm;
}

// Simple Encoder Callback
// Timestamping also an option
// This should all be C++
void encoder_callback_1(int dir)
{
  sm_encoder_count += dir;
}

void encoder_callback_2(int dir)
{
  lm_encoder_count += dir;
}

void init_encoders()
{
  // Encoder state and initalization
  rot_encoder_1 = Pi_Renc(ENCODER_1_A, ENCODER_1_B, encoder_callback_1);
  
  rot_encoder_2 = Pi_Renc(ENCODER_2_A, ENCODER_2_B, encoder_callback_2);
}

// Calculates current readings for each actuator
void current_calculations(
    int* channel_0_in,
    int* channel_1_in,
    double* raw_current_out,
    double* amplified_current_out,
    bool*  use_amped_est)
{
  const double MPC_D_TO_V = 3.3 / 4095.0; // 12 BITS
  const double VNH5019_V_TO_A = 1.0 / .140; // 140 mV/A when driving
  // Linear Actuator Calculations
  int la_raw_d = channel_1_in[0]; 
  int la_amp_d = channel_0_in[0];
  double la_raw_v = la_raw_d * MPC_D_TO_V;
  double la_amp_v = la_amp_d * MPC_D_TO_V;
  // Voltage to current from specs
  double la_raw_a = la_raw_v * VNH5019_V_TO_A;  
  // Fix the amplified estimate to find v_in
  double la_fixed_amped_v = la_amp_v / LIN_ACT_AMP; 
  // (1 + BOARD_4_R7 / BOARD_4_R6);
  double la_amp_a = la_fixed_amped_v * VNH5019_V_TO_A;  
  raw_current_out[0] = la_raw_a;
  amplified_current_out[0] = la_amp_a;
  use_amped_est[0] = la_amp_v < 3.0 && la_raw_v < 0.5;


  // Small Motor Calculations
  int sm_raw_d = channel_0_in[1];  
  int sm_amp_d = channel_1_in[1]; 
   // Read Voltages
  double sm_raw_v = sm_raw_d * MPC_D_TO_V;
  double sm_amp_v = sm_amp_d * MPC_D_TO_V;
  // Voltage to current from specs
  double sm_raw_a = sm_raw_v * VNH5019_V_TO_A;  
  // Fix the amplified estimate to find v_in
  double sm_fixed_amped_v = sm_amp_v / MOTOR_2_AMP; 
  // (1 + BOARD_2_R2 / BOARD_2_R1);
  double sm_amp_a = sm_fixed_amped_v * VNH5019_V_TO_A;  
  raw_current_out[1] = sm_raw_a;
  amplified_current_out[1] = sm_amp_a;
  use_amped_est[1] = sm_amp_v < 3.0 && sm_raw_v < 0.5;


  // Large Motor Calculations
  const double ACS709_V_TO_A = 1.0 / 0.0185; // 18.5mV/A at 3.3V
  // Digital estimates
  int lm_raw_d = channel_1_in[2]; 
  int lm_amp_d = channel_0_in[2];
  // Read Voltages
  double lm_raw_v = lm_raw_d * MPC_D_TO_V;
  double lm_amp_v = lm_amp_d * MPC_D_TO_V;
  // Voltage to current from specs
  double lm_raw_a = (lm_raw_v - MOTOR_1_NO_CURRENT_V) * ACS709_V_TO_A;  
  
  // Fix the amplified estimate to find v_in
  const double a_vcc = MOTOR_1_OFFSET_V; 
  //3.3 * BOARD_4_R5 / (BOARD_4_R5 + BOARD_4_R4);
  double lm_fixed_amped_v = (lm_amp_v / MOTOR_1_AMP) - a_vcc; 
  // BOARD_4_R1 /BOARD_4_R3 + a_vcc;
  double lm_amp_a = (lm_fixed_amped_v - MOTOR_1_NO_CURRENT_V) * ACS709_V_TO_A;

  raw_current_out[2] = lm_raw_a;
  amplified_current_out[2] = lm_amp_a;
  use_amped_est[2] = lm_amp_v > 0.3 && lm_amp_v < 3.0; 
}


float get_la_dc(float desired_current_a)
{
  float la_last_current = use_amp_readings[0] ? amp_current_readings[0] : raw_current_readings[0];  
  float error = desired_current_a - la_last_current;
  float last_dc = pwm_commands[0];
  return fmax(0.0, fmin(1.0, last_dc + error * LIN_ACT_KP_A)); 
}

/** Sets the duty cycle for the linear actuator 
 * If the current is too high, slower it down
 */ 
int set_la_dc(float dc)
{
// TODO add iteration check
  int lin_act_pwm = gpioPWM(LIN_ACT_PWM, round(LIN_ACT_RANGE * abs(dc)));
  int lin_act_dir = gpioWrite(LIN_ACT_DIR, dc * LIN_ACT_FLIP_DIR > 0);
  pwm_commands[0] = dc;
  return lin_act_pwm || lin_act_dir; 
}

/** Sets the duty cycle for the small motor
 *
 */
int set_lm_dc(float dc)
{
  int lm_pwm = gpioPWM(MOTOR_1_PWM, round(MOTOR_1_RANGE * abs(dc)));
  int lm_dir = gpioWrite(MOTOR_1_DIR, dc * MOTOR_1_FLIP_DIR > 0);
  pwm_commands[2] = dc;
  return lm_pwm || lm_dir;
}

// You get it
int set_sm_dc(float dc)
{
  int sm_pwm = gpioPWM(MOTOR_2_PWM, round(MOTOR_2_RANGE * abs(dc)));
  int sm_dir = gpioWrite(MOTOR_2_DIR, dc * MOTOR_2_FLIP_DIR > 0);
  pwm_commands[1] = dc;
  return sm_pwm || sm_dir;
}

void add_loop_state(vector<LoopState>& history) 
{
  LoopState ls;
  ls.loop_time = curr_loop_time;
  ls.loop_count = num_iters;

  ls.la_raw_adc = channel_1[0];
  ls.la_amp_adc = channel_0[0];
  ls.sm_raw_adc = channel_0[1];
  ls.sm_amp_adc = channel_1[1];
  ls.lm_raw_adc = channel_1[2];
  ls.lm_amp_adc = channel_0[2];

  ls.la_current = use_amp_readings[0] ? amp_current_readings[0] : raw_current_readings[0];
  ls.sm_current = use_amp_readings[1] ? amp_current_readings[1] : raw_current_readings[1];
  ls.lm_current = use_amp_readings[2] ? amp_current_readings[2] : raw_current_readings[2];
  
  ls.sm_count = sm_encoder_count;
  ls.lm_count = lm_encoder_count;
 
  ls.sm_pos_r = -1;
  ls.sm_vel_est_rs = -1;

  ls.lm_pos_r = -1;
  ls.lm_vel_est_rs = -1;
  // Duty cycles in percents
  ls.la_duty_cycle = pwm_commands[0];
  ls.sm_duty_cycle = pwm_commands[1];
  ls.lm_duty_cycle = pwm_commands[2];

  history.push_back(ls);
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
  free_adc_reader(adc_reader);
  if (s == 0) {
    return;
  }
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


// Logic at the bottom
void update_slack_count(void)
{
  sm_theta = 2 * M_PI * sm_encoder_count / ENCODER_1_CPR;
  lm_theta = 2 * M_PI * lm_encoder_count / ENCODER_2_CPR;

  // Modeled the spool as having a decreasing radius over time
  rope_out = SPOOL_R0 / SPOOL_A * (1.0 - exp(-1 * SPOOL_A * lm_theta));
 
  // How much rope was fed off the spool
  double spool_rope_delta = rope_out - prev_rope_out;
  // How much rope was fed through the upper pulley
  double up_rope_delta = UP_RADIUS * (sm_theta - prev_sm_theta);

  // Slack added between them
  double slack_delta = spool_rope_delta - up_rope_delta;
 
  // Update slack measurement;
  up_slack += slack_delta;

  prev_rope_out = rope_out;
  prev_sm_theta = sm_theta;
}

