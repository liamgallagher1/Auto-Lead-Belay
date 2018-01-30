/**
 * ADC Code for MCP3202 
 *
 * Modifed from the pigpio example code
 *
 * @author Liam Gallagher
 *
 * Modified by Liam Gallagher liam.gallagher0@gmail.com in January 2018
 */

// Creates an ADC reader to reach from both D1 and D2 pins of an MCP3202
// Theres probably a better way
// TODO should extend to multiple ADCS


typedef struct ADC_Reader {
  // GPIO Pins
  int slave_select_pin;
  // input(s)
  int* miso_pins;
  int num_inputs;
  // output
  int mosi_pin;
  // clock
  int clock_pin;
  // Raw SPI pointer needed for pigpio
  rawSPI_t rawSPI;
  // Wave Info
  rawWaveInfo_t rwi;

  int topOOL;
  // Why do I do this though
  float cbs_per_reading;

  // Wave IDs
  int wave_id_1;
  int last_val_1;

  int wave_id_2;
  int last_val_2;
} ADC_Reader;


// Constructs an ADC reader
// Returns null if theres an error
struct ADC_Reader* init_adc_reader(
  int slace_select_pin,
  int *miso_pins,
  int num_inputs,
  int mosi_pin,
  int clock_pin);

// Frees reader resources.
// don't not use reader after this
// NOTE termindates GPIO as well
void free_adc_reader(
  ADC_Reader* reader);



void last_readings(
  ADC_Reader* reader,
  int* raw_reading,
  int* amplified_reading);



