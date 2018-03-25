/**
 * ADC Code for MCP3202 
 *
 * Modifed from the pigpio example code
 *
 * @author Liam Gallagher
 *
 * Modified by Liam Gallagher liam.gallagher0@gmail.com in January 2018
 */

// Creates an ADC reader to read from both D1 and D2 pins of an MCP3202
// Theres probably a better way to do this
// TODO actually add the ability to do this from multiple inputs

// Persistent data for the reader
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


  // This state was copied from the old implementation.
  // I don't understand it or like their style.
  int topOOL;
  // Why do I do this though
  float cbs_per_reading;
} ADC_Reader;


// Constructs an ADC reader
// Returns null if theres an error
struct ADC_Reader* init_adc_reader(
  int slace_select_pin,
  int *miso_pins,
  int num_inputs,
  int mosi_pin,
  int clock_pin);

// Free's reader resources.
// Don't not use reader after this
// NOTE termindates GPIO as well
void free_adc_reader(
  ADC_Reader* reader);


// Gets the most recent readings from the ADC
// TODO currently assumes that the amplified value is greater
// TODO might currently add a delay
void last_readings(
  ADC_Reader* reader,
  int* channel_0,
  int* channel_1);
