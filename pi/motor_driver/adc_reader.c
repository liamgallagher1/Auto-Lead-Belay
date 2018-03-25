#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <pigpio.h>

#include "adc_reader.h"
            
static int BUFFER = 250;        // "Generally make as large as possible"?

static int BITS = 12;           // Bits per reading.
static int BX = 6;              // Bit position of data bit B11
static int B0 = 17;             // Bit position of data bit B0

// Reading every number of microseconds
// Leads to 1000 Hz sampling
static int REPEAT_MICROS = 50; 

struct ADC_Reader* init_adc_reader(
    int slave_select_pin,
    int* miso_pins,
    int num_inputs,
    int mosi_pin,
    int clock_pin)
{
  ADC_Reader* reader = (struct ADC_Reader*)  calloc(sizeof(struct ADC_Reader), 1);
  reader->slave_select_pin = slave_select_pin;
  reader->miso_pins = miso_pins;
  reader->num_inputs = num_inputs;
  reader->mosi_pin = clock_pin;
  reader->clock_pin = clock_pin;

  // Create raw spi type
  reader->rawSPI.clk     =  clock_pin; // GPIO for SPI clock.
  reader->rawSPI.mosi    =  mosi_pin; // GPIO for SPI MOSI.
  reader->rawSPI.ss_pol  =  1; // Slave select resting level.
  reader->rawSPI.ss_us   =  1; // Wait 1 micro after asserting slave select.
  reader->rawSPI.clk_pol =  0; // Clock resting level.
  reader->rawSPI.clk_pha =  0; // 0 sample on first edge, 1 sample on second edge.
  reader->rawSPI.clk_us  =  1; // 2 clocks needed per bit so 500 kbps.

  if (gpioInitialise() < 0) {
    printf("Gpio init failed\n");
    free_adc_reader(reader);
    return 0;
  }
  gpioSetMode(clock_pin, PI_OUTPUT);
  gpioSetMode(mosi_pin, PI_OUTPUT);
  gpioSetMode(slave_select_pin, PI_OUTPUT);
  // Flush any old wave data?
  gpioWaveAddNew();


  int offset = 0;
  /*
   MCP3202 12-bit ADC 2 channels

   1  2  3  4  5  6   7   8  9  10 11 12 13 14 15 16 17
   SB SD OS MS NA B11 B10 B9 B8 B7 B6 B5 B4 B3 B2 B1 B0

   SB  1  1
   SD  1  0=differential 1=single
   OS  0  0=ch0, 1=ch1 (in single mode)
   MS  0  0=tx lsb first after tx msb first
   */
  // Construct multiple SPI READS, alternating input source
  char buff[2];
  buff[0] = 0xCF; // Single Ended, Channel 0
  buff[1] = 0xEF; // Single Ended, Channel 1
 
  for (int i = 0; i < BUFFER; ++i) {
    // For odd i read from Channel 1, for even i read from channel 0
    if (i % 2) {
      rawWaveAddSPI(&reader->rawSPI, offset, slave_select_pin, &buff[0], 8, BX, B0, B0);
    } else {
      rawWaveAddSPI(&reader->rawSPI, offset, slave_select_pin, &buff[1], 8, BX, B0, B0);
    }
    // Wait for longer than the time to transmit the message?
    offset += REPEAT_MICROS;
  }
  // Create gentle finish before it repeats
  gpioPulse_t final[2];
  final[0].gpioOn = 0;
  final[0].gpioOff = 0;
  final[0].usDelay = offset;
  final[1].gpioOn = 0; // Need a dummy to force the final delay.
  final[1].gpioOff = 0;
  final[1].usDelay = 0;
  gpioWaveAddGeneric(2, final);
  int wid = gpioWaveCreate(); // Create the wave from added data.

  if (wid < 0){
    printf("Can't create wave, %d too many?\n", BUFFER);
    free_adc_reader(reader);
    return 0;
  }
  
  /**
   * The wave resources are now assigned,  Get the number
   *  of control blocks (CBs) so we can calculate which reading
   *  is current when the program is running.
   */
  rawWaveInfo_t rwi = rawWaveInfo(wid);
  reader->rwi = rwi;

  printf("# cb %d-%d ool %d-%d del=%d ncb=%d nb=%d nt=%d\n",
      rwi.botCB, rwi.topCB, rwi.botOOL, rwi.topOOL, rwi.deleted,
      rwi.numCB,  rwi.numBOOL,  rwi.numTOOL);
  /**
   * CBs are allocated from the bottom up.  As the wave is being
   * transmitted the current CB will be between botCB and topCB
   * inclusive.
   */
  int botCB = rwi.botCB;
   /**
   * Assume each reading uses the same number of CBs (which is
   * true in this particular example).                        
   */
  // TODO understand what this float means.                                                       
  float cbs_per_reading = floor((float)rwi.numCB / (float)BUFFER);
  reader->cbs_per_reading = cbs_per_reading;
  printf("# cbs=%d per read=%.1f base=%d\n",
      rwi.numCB, cbs_per_reading, botCB);


  /**
  * OOL are allocated from the top down. There are BITS bits
  * for each ADC reading and BUFFER ADC readings.  The readings
  * will be stored in topOOL - 1 to topOOL - (BITS * BUFFER).
  */
  reader->topOOL = rwi.topOOL;

  // Send wave to pigpio, set to repeat
  gpioWaveTxSend(wid, PI_WAVE_MODE_REPEAT);

  return reader;
}

// Frees resources allocated to the reader
// Terminates the gpio as well. unclear if this is wise.
void free_adc_reader(
  ADC_Reader* reader)
{
  if (!reader) return;
  free(reader);
  gpioTerminate();
}

/*
 * This function extracts the MISO bits for each ADC and
 * collates them into a reading per ADC.
 */
void get_reading(
  ADC_Reader* reader, // The reader struct
  int OOL,   // Address of first OOL for this reading.
  int bytes, // Bytes between readings.
  int bits,  // Bits per reading.
  char *buf) // Output 
{
  uint32_t level;

  int p = OOL;

  for (int i = 0; i < bits; i++) {
    level = rawWaveGetOut(p);
    // TODO understand this
    int a = 0; // This can go unless we use more adcs
    putBitInBytes(i, buf+(bytes*a), level & (1<<(reader->miso_pins[0])));
    p--;
  }
}



void last_readings(
  ADC_Reader* reader,
  int* channel_0,
  int* channel_1)
{
  // Control block for current reading
  int cb = rawWaveCB() - reader->rwi.botCB;  
 
  // Which "reading #" is this?
  int now_reading = (int) round((float) cb / reader->cbs_per_reading);
  // Go to two readings previous, I think
  now_reading = (now_reading + BUFFER - 2) % BUFFER;
  // Raw data to output readings too 
  char rx[8];

  // get reading from either CH1 or CH2
  int OOL = reader->topOOL - ((now_reading % BUFFER) * BITS) - 1;
  // Black magic that may or may not work
  get_reading(reader, OOL, 2, BITS, rx);
  int i = 0;
  // Pull the values from that
  int val1 = (rx[i*2]<<4) + (rx[(i*2)+1]>>4);
  
  // now get reading from the other one 
  now_reading++;
  OOL = reader->topOOL - ((now_reading % BUFFER) * BITS) - 1;
  // Black magic that may or may not work
  get_reading(reader, OOL, 2, BITS, rx);
  // Pull the values from that
  int val2 = (rx[i*2]<<4) + (rx[(i*2)+1]>>4);
 
  int is_cha_0 = (OOL / BITS + 1) % 2;  
  // Currently just use the lesser one to be non amplified
  if (is_cha_0) {
    *channel_0 = val1;
    *channel_1 = val2;
  } else {
    *channel_0  = val2;
    *channel_1= val1;
  }
  return;
}

