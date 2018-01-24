

static int BUFFER = 250;


ADC_Reader* init_adc_reader(
    int slave_select_pin,
    int* miso_pins,
    int num_inputs,
    int mosi_pin,
    int clock_pin)
{
  ADC_Reader* reader = (ADC_Reader*) calloc(sizeof(ADC_Reader), 1);
  reader->slave_select_pin = slave_select_pin;
  reader->miso_pins = miso_pins;
  reader->num_inputs = num_inputs;
  reader->mosi_pin = clock_pin;
  reader->clock_pin = clock_pin;
 
  if (gpioInitalise() < 0) {
    printf("Gpio init failed\n");
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
  char buf[2];
  buff[0] = 0xC0; // Single Ended, Channel 0
  buff[1] = 0xE0; // Single Ended, Channel 1
 
  for (int i = 0; i < BUFFER; ++i) {
    // For odd i read from Channel 1, for even i read from channel 0
    if (i % 2) {
      rawWaveAddSPI(&rawSPI, offset, slave_select_pin, buff+1, 3, BX, B0, B0);
    } else {
      rawWaveAddSPI(&rawSPI, offset, slave_select_pin, buff, 3, BX, B0, B0);
    }
    // Wait for longer than the time to transmit the message?
    offset += REPEAT_MICROS;
  }
  // Create gentle finish (Why do we wait this long?)
  final[0].gpioOn = 0;
  final[0].gpioOff = 0;
  final[0].usDelay = offset;

  final[1].gpioOn = 0; // Need a dummy to force the final delay.
  final[1].gpioOff = 0;
  final[1].usDelay = 0;

  gpioWaveAddGeneric(2, final);

  wid = gpioWaveCreate(); // Create the wave from added data.

  if (wid < 0){
    printf("Can't create wave, %d too many?\n", BUFFER);
    return 1;
  }
  /*
      The wave resources are now assigned,  Get the number
      of control blocks (CBs) so we can calculate which reading
      is current when the program is running.
   */
  rwi = rawWaveInfo(wid);
  printf("# cb %d-%d ool %d-%d del=%d ncb=%d nb=%d nt=%d\n",
      rwi.botCB, rwi.topCB, rwi.botOOL, rwi.topOOL, rwi.deleted,
      rwi.numCB,  rwi.numBOOL,  rwi.numTOOL);
   /*
      CBs are allocated from the bottom up.  As the wave is being
      transmitted the current CB will be between botCB and topCB
      inclusive.
   */

   botCB = rwi.botCB;


}




