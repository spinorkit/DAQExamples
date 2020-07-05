#undef HID_ENABLED
// Arduino DUE ADC->DMA->USB 1MSPS
// Input: Analog in A0
// Output: Raw stream of uint15_t in range 0-4095 on Native SerialUSB/ACM
unsigned long sampleSize, st, et;
volatile int bufn, obufn;
const int kBufSize = 256;
uint16_t buf[4][kBufSize];   // 4 buffers of 256 readings

// void ADC_Handler() {
//   int f = ADC->ADC_ISR;
//   if (f & (1 << 27)) {
//     bufn = (bufn + 1) & 3;
//     ADC->ADC_RNPR = (uint32_t)buf[(bufn + 1) & 3];
//     ADC->ADC_RNCR = 256;
//   }
// }

//I think there is an error in ADC_Handler where the buffer pointers are updated. By the 
//time the handler is called, the DMA is already filling the "next" buffer = bufn+1. 
//So not only must bufn be incremented, but the "next" buffer should then be bufn+2. 
//I think the updates work better like this:
void ADC_Handler(){ // move DMA pointers to next buffer
int f=ADC->ADC_ISR;
if (f&(1<<27))
   {
   bufn=(bufn+1)&3;
   ADC->ADC_RNPR=(uint32_t)buf[(bufn+1)&3];
   ADC->ADC_RNCR=kBufSize;
   }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  SerialUSB.begin(0);
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  //ADC->ADC_MR |= 0x80;  // free running, 1000 0000
  ADC->ADC_MR |= 0x1480;  // prescal = 20 => 20Mhz ADC clk,  + free running
  ADC->ADC_CHER = 0x80; // enable ch's

  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_IDR = ~(1 << 27);
  ADC->ADC_IER = 1 << 27;
  //ADC->ADC_EMR = 0x01000000;
  ADC->ADC_RPR = (uint32_t)buf[0];  // DMA buffer
  ADC->ADC_RCR = 256;
  ADC->ADC_RNPR = (uint32_t)buf[1]; // next DMA buffer
  ADC->ADC_RNCR = 256;
  bufn = 1;
  obufn = 0;
  ADC->ADC_PTCR = 1;
  ADC->ADC_CR = 2;

}

void loop() {

  while (!SerialUSB.available()) {
    digitalWrite(LED_BUILTIN, HIGH); delay(25);
    digitalWrite(LED_BUILTIN, LOW); delay(225);
  }
  
  sampleSize = SerialUSB.parseInt();
  if (sampleSize > 1) {
    st = micros();
    for (int i = 0; i < sampleSize; i++) {
      while ((obufn + 1) % 4 == bufn); // wait for buffer to be full
      //SerialUSB.print(obufn); SerialUSB.print(" "); SerialUSB.println(bufn);
      SerialUSB.write((uint8_t *)buf[obufn], 512); // send it - 512 bytes = 256 uint16_t
      obufn = (obufn + 1) & 3;
    }
    et = micros();
    SerialUSB.println();
    SerialUSB.println("#");
    SerialUSB.println(et - st);
    SerialUSB.print( (sampleSize * 256 * 1000.0) / (et - st)); SerialUSB.println(" kHz");
  }
  sampleSize = 0;
}