#include "src/Adafruit_ZeroTimer.h"

//Adafruit_ZeroTimer zt4 = Adafruit_ZeroTimer(4);
Adafruit_ZeroTimer adcTimer(4);


/* Valid PWM outs:

FOR SAMD21:
  Timer3: channel 0 on D2 or D10, channel 1 on D5 or D12
  Timer4: channel 0 on SDA or A1, channel 2 on SCL or A2
  Timer5: channel 0 on MOSI, channel 1 on SCK

FOR SAMD51:
  Timer3: channel 0 on D10 or MISO, channel 1 on D11
  Timer4: channel 0 on A4, D7, or D1, channel 2 on A5, D4, or D0
  Timer5: channel 0 on D5, channel 1 on D6
*/

#if defined(__SAMD51__)
#define TIMER3_OUT0 10
#define TIMER3_OUT1 11

#define TIMER4_OUT0 A4
#define TIMER4_OUT1 A5

#define TIMER5_OUT1 6
#else
#define TIMER3_OUT0 10
#define TIMER3_OUT1 12

#define TIMER4_OUT0 A1
#define TIMER4_OUT1 A2

#define TIMER5_OUT1 SCK
#endif

const int kADCPointsPerSec = 4;

const int kADCStartChan = 2; //A2
const int kADCChannels = 2; 

const int kADCEndChan = kADCStartChan + kADCChannels;

volatile int gChan = 0;

inline void syncADC() 
{
  while (ADC->STATUS.bit.SYNCBUSY);
}

volatile bool gADCstate = false;


void setup() {

pinMode(6, OUTPUT);
   pinMode(LED_BUILTIN, OUTPUT);

//Setup event system so TC4 triggers ADC conversion start
PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;                                  // Switch on the event system peripheral

while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable the generic clock...
                      GCLK_CLKCTRL_GEN_GCLK0 |    // On GCLK0 at 48MHz
                      GCLK_CLKCTRL_ID( GCM_EVSYS_CHANNEL_0 );    // Route GCLK0 to EVENT channle

while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                               // Attach the event user (receiver) to channel 0 (n + 1)
                  EVSYS_USER_USER(EVSYS_ID_USER_ADC_START);             // Set the event user (receiver) as ADC START

EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |               // No event edge detection
                     EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                  // Set event path as asynchronous
                     EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC4_MCX_0) |      // Set event generator (sender) as TC4 Match/Capture 0
                     EVSYS_CHANNEL_CHANNEL(0);                          // Attach the generator (sender) to channel 0                                 

//Setup ADC

analogReadResolution(12);
analogReference(AR_DEFAULT);

pinPeripheral(A2, PIO_ANALOG);
pinPeripheral(A3, PIO_ANALOG);

//ADC->INPUTCTRL.bit.INPUTOFFSET = kADCStartChan;
syncADC();
ADC->INPUTCTRL.bit.MUXPOS = kADCStartChan;
syncADC();
ADC->INPUTCTRL.bit.INPUTOFFSET = 0;
syncADC();
ADC->INPUTCTRL.bit.INPUTSCAN = 0;//kADCChannels-1;
syncADC();

gChan = 0;

//PM->APBCMASK.reg |= PM_APBCMASK_ADC; already done by wiring.c

//ADC->INPUTCTRL.reg
ADC->EVCTRL.reg = ADC_EVCTRL_STARTEI; //Start on event

ADC->INTENSET.reg = ADC_INTENSET_RESRDY; //Enable interrupt on result ready

syncADC();
ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
syncADC();

NVIC_EnableIRQ(ADC_IRQn);


  /********************* Timer #4 + #5, 32 bit, one PWM out */
  adcTimer.configure(TC_CLOCK_PRESCALER_DIV1, // prescaler
                TC_COUNTER_SIZE_32BIT,   // bit width of timer/counter
                TC_WAVE_GENERATION_MATCH_FREQ // frequency or PWM mode
                );
  //adcTimer.setPeriodMatch(1000, 200);      // channel 1 only, 200/1000 count
  adcTimer.setCompare(0, VARIANT_MCK/2/kADCPointsPerSec);
  if (! adcTimer.PWMout(true, 0, A1)) {
    Serial.println("Failed to configure PWM output");
  }

TC4->COUNT32.EVCTRL.reg |= TC_EVCTRL_MCEO0;
while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  adcTimer.enable(true);


   #if 0

   // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT8;           // Set the counter to 8-bit mode
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  REG_TC4_COUNT8_CC0 = 0x55;                      // Set the TC4 CC0 register to some arbitary value
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  REG_TC4_COUNT8_CC1 = 0xAA;                      // Set the TC4 CC1 register to some arbitary value
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  REG_TC4_COUNT8_PER = 0xFF;                      // Set the PER (period) register to its maximum value
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  //NVIC_DisableIRQ(TC4_IRQn);
  //NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 | TC_INTFLAG_OVF;        // Clear the interrupt flags
  REG_TC4_INTENSET = TC_INTENSET_MC1 | TC_INTENSET_MC0 | TC_INTENSET_OVF;     // Enable TC4 interrupts
  // REG_TC4_INTENCLR = TC_INTENCLR_MC1 | TC_INTENCLR_MC0 | TC_INTENCLR_OVF;     // Disable TC4 interrupts
 
  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV64 |     // Set prescaler to 64, 16MHz/64 = 256kHz
                   TC_CTRLA_ENABLE;               // Enable TC4
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

#endif

}



void loop() {
  // put your main code here, to run repeatedly:
  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(100);
  //digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(100);
//gADCstate = !gADCstate;
//digitalWrite(LED_BUILTIN, gADCstate = !gADCstate);  
//digitalWrite(6, gADCstate = !gADCstate);  
}


void ADC_Handler(void)
{
//    LED1_TOGGLE;//to observe interrupt routine (one times only)
    
    //clearing interrupt flag
    //REG_ADC_INTFLAG=ADC_INTFLAG_RESRDY;
//ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; but lets just read the result instead!
int val = ADC->RESULT.reg;
//while (ADC->STATUS.bit.SYNCBUSY == 1);  
// syncADC();
// ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; //Need to reset interrupt

// //ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
syncADC();
int chan = ADC->INPUTCTRL.bit.MUXPOS;
syncADC();

//gRingBuffer[chan-kADCStartChan].push(val);

syncADC();
if(++chan < kADCEndChan)
   {
   ADC->INPUTCTRL.bit.MUXPOS = chan;      
   syncADC();
   // ADC->EVCTRL.reg = 0; //not start on event
   // syncADC();

   ADC->SWTRIG.bit.START = 1;
   }
else
   {
   ADC->INPUTCTRL.bit.MUXPOS = kADCStartChan;
   
   // syncADC();
   // ADC->EVCTRL.reg = ADC_EVCTRL_STARTEI; //Start on event

   }

syncADC();
digitalWrite(6, gADCstate = !gADCstate );  


// syncADC();
// int offset = ADC->INPUTCTRL.bit.INPUTOFFSET;
// syncADC();

// if(gChan >= kADCChannels-1)
//    {
//    gChan = 0;
//    syncADC();
//    ADC->CTRLB.bit.FREERUN = 0;

//    syncADC();
//    ADC->EVCTRL.reg = ADC_EVCTRL_STARTEI; //Start on event

//    syncADC();
//    ADC->INPUTCTRL.bit.INPUTOFFSET = 0;   

//    //syncADC();
//    //digitalWrite(6, gADCstate = !gADCstate );  

//    }
// else if(gChan == 0)
//    {
//    syncADC();
//    ADC->EVCTRL.reg = 0; //not start on event

//    syncADC();
//    ADC->CTRLB.bit.FREERUN = 1;

//    syncADC();
//    digitalWrite(6, gADCstate = !gADCstate );  

//    }
// syncADC();
// ++gChan;

//digitalWrite(LED_BUILTIN, gADCstate = !gADCstate);  
}


#if 0
void TC4_Handler()                              // Interrupt Service Routine (ISR) for timer TC4
{     
  // Check for overflow (OVF) interrupt
  if (TC4->COUNT8.INTFLAG.bit.OVF && TC4->COUNT8.INTENSET.bit.OVF)             
  {
    // Put your timer overflow (OVF) code here:     
    // ...
   
    REG_TC4_INTFLAG = TC_INTFLAG_OVF;         // Clear the OVF interrupt flag
  }

  // Check for match counter 0 (MC0) interrupt
  if (TC4->COUNT8.INTFLAG.bit.MC0 && TC4->COUNT8.INTENSET.bit.MC0)             
  {
    // Put your counter compare 0 (CC0) code here:
    // ...
   
    REG_TC4_INTFLAG = TC_INTFLAG_MC0;         // Clear the MC0 interrupt flag
  }

  // Check for match counter 1 (MC1) interrupt
  if (TC4->COUNT8.INTFLAG.bit.MC1 && TC4->COUNT8.INTENSET.bit.MC1)           
  {
    // Put your counter compare 1 (CC1) code here:
    // ...
   
    REG_TC4_INTFLAG = TC_INTFLAG_MC1;        // Clear the MC1 interrupt flag
  }
}

#endif
