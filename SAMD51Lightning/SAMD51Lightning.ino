#include "src/Adafruit_ZeroTimer.h"

Adafruit_ZeroTimer adcTimer(4);

#ifdef TIMER_OUTPUT_FOR_TEST
Adafruit_ZeroTimer zt3(3); //Testing
#endif

/* Valid PWM outs (for Adafruit Feather ):

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

//#define ENABLE_ADCTIMER_PWMOUT 1
//#define TIMING_CHECK 1

#ifdef TIMING_CHECK
const int kDefaultADCPointsPerSec = 1;//1024;//100; //~5000 max with 2 samples (1 point) per packet
#else
const int kDefaultADCPointsPerSec = 100; //~5000 max with 2 samples (1 point) per packet
#endif

int gADCPointsPerSec = kDefaultADCPointsPerSec; //~5000 max with 2 samples (1 point) per packet

const int kNSampleRates = 6;
const int kSampleRates[] = {4000, 2000, 1000, 400, 250, 100};


const int kADCStartChan = 2; //A1

#ifdef TIMING_CHECK
const int kADCChannels = 1;//2; 
#else
const int kADCChannels = 2; 
#endif

const int kADCEndChan = kADCStartChan + kADCChannels;

 // =====================================================================
 //     CLOCK
 // ---------------------------------------------------------------------
 void setupPLL(uint16_t which, uint16_t PllFactFra, uint16_t PllFactInt)
 {
	 // ----------------------------------------------------------------------------------------
	 // ----------------------------------------------------------------------------------------
	 // PHASE LOCKED LOOP 0 to MCU Clock
	 // contrary to Start's approach, seems like we should get PLL humming before
	 // changing GCLK.
	 OSCCTRL->Dpll[which].DPLLCTRLA.bit.ENABLE = 0;
	 while (OSCCTRL->Dpll[which].DPLLSYNCBUSY.bit.ENABLE)
	 ;

	 OSCCTRL->Dpll[which].DPLLRATIO.reg = (PllFactFra<<16) + PllFactInt;  //fraction and int for 100MHz = 24, 0xbea
	 while (OSCCTRL->Dpll[which].DPLLSYNCBUSY.bit.DPLLRATIO)
	 ;

	 OSCCTRL->Dpll[which].DPLLCTRLB.bit.DIV = 0; // not meaningful for 32KHz
	 OSCCTRL->Dpll[which].DPLLCTRLB.bit.DCOEN = 0;
	 OSCCTRL->Dpll[which].DPLLCTRLB.bit.LBYPASS = 1;	// errata, slow reference clock need to bypass lock lost
	 OSCCTRL->Dpll[which].DPLLCTRLB.bit.LTIME = 0;
	 OSCCTRL->Dpll[which].DPLLCTRLB.bit.REFCLK = 1;	// reference clock is 32KHz external
	 OSCCTRL->Dpll[which].DPLLCTRLB.bit.WUF = 0;


	 OSCCTRL->Dpll[which].DPLLCTRLA.bit.ONDEMAND = 0;
	 OSCCTRL->Dpll[which].DPLLCTRLA.bit.RUNSTDBY = 1;
	 OSCCTRL->Dpll[which].DPLLCTRLA.bit.ENABLE = 1;
	 while (OSCCTRL->Dpll[which].DPLLSYNCBUSY.bit.ENABLE)
	 ;

	 while (! (OSCCTRL->INTFLAG.bit.DPLL0LDRTO || OSCCTRL->Dpll[which].DPLLSTATUS.bit.CLKRDY))
	 ;

 }

 void FatalError(const char *msg, uint32_t freq)
 {
 Serial.print(msg);
 Serial.println(freq);
 }

 uint32_t myMCU_CLOCK = 0;

 // =============================================================================
 // 32768 slow clock using external oscillator.
 // GCLK3 feeds 32768 so FLL could be used
 // FDPLL multiplies 32768 to PllFreq_Hz
 // GCLK0 divides Pll0Freq to McuFreq
 // GCLK1 divides Pll1Freq by 1
 //
 // Freed from Start, which means anything clocked needs to be.
 void setMyClocks(uint32_t PllFreq_Hz, uint32_t McuFreqHz)
 {
	 
	 // BUG in Start, PLL must be >96MHz but they let you configure 80 and even 20MHz...
	 // 54.12.5 Fractional Digital Phase Lock Loop (FDPLL) Characteristics
	 // Table 54-48. Fractional Digital Phase Lock Loop Characteristics
	 // Symbol  Min. Typ. Max. Units
	 // fIN     32    -   3200 kHz     using 32.768
	 // fOUT    96    -    200 MHz     using 120MHz (can post divide)
	 // yet the manual even gives a 48MHz example
	 
	 // If possible, we can use the post divider to get 376471 Hz
	 // using a legal PLL and post divider.  if illegal, trap
	 if (PllFreq_Hz < (96000254/255) || PllFreq_Hz > 200000000)
	    FatalError("ERR_CLOCK_PLL", PllFreq_Hz);
	 
	 uint32_t PllDivOut = 1, targetFreq = PllFreq_Hz;
	 
	 while (targetFreq < 96000000 && targetFreq <= 200000000 && PllDivOut < 256)
	 {
		 PllDivOut++;
		 targetFreq = PllFreq_Hz*PllDivOut;
	 }
	 
	 if (targetFreq > 200000000 || PllDivOut >= 256)
	   FatalError("ERR_CLOCK_TGT", targetFreq);
	 
	 
	 if (McuFreqHz > 120000000)
	 McuFreqHz = 120000000;
	 if (McuFreqHz > PllFreq_Hz)
	 McuFreqHz = PllFreq_Hz;
	 
	 myMCU_CLOCK = McuFreqHz;

	 uint32_t PllFactInt = targetFreq/32768 - 1;
	 uint32_t PllFactFra = (32*(targetFreq - 32768*(PllFactInt+1)))/32768;
	 
	 // ----------------------------------------------------------------------------------------
	 // WAIT STATES
	 // BUG in Start - lets you set clocks without auto or valid wait states.
	 // 54.11 conservatively for 1.8V
	 uint32_t waits = 0;
	 if (McuFreqHz >=  22000000) waits++;
	 if (McuFreqHz >=  44000000) waits++;
	 if (McuFreqHz >=  67000000) waits++;
	 if (McuFreqHz >=  89000000) waits++;
	 if (McuFreqHz >= 111000000) waits++;
	 if (McuFreqHz >= 120000000) waits++;
	 NVMCTRL->CTRLA.bit.RWS = waits; // allows 111MHz at 1.8V

	 // ----------------------------------------------------------------------------------------
	 // Now that we know we CAN do this, reset clocks
	 GCLK->CTRLA.bit.SWRST;
	 while (GCLK->SYNCBUSY.bit.SWRST)
	 ;

	 // ----------------------------------------------------------------------------------------
	 // MASTER CLOCK
	 // Master clock divide by 1.
	 MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1_Val;

	 
	 // ----------------------------------------------------------------------------------------
	 // ----------------------------------------------------------------------------------------
	 // EXTERNAL 32KHZ CLOCK
	 // To guarantee the XOSC32K behavior in crystal mode, PC00 must be static.
	 // PC00 is not pinned out in smaller chips
	 OSC32KCTRL->XOSC32K.bit.CGM = 01;		// normal drive
	 OSC32KCTRL->XOSC32K.bit.XTALEN = 0;     // using OSC, so SHOULDN'T run drive
	 OSC32KCTRL->XOSC32K.bit.EN32K = 1;      // seems needed and there is no gpio for this.  Internal signal?
	 OSC32KCTRL->XOSC32K.bit.ONDEMAND = 0;
	 OSC32KCTRL->XOSC32K.bit.RUNSTDBY = 1;	// always on
	 OSC32KCTRL->XOSC32K.bit.ENABLE = 1;		// enabled

	 OSC32KCTRL->CFDCTRL.bit.CFDEN = 0;
	 OSC32KCTRL->EVCTRL.bit.CFDEO = 0;

	 // no setting up PLL until there is a phase to lock a loop to
	 // FDPLL has this errata but it looks like osc32K does too,  use INTFLAG, not STATUS
	 while (!OSC32KCTRL->INTFLAG.bit.XOSC32KRDY)
	 ;

	 // RTCC from a real clock
	 OSC32KCTRL->RTCCTRL.bit.RTCSEL = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC32K_Val;
	 
	 // ----------------------------------------------------------------------------------------
	 // IF DEBUG, OUTPUT 32K CLOCK, either way generate it
	 // GCLK4 echoes OSC32K but can be used in other places.
	 // GCLK2, 3, and 6 do NOT seem to output properly
	 GCLK->GENCTRL[4].reg = (1 << 16) | (0x21 << 8) | 5; // 32K = 5, output, enabled and standby = 0x29, divide by 1
	 while (GCLK->SYNCBUSY.bit.GENCTRL4)
	 ;

	 setupPLL(0, PllFactFra, PllFactInt);
	 setupPLL(1, PllFactFra, PllFactInt); // initially 120MHz

	 // GCLK's to the real system.  0 is MCLK and 1 is ADCclk
	 // Cannot output these clocks
	 GCLK->GENCTRL[0].reg = (PllDivOut << 16) | (0x21 << 8) | 7; // dpll0 = 7, enabled and standby = 0x21, divide by 1
	 while (GCLK->SYNCBUSY.bit.GENCTRL0)
	 ;

	 // in debug, outputs on the switch.
	 GCLK->GENCTRL[1].reg = (PllDivOut << 16) | (0x21 << 8) | 8; // dpll1 = 8, enabled and standby = 0x21, divide by 100
	 while (GCLK->SYNCBUSY.bit.GENCTRL1)
	 ;
	 
	 // GCLK[3] and GCLK[6] do not seem to work as planned
 }


void debugNewLine()
{
//Serial.write('\n'); //Readability while testing only!
}


inline void syncADC0_ENABLE() 
{
  while (ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);
}

inline void syncADC0_CTRLB() 
{
  while (ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);
}

inline void syncADC0_SAMPCTRL() 
{
  while (ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL);
}

inline void syncADC0_INPUTCTRL() 
{
  while (ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL);
}

inline void syncADC0_SWTRIG() 
{
  while (ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_SWTRIG);
}

void startADCTimer(uint32_t frequency) 
{
/********************* Timer #4 + #5, 32 bit, one PWM out */
adcTimer.configure(TC_CLOCK_PRESCALER_DIV1, // prescaler
            TC_COUNTER_SIZE_32BIT,   // bit width of timer/counter
            TC_WAVE_GENERATION_MATCH_FREQ // frequency or PWM mode
            );
//adcTimer.setPeriodMatch(1000, 200);      // channel 1 only, 200/1000 count
//Adafruit timer routines set the timer source to GCLK1 (48 MHz)
adcTimer.setCompare(0, VARIANT_GCLK1_FREQ/frequency - 1);
#ifdef ENABLE_ADCTIMER_PWMOUT
//N.B. this will be at half the rate of the ADC (i.e. each edge triggers a set of conversions across channels)
if (! adcTimer.PWMout(true, 0, TIMER4_OUT0)) {
   Serial.println("Failed to configure PWM output");
}
#endif

TC4->COUNT32.EVCTRL.reg |= TC_EVCTRL_MCEO0;
while (TC4->COUNT32.SYNCBUSY.reg > 0);                // Wait for synchronization

//Setup Event system
MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;

  // Select the event system user on channel 0 (USER number = channel number + 1)
EVSYS->USER[EVSYS_ID_USER_ADC0_START].reg = EVSYS_USER_CHANNEL(1);         // Set the event user (receiver) as timer TC0

// EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |               // No event edge detection
//                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                  // Set event path as asynchronous
//                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC4_MCX_0) |      // Set event generator (sender) as TC4 Match/Capture 0
//                      EVSYS_CHANNEL_CHANNEL(0);                          // Attach the generator (sender) to channel 0                                 

EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |               // No event edge detection
                     EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                  // Set event path as asynchronous
                     EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC4_MCX_0);        // Set event generator (sender) as TC4 Match/Capture 0

adcTimer.enable(true);
}

void adc_setup()
{
   //Setup event system so TC4 triggers ADC conversion start
//PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;                                  // Switch on the event system peripheral

// while(GCLK->SYNCBUSY.reg & GCLK_STATUS_SYNCBUSY);
// GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable the generic clock...
//                       GCLK_CLKCTRL_GEN_GCLK0 |    // On GCLK0 at 48MHz
//                       GCLK_CLKCTRL_ID( GCM_EVSYS_CHANNEL_0 );    // Route GCLK0 to EVENT channel

// while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


// EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                               // Attach the event user (receiver) to channel 0 (n + 1)
//                   EVSYS_USER_USER(EVSYS_ID_USER_ADC_START);             // Set the event user (receiver) as ADC START

// EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |               // No event edge detection
//                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                  // Set event path as asynchronous
//                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC4_MCX_0) |      // Set event generator (sender) as TC4 Match/Capture 0
//                      EVSYS_CHANNEL_CHANNEL(0);                          // Attach the generator (sender) to channel 0                                 

//Setup ADC

analogReadResolution(12);
analogReference(AR_DEFAULT);

pinPeripheral(A1, PIO_ANALOG);
pinPeripheral(A2, PIO_ANALOG);

//ADC->INPUTCTRL.bit.INPUTOFFSET = kADCStartChan;
ADC0->INPUTCTRL.bit.MUXPOS = kADCStartChan;
syncADC0_INPUTCTRL();
// ADC0->INPUTCTRL.bit.INPUTOFFSET = 0;
// syncADC0_INPUTCTRL();
// ADC0->INPUTCTRL.bit.INPUTSCAN = 0;//kADCChannels-1;
// syncADC0_INPUTCTRL();


//PM->APBCMASK.reg |= PM_APBCMASK_ADC; already done by wiring.c

//ADC->INPUTCTRL.reg
ADC0->EVCTRL.reg = ADC_EVCTRL_STARTEI; //Start on event

ADC0->INTENSET.reg = ADC_INTENSET_RESRDY; //Enable interrupt on result ready

ADC0->CTRLA.bit.ENABLE = 1;             // Enable ADC
syncADC0_ENABLE();

//NVIC_SetPriority(ADC_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for ADC to 0 (highest)

NVIC_EnableIRQ(ADC0_1_IRQn);
}

template <class T, unsigned int Log2Size>
class RingBufferSized
   {
   public:
      enum
      {
      kBufSize = 1<<Log2Size,
      kLenMask = kBufSize-1,
      };

   RingBufferSized() : mIn(0),mOut(0)
      {
      }

   void Clear()
      {
      mOut = mIn;
      }

   int GetCount() const
   {
   return (mIn-mOut) & kLenMask;
   }

   int GetSpace() const
   {
   return kLenMask - GetCount();
   }

   bool Push(T val)
      {
      if(GetSpace())
         {
         mBuffer[mIn++] = val;
         mIn &= kLenMask;
         return true;
         }
      return false;
      }

   //Returns num pushed
   int Push(const T *val, int nToPushIn)
      {
      int nToPushRemain = nToPushIn;
      int space = GetSpace();

      if(nToPushRemain > space)
         nToPushRemain = space; //limit to available space
      else
         space = nToPushIn; //space is now number that will be pushed

      if(nToPushRemain)
         {//There is space
         int lenToCopy1 = (kBufSize-mIn); //space available before wrapping
         if(lenToCopy1 > nToPushRemain)
            lenToCopy1 = nToPushRemain;
         memcpy(mBuffer+mIn,val,lenToCopy1*sizeof(T));
         mIn += lenToCopy1;
         mIn &= kLenMask;
         nToPushRemain -= lenToCopy1;
         if(nToPushRemain)
            {//still some left to copy, wrap to start of buffer
            memcpy(mBuffer,val+lenToCopy1,nToPushRemain*sizeof(T));
            mIn += nToPushRemain;
            mIn &= kLenMask;
            }
         }
      return space; //Space is number pushed.
      }


   bool Get(T *val) const
      {
      if(GetCount())
         {
         *val = mBuffer[mOut];
         return true;
         }
      return false;
      }

   const T& Get() const
      {
      return mBuffer[mOut];
      }

   const T& GetNext()
      {
      const T& result = mBuffer[mOut++];
      mOut &= kLenMask;
      return result;
      }

   bool GetNext(T *val)
      {
      if(GetCount())
         {
         *val = mBuffer[mOut++];
         mOut &= kLenMask;
         return true;
         }
      return false;
      }

   bool NextOut()
      {
      if(GetCount())
         {
         mOut++;
         mOut &= kLenMask;
         return true;
         }
      return false;
      }

   protected:
   T mBuffer[kBufSize];
   volatile int mIn;
   volatile int mOut;
   };

const int kMaxCommandLenBytes = 64;

const int kBytesPerSample = sizeof(int16_t);

const int kLog2BufferPoints = 13;
const int kBufferSizeBytes = (1 << kLog2BufferPoints)*kADCChannels*kBytesPerSample;

const int kPointsPerPacket = 1;
const int kPointsPerMediumSizePacket = 10;

const int kLog2BufferSizeBytes = 15;
const int kLog2ADCChannels = 1;
const int kLog2BytesPerSample = 1;

int gADCPointsPerPacket = kPointsPerPacket;

typedef RingBufferSized<int16_t, kLog2BufferPoints> TRingBuf;

TRingBuf gSampleBuffers[kADCChannels];


volatile int32_t gFirstADCPointus = 0;


enum State
{
kIdle,
kStartingSampling,
kHadFirstSample,
kSampling,  
};

volatile State gState = kIdle;
volatile bool gFirstSampleTimeRequested = false;

volatile bool gADCstate = false;

void setup() 
{
Serial.begin (0);
while(!Serial);

pinMode(6, OUTPUT); //Test only - toggles on each ADC_Handler()
pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, LOW); 

#ifdef TIMER_OUTPUT_FOR_TEST
  /********************* Timer #3, 16 bit, one PWM out, period = 1024 */
  zt3.configure(TC_CLOCK_PRESCALER_DIV1, // prescaler
                TC_COUNTER_SIZE_16BIT,   // bit width of timer/counter
                TC_WAVE_GENERATION_MATCH_PWM // frequency or PWM mode
                );
  const uint32_t kTicks = VARIANT_GCLK1_FREQ/1024;
  zt3.setPeriodMatch(kTicks - 1, kTicks/4);      // channel 1 only, 200/1000 count
  if (! zt3.PWMout(true, 1, TIMER3_OUT1)) {
    Serial.println("Failed to configure PWM output");
  }

#ifdef _VARIANT_SAMD51_THING_PLUS_
//Sparkfun Thing Plus has different pin mapping from Adafruit Feather M4
PORT->Group[0].PINCFG[15].reg = PORT_PINCFG_PMUXEN; //PA15
PORT->Group[0].PMUX[7].reg &= ~(PORT_PMUX_PMUXO_Msk);
PORT->Group[0].PMUX[7].reg |= 0x04 << 4;//   PORT_PMUX_PMUXO_E;
#endif
//  PORT->Group[g_APinDescription[ulPin].ulPort].DIRSET.reg = (uint32_t)(1<<g_APinDescription[ulPin].ulPin) ;

  zt3.enable(true);
/*********************************************/

#endif


adc_setup();
startADCTimer(gADCPointsPerSec);
}

#ifdef TIMING_CHECK
volatile int32_t gLastADCus = 0;
int32_t gLastLastADCus = 0;
#endif

void ADC0_1_Handler()
{
#ifdef TIMING_CHECK
gLastADCus = micros();
#endif
digitalWrite(6, gADCstate = !gADCstate );  

int val = ADC0->RESULT.reg;

syncADC0_INPUTCTRL();
int chan = ADC0->INPUTCTRL.bit.MUXPOS;

if(!gSampleBuffers[chan-kADCStartChan].Push(val))
   digitalWrite(LED_BUILTIN, LOW); //Turn off LED to indicate overflow

if(chan == kADCStartChan && gState == kStartingSampling)
   {
   gFirstADCPointus = micros();
   gState = kHadFirstSample;   
   }

if(++chan < kADCEndChan)
   {
   ADC0->INPUTCTRL.bit.MUXPOS = chan;  
   syncADC0_INPUTCTRL();  
     
   ADC0->SWTRIG.bit.START = 1;
   syncADC0_SWTRIG();
   }
else
   {
   ADC0->INPUTCTRL.bit.MUXPOS = kADCStartChan;
   syncADC0_INPUTCTRL();
   }

//digitalWrite(6, gADCstate = !gADCstate );  
}

class PacketBase
{
protected:
   static uint8_t sPacketCount;  
};

uint8_t PacketBase::sPacketCount = 0;


class Packet : protected PacketBase
{
   //The header is 5 nibbles, i.e. "P\xA0\x40". The low nibble of the
   //3rd byte is the packet time (0x04) for data packets.
   //The head and packet type is followed by a 1 byte packet count number,
   //making a total of 4 bytes before the payload daya that need to match the 
   //expected pattern(s) before the client can detect a packet.
   const char sHeader[2] = {'P',0xA0}; //D for data

public:

   static void ResetPacketCount()
      {
      sPacketCount = 0;   
      }

   Packet() : mPoint(0)
      {
      }

   bool addSample(int chan, int16_t sample)
      {
      if(mPoint >= gADCPointsPerPacket)
         return false;
//Testing!!
//if(chan == 0)
//   mData[mPoint][chan] = 0;
//else
      mData[mPoint][chan] = (sample << 4) - 0x8000;

      return true;
      }

   void nextPoint()
      {
      ++mPoint;   
      }

   //returns number of bytes written
   int write(Stream &stream) const
      {
      int n = stream.write(sHeader, 2);
      //Write the packet type byte (D for data, M for medium sized data packet)
      n += stream.write(uint8_t(gADCPointsPerPacket==1?'D':'M'));
      n += stream.write(sPacketCount++);
      n += stream.write(reinterpret_cast<const uint8_t*>(mData), sizeof(int16_t)*kADCChannels*gADCPointsPerPacket);
      return n;
      }


protected:

   int mPoint;
   int16_t mData[kPointsPerMediumSizePacket][kADCChannels];

};



class TimePacket : protected PacketBase
{
   const char sHeaderAndPacketType[3] = {'P',0xA0,'N'}; //'N' for now

public:
   TimePacket(int32_t tick32us, uint8_t timeRequestNumber) :
      mTimeRequestNumber(timeRequestNumber)
      {
      mData[0] = tick32us;
      }

      //returns number of bytes written
   int write(Stream &stream) const
      {
      int n = stream.write(sHeaderAndPacketType, 3);
      n += stream.write(sPacketCount++);
      n += stream.write(mTimeRequestNumber);
      n += stream.write(reinterpret_cast<const uint8_t*>(mData), sizeof(mData));
      return n;
      }

protected:

   int32_t mData[1];
   uint8_t mTimeRequestNumber;
};

class FirstSampleTimePacket : protected PacketBase
{
   const char sHeaderAndPacketType[3] = {'P',0xA0,'F'}; //'F' for First sample time

public:
   FirstSampleTimePacket(int32_t tick32us)
      {
      mData[0] = tick32us;
      }

      //returns number of bytes written
   int write(Stream &stream) const
      {
      int n = stream.write(sHeaderAndPacketType, 3);
      n += stream.write(sPacketCount++);
      n += stream.write(reinterpret_cast<const uint8_t*>(mData), sizeof(mData));
      return n;
      }

protected:

   int32_t mData[1];
};


void StartSampling()
{
adcTimer.enable(false);
NVIC_DisableIRQ(ADC0_1_IRQn);
NVIC_ClearPendingIRQ(ADC0_1_IRQn);

for(int chan(0); chan<kADCChannels;++chan)
   {
   auto &buffer = gSampleBuffers[chan];
   buffer.Clear();
   }

adc_setup();

//Restart the ADC timer
startADCTimer(gADCPointsPerSec);


//digitalWrite(12, LOW); //Clear Buffer overflow
//Packet::ResetPacketCount();
gState = kStartingSampling;

digitalWrite(LED_BUILTIN, HIGH);
}

void StopSampling()
{
gState = kIdle;
gFirstSampleTimeRequested = false;

for(int chan(0); chan<kADCChannels;++chan)
   {
   auto buffer = gSampleBuffers[chan];
   buffer.Clear();
   }
digitalWrite(LED_BUILTIN, LOW);   
}


void sendFirstSampleTimeIfNeeded()
{
if(!gFirstSampleTimeRequested)
   return;

gFirstSampleTimeRequested = false;
debugNewLine();   //Readability while testing only!

FirstSampleTimePacket ftPacket(gFirstADCPointus);
ftPacket.write(Serial);

debugNewLine();   //Readability while testing only!
}


void loop()
{
#ifdef TIMING_CHECK
int32_t delta = gLastADCus - gLastLastADCus;
if(delta > 0)
   {
   Serial.println("  delta = "+ String(delta));
   gLastLastADCus = gLastADCus;
   }
#endif

int hasRx = Serial.peek();

if(hasRx >= 0)
   {
   char cmdBuf[kMaxCommandLenBytes];
   int bytesRead = Serial.readBytesUntil('\n', cmdBuf, kMaxCommandLenBytes);
   #ifdef ENABLE_SERIAL_DEBUGGING
   SerialUSB.println("bytesRead="+String(bytesRead));
   SerialUSB.println(cmdBuf[0], HEX);
   SerialUSB.println(cmdBuf[1], HEX);
   SerialUSB.println();
   #endif
   auto cmd = cmdBuf[0];
   switch (cmd)
      {
      case 'b':   //begin sampling
         StartSampling();
         break;
      case 'f':   //first sample time
         gFirstSampleTimeRequested = true;
         if(gState == kSampling)
            sendFirstSampleTimeIfNeeded();
         break;
      case 's':   //stop sampling
         StopSampling();
         break;
      case 'n':   //return micro second time now
         {
         int32_t now = micros();
         //uint64_t now64 = micros64();
         //digitalWrite(5, HIGH);

         auto timeRequestNumber = cmdBuf[1];
         TimePacket timePacket(now, timeRequestNumber);
         timePacket.write(Serial);

         //digitalWrite(5, LOW);

         break;   
         }
      case 'v':   //version info
         Serial.write("ArduinoRT Example V0.9.0 $$$");
         Packet::ResetPacketCount(); //new session

         #ifdef ENABLE_SERIAL_DEBUGGING
         SerialUSB.println("Sent version info");
         #endif
         break;
      case '~': //sample rate
         {
         auto rateChar = cmdBuf[1]; //'0123456'
         unsigned int index = rateChar - '0';
         if(index < sizeof(kSampleRates)/sizeof(int))
            gADCPointsPerSec = kSampleRates[index];
         if(gADCPointsPerSec > 100)
            gADCPointsPerPacket = kPointsPerMediumSizePacket;
         else
            gADCPointsPerPacket = kPointsPerPacket;
            
         break;
         }
      default:
         break;
      }

   }

if(gState == kIdle)
   return;

if(gState == kHadFirstSample)
   {
   gState = kSampling;
   sendFirstSampleTimeIfNeeded();
   }

//Find the number of samples in the ringbuffer with the least samples
int points = gSampleBuffers[0].GetCount();
for(int chan(1); chan<kADCChannels;++chan)
   {
   auto &buffer = gSampleBuffers[chan];
   points = min(buffer.GetCount(), points);
   }


while(points >= gADCPointsPerPacket)
   {
   Packet packet;

   for(int pt(0);pt<gADCPointsPerPacket;++pt)
      {
      for(int chan(0); chan<kADCChannels;++chan)
         {
         auto &buffer = gSampleBuffers[chan];
         packet.addSample(chan, buffer.GetNext());
         }
      packet.nextPoint();   
      }
   
   //digitalWrite(7, HIGH);
   packet.write(Serial);
   //digitalWrite(7, LOW);

   --points;

   //debugNewLine();   //Readability while testing only!
   }

}//loop