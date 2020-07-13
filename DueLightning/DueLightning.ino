
//https://www.arduino.cc/en/Hacking/PinMappingSAM3X

/*
Due Pin Number	SAM3X Pin Name	Mapped Pin Name	Max Output Current (mA)	Max Current Sink (mA) Function
4	PA16	Analog In 0	3	6       AD7
55	PA24	Analog In 1	3	6     AD6
56	PA23	Analog In 2	3	6     AD5
57	PA22	Analog In 3	3	6     AD4
58	PA6	Analog In 4	3	6       AD3
59	PA4	Analog In 5	3	6       AD2
60	PA3	Analog In 6	3	6       AD1
61	PA2	Analog In 7	3	6       AD0
62	PB17	Analog In 8	3	6     AD10
63	PB18	Analog In 9	3	6     AD11
64	PB19	Analog In 10	3	6   AD12
65	PB20	Analog In 11	3	6   AD13
66	PB15	DAC0	3	6
67	PB16	DAC1	3	6
*/


/**
 SR/IRQ TC Channel	Due pins
TC0	TC0	0	2, 13
TC1	TC0	1	60, 61
TC2	TC0	2	58
TC3	TC1	0	none  <- this line in the example below
TC4	TC1	1	none
TC5	TC1	2	none
TC6	TC2	0	4, 5
TC7	TC2	1	3, 10
TC8	TC2	2	11, 12
*/

#define Serial SerialUSB

/**
 * \brief Set RC on the selected channel.
 *
 * \param tc Pointer to a TC instance.
 * \param chan Channel to configure.
 * \param v New value for RC.
 */
uint32_t TC_GetRC(Tc *tc, uint32_t chan) {
	return tc->TC_CHANNEL[chan].TC_RC;
}

uint32_t TC_GetCV(Tc *tc, uint32_t chan) {
	return tc->TC_CHANNEL[chan].TC_CV;
}


void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) 
{
pmc_set_writeprotect(false);
pmc_enable_periph_clk((uint32_t)irq);
TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1); // TC_CMR_TCCLKS_TIMER_CLOCK4);
uint32_t rc = VARIANT_MCK/2/frequency; //2 because we selected TIMER_CLOCK1 above
TC_SetRA(tc, channel, rc/2); //50% high, 50% low
TC_SetRC(tc, channel, rc);
TC_Start(tc, channel);
tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
NVIC_EnableIRQ(irq);
}


void startADCTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) 
{
pmc_set_writeprotect(false);
pmc_enable_periph_clk((uint32_t)irq);

TC_Stop(tc, channel);

TcChannel *tcChan = tc->TC_CHANNEL + channel;

TC_Configure(tc, channel, 
   TC_CMR_TCCLKS_TIMER_CLOCK1 |           // use TCLK1 (prescale by 2, = 42MHz)
   TC_CMR_WAVE |                          // waveform mode
   TC_CMR_WAVSEL_UP_RC |                  // count-up PWM using RC as threshold
   TC_CMR_EEVT_XC0 |                      // Set external events from XC0 (this setup TIOB as output)
   TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR | TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR
   ); 

uint32_t rc = VARIANT_MCK/2/frequency; //2 because we selected TIMER_CLOCK1 above
TC_SetRC(tc, channel, rc);
TC_SetRA(tc, channel, rc/2); //50% high, 50% low

tcChan->TC_CMR = (tcChan->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares

TC_Start(tc, channel);
}

const int kADCPointsPerSec = 4000;

inline uint32_t saveIRQState(void)
{
  uint32_t pmask = __get_PRIMASK() & 1;
  __set_PRIMASK(1);
  return pmask;
}


inline void restoreIRQState(uint32_t pmask)
{
__set_PRIMASK(pmask);
}

enum State
{
kIdle,
kStartingSampling,
kHadFirstSample,
kSampling,  
};

volatile State gState = kIdle;
volatile bool gFirstSampleTimeRequested = false;

void debugNewLine()
{
//Serial.write('\n'); //Readability while testing only!
}


void setup()
{
Serial.begin (115200);
#ifdef ENABLE_SERIAL_DEBUGGING 
SerialUSB.begin(0);  //debugging
#endif

pinMode(8, OUTPUT);
pinMode(12, OUTPUT);

adc_setup () ;         // setup ADC

//Timer, channel, IRQ, Frequency
startADCTimer(TC0, 0, TC0_IRQn, kADCPointsPerSec);

setup_pio_TIOA0 () ;  // drive Arduino pin 2 at 48kHz to bring clock out
dac_setup () ;        // setup up DAC auto-triggered at 48kHz

#ifdef ENABLE_SERIAL_DEBUGGING
SerialUSB.println("Arduino setup complete");
#endif
}


void setup_pio_TIOA0 ()  // Configure Ard pin 2 as output from TC0 channel A (copy of trigger event)
{
  PIOB->PIO_PDR = PIO_PB25B_TIOA0 ;  // disable PIO control
  PIOB->PIO_IDR = PIO_PB25B_TIOA0 ;   // disable PIO interrupts
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0 ;  // switch to B peripheral
}


void dac_setup ()
{
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ; // start clocking DAC
  DACC->DACC_CR = DACC_CR_SWRST ;  // reset DAC

  DACC->DACC_MR =
    DACC_MR_TRGEN_EN | DACC_MR_TRGSEL (1) |  // trigger 1 = TIO output of TC0
    (0 << DACC_MR_USER_SEL_Pos) |  // select channel 0
    DACC_MR_REFRESH (0x0F) |       // bit of a guess... I'm assuming refresh not needed at 48kHz
    (24 << DACC_MR_STARTUP_Pos) ;  // 24 = 1536 cycles which I think is in range 23..45us since DAC clock = 42MHz

  DACC->DACC_IDR = 0xFFFFFFFF ; // no interrupts
  DACC->DACC_CHER = DACC_CHER_CH0 << 0 ; // enable chan0
}

void dac_write (int val)
{
  DACC->DACC_CDR = val & 0xFFF ;
}



void adc_setup ()
{
/* 
variant.cpp Arduino ADC initialization

 // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);

*/
  pmc_enable_periph_clk(ID_ADC);

  /**
 * \brief Initialize the given ADC with the specified ADC clock and startup time.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_mck Main clock of the device (value in Hz).
 * \param ul_adc_clock Analog-to-Digital conversion clock (value in Hz).
 * \param uc_startup ADC start up time. Please refer to the product datasheet
 * for details.
 *
 * \return 0 on success.
 */
  adc_init(ADC, VARIANT_MCK, ADC_FREQ_MAX, ADC_STARTUP_FAST);

  /**
 * \brief Configure ADC timing.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param uc_tracking ADC tracking time = uc_tracking / ADC clock.
 * \param uc_settling Analog settling time = (uc_settling + 1) / ADC clock.
 * \param uc_transfer Data transfer time = (uc_transfer * 2 + 3) / ADC clock.
 * 
void adc_configure_timing(Adc *p_adc, const uint8_t uc_tracking,
		const enum adc_settling_time_t settling,const uint8_t uc_transfer) 

    12-bit mode: tTRACK = 0.054 × ZSOURCE + 205
With tTRACK expressed in ns and ZSOURCE expressed in ohms.
I.e. 1kOhm source => 260 ns.

tTRACK in nanoseconds 
12 bit mode: 1/fS = tTRACK - 15 × tCP_ADC + 5 tCP_ADC

Tracking Time = (TRACKTIM + 1) * ADCClock periods.
Transfer Period = (TRANSFER * 2 + 3) ADCClock periods.

 */

  adc_configure_timing(ADC, 15, ADC_SETTLING_TIME_3, 1);


  NVIC_EnableIRQ (ADC_IRQn) ;   // enable ADC interrupt vector
  ADC->ADC_IDR = 0xFFFFFFFF ;   // disable interrupts
  //ADC->ADC_IER = 0x80 ;         // enable AD7 End-Of-Conv interrupt (Arduino pin A0)
  ADC->ADC_IER = 0x80 ;         // enable AD7 End-Of-Conv interrupt (Arduino pin A0)
  ADC->ADC_CHDR = 0xFFFF ;      // disable all channels
  //ADC->ADC_CHER = 0x80 ;        // enable just A0
  ADC->ADC_CHER = 0xc0 ;        // enable A1 and A0
  ADC->ADC_CGR = 0x15555555 ;   // All gains set to x1
  ADC->ADC_COR = 0x00000000 ;   // All offsets off
 
  ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0) | (1 << 1) | ADC_MR_TRGEN ;  // 1 = trig source TIO from TC0
}

template <class T, uint Log2Size>
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

const int kADCChannels = 2; //must be power of 2 (for now)
const int kBytesPerSample = sizeof(int16_t);

const int kLog2BufferPoints = 12;
const int kBufferSizeBytes = (1 << kLog2BufferPoints)*kADCChannels*kBytesPerSample;

const int kPointsPerPacket = 1;

const int kLog2BufferSizeBytes = 15;
const int kLog2ADCChannels = 1;
const int kLog2BytesPerSample = 1;


typedef RingBufferSized<int16_t, kLog2BufferPoints> TRingBuf;

TRingBuf gSampleBuffers[kADCChannels];


volatile int32_t gFirstADCPointus = 0;

void ADC_Handler (void)
{
if(gState == kStartingSampling)
   {
   gFirstADCPointus = micros();
   gState = kHadFirstSample;
   }

if (ADC->ADC_ISR & ADC_ISR_EOC6)   // ensure there was an End-of-Conversion and we read the ISR reg
  {
  int val = *(ADC->ADC_CDR+6);    // get conversion result
  digitalWrite(8, HIGH);

  if(gState == kSampling)
    {
    gSampleBuffers[0].Push(val);           // stick in circular buffer
    //Serial.print(val,HEX);
    }
  }

if (ADC->ADC_ISR & ADC_ISR_EOC7)   // ensure there was an End-of-Conversion and we read the ISR reg
  {
  int val = *(ADC->ADC_CDR+7) ;    // get conversion result
  digitalWrite(12, HIGH);

  if(gState == kSampling)
     {
     gSampleBuffers[1].Push(val);           // stick in circular buffer
     dac_write (0xFFF & ~val) ;       // copy inverted to DAC output FIFO
     //Serial.print("\t");
     //Serial.println(val,HEX);
     }
  }

digitalWrite(8, LOW);
digitalWrite(12, LOW);
}

class Packet
{
   //The header is 5 nibbles, i.e. "P\xA0\x40". The low nibble of the
   //3rd byte is the packet time (0x04) for data packets.
   //The head and packet type is followed by a 1 byte packet count number,
   //making a total of 4 bytes before the payload daya that need to match the 
   //expected pattern(s) before the client can detect a packet.
   const char sHeaderAndPacketType[3] = {'P',0xA0,'D'}; //D for data

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
      if(mPoint >= kPointsPerPacket)
         return false;
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
      int n = stream.write(sHeaderAndPacketType, 3);
      n += stream.write(sPacketCount++);
      n += stream.write(reinterpret_cast<const uint8_t*>(mData), sizeof(mData));
      return n;
      }


protected:

   static uint8_t sPacketCount;  

   int mPoint;
   int16_t mData[kPointsPerPacket][kADCChannels];

};

uint8_t Packet::sPacketCount = 0;   


class TimePacket : protected Packet
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

class FirstSampleTimePacket : protected Packet
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
//TODO: restart the ADC timer here
Packet::ResetPacketCount();
gState = kStartingSampling;
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

//int rxAvail = Serial.available();
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
         auto timeRequestNumber = cmdBuf[1];
         TimePacket timePacket(now, timeRequestNumber);
         timePacket.write(Serial);
         break;   
         }
      case 'v':   //version info
         Serial.write("ArduinoRT Example V0.9.0 $$$");
         #ifdef ENABLE_SERIAL_DEBUGGING
         SerialUSB.println("Sent version info");
         #endif
         break;
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


while(points >= kPointsPerPacket)
   {
   Packet packet;

   for(int pt(0);pt<kPointsPerPacket;++pt)
      {
      for(int chan(0); chan<kADCChannels;++chan)
         {
         auto &buffer = gSampleBuffers[chan];
         packet.addSample(chan, buffer.GetNext());
         }
      packet.nextPoint();   
      }
   
   packet.write(Serial);

   --points;

   debugNewLine();   //Readability while testing only!
   }

}
