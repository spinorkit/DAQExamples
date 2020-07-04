const int ADC_FREQ = 10000;

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

void setup()
{
  Serial.begin (115200) ; // was for debugging
  adc_setup () ;         // setup ADC
 
  pmc_enable_periph_clk (TC_INTERFACE_ID + 0*3+0) ;  // clock the TC0 channel 0

  TcChannel * t = &(TC0->TC_CHANNEL)[0] ;    // pointer to TC0 registers for its channel 0
  t->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while setup regs
  t->TC_IDR = 0xFFFFFFFF ;     // disable interrupts
  t->TC_SR ;                   // read int status reg to clear pending
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // use TCLK1 (prescale by 2, = 42MHz)
              TC_CMR_WAVE |                  // waveform mode
              TC_CMR_WAVSEL_UP_RC |          // count-up PWM using RC as threshold
              TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;
 
 // t->TC_RC =  875 ;     // counter resets on RC, so sets period in terms of 42MHz clock
 // t->TC_RA =  440 ;     // roughly square wave
  t->TC_RC =  VARIANT_MCK/2/ADC_FREQ;     // counter resets on RC, so sets period in terms of 42MHz clock
  t->TC_RA =  VARIANT_MCK/2/ADC_FREQ/2 ;     // roughly square wave

  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares
 
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.

  setup_pio_TIOA0 () ;  // drive Arduino pin 2 at 48kHz to bring clock out
  dac_setup () ;        // setup up DAC auto-triggered at 48kHz

  enableMicroSecClock();
}

volatile boolean pinState;
volatile int32_t gMilliSec;

void TC3_Handler()
{
++gMilliSec;
TC_GetStatus(TC1, 0);
digitalWrite(12, pinState = !pinState);
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
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


//const int32_t kMilliSecRC = 84000000/2/1000; //42000
#define kMilliSecRC 42000

void enableMicroSecClock()
{
pinMode(12,OUTPUT);
startTimer(TC1, 0, TC3_IRQn, 1000); //TC1 channel 0, the IRQ for that channel and the desired frequency

 //   TcChannel * t = &(TC1->TC_CHANNEL)[0] ;    // pointer to TC1 registers for its channel 0
//   t->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while setup regs
//   t->TC_IDR = 0xFFFFFFFF ;     // disable interrupts
//   t->TC_SR ;                   // read int status reg to clear pending
//   t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // use TCLK1 (prescale by 2, = 42MHz)
//               TC_CMR_WAVE |                  // waveform mode
//               TC_CMR_WAVSEL_UP_RC |          // count-up PWM using RC as threshold
//               TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
//               TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
//               TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;
 
//  // t->TC_RC =  875 ;     // counter resets on RC, so sets period in terms of 42MHz clock
//  // t->TC_RA =  440 ;     // roughly square wave
//   t->TC_RC =  VARIANT_MCK/2/10000000;     // counter resets on RC, so sets period in terms of 42MHz clock
//   t->TC_RA =  VARIANT_MCK/2/1000000/4 ;     

//   t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares
 
//   t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.

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
  NVIC_EnableIRQ (ADC_IRQn) ;   // enable ADC interrupt vector
  ADC->ADC_IDR = 0xFFFFFFFF ;   // disable interrupts
  ADC->ADC_IER = 0x80 ;         // enable AD7 End-Of-Conv interrupt (Arduino pin A0)
  ADC->ADC_CHDR = 0xFFFF ;      // disable all channels
  ADC->ADC_CHER = 0x80 ;        // enable just A0
  ADC->ADC_CGR = 0x15555555 ;   // All gains set to x1
  ADC->ADC_COR = 0x00000000 ;   // All offsets off
 
  ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0) | (1 << 1) | ADC_MR_TRGEN ;  // 1 = trig source TIO from TC0
}

// Circular buffer, power of two.
#define BUFSIZE 0x400
#define BUFMASK 0x3FF
volatile int samples [BUFSIZE] ;
volatile int sptr = 0 ;
volatile int isr_count = 0 ;   // this was for debugging
volatile int lastIsr = 0 ;   // this was for debugging


#ifdef __cplusplus
extern "C"
{
#endif

void ADC_Handler (void)
{
  if (ADC->ADC_ISR & ADC_ISR_EOC7)   // ensure there was an End-of-Conversion and we read the ISR reg
  {
    int val = *(ADC->ADC_CDR+7) ;    // get conversion result
    samples [sptr] = val ;           // stick in circular buffer
    sptr = (sptr+1) & BUFMASK ;      // move pointer
    dac_write (0xFFF & ~val) ;       // copy inverted to DAC output FIFO
  }
  isr_count ++ ;
}

#ifdef __cplusplus
}
#endif


int32_t microsFixed()
{
int32_t ticks2;
int32_t millis2;
uint32_t ticks1 = TC_GetCV(TC1, 0);
int32_t millis1 = gMilliSec;

do
  {
  ticks2 = TC_GetCV(TC1, 0);
  millis2 = gMilliSec;
  } 
while (millis1 != millis2 || ticks1 > ticks2);

return millis2*1000 + (1000*ticks2)/kMilliSecRC;
}

int32_t microsNoInt()
{
uint32_t state = saveIRQState(); //Disable interrupts
uint32_t ticks1 = TC_GetCV(TC1, 0);
int32_t millis = gMilliSec;
uint32_t ticks2 = TC_GetCV(TC1, 0);
restoreIRQState(state);   //Enable interrupts
if(ticks1 > ticks2)
  ++millis; //HW counter wrapped

return millis*1000 + (1000*ticks2)/kMilliSecRC;
}


// Interrupt-compatible version of micros
// Theory: repeatedly take readings of SysTick counter, millis counter and SysTick interrupt pending flag.
// When it appears that millis counter and pending is stable and SysTick hasn't rolled over, use these 
// values to calculate micros. If there is a pending SysTick, add one to the millis counter in the calculation.
uint32_t micros2( void )
{
    uint32_t ticks, ticks2;
    uint32_t pend, pend2;
    uint32_t count, count2;

    ticks2  = SysTick->VAL;
    //pend2   = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)||((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)))  ;
    count2  = GetTickCount();

    do {
        ticks=ticks2;
        //pend=pend2;
        count=count2;
        ticks2  = SysTick->VAL;
        //pend2   = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)||((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)))  ;
        count2  = GetTickCount();
    } while ((count != count2) || (ticks < ticks2));

    return count * 1000 + (1000*(SysTick->LOAD + 1 - ticks))/ SysTick->LOAD ; 
    // this is an optimization to turn a runtime division into two compile-time divisions and 
    // a runtime multiplication and shift, saving a few cycles
}


int lastMicro; 

int printCount;

void loop()
{
  if (isr_count-lastIsr >= 100000)
    {
    lastIsr = isr_count;
    int now = micros();
    //isr_count = 0; 
    Serial.println(String(printCount) + "  delay  "+ String(now- lastMicro));
    ++printCount;

    // int printEnd = microsNoInt();
    // Serial.println(" print took: "+ String(printEnd - now));

    lastMicro = now; 
    }

}