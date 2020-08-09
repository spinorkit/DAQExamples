/*
 * RS_Clocks.c
 *
 * Created: 2/4/2018 7:52:58 AM
 *  Author: jeff andle, Synergistic Science
 * 
 * https://community.atmel.com/forum/32768khz-8mhz-or-48mhz-external-crystal-d51
 */ 
 #include "atmel_start.h"
 #include <peripheral_clk_config.h>


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



 // =============================================================================
 // 32768 slow clock using external oscillator.
 // GCLK4 feeds 32768 so FLL could be used
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
	 FatalError(ERR_CLOCK_PLL, PllFreq_Hz);
	 
	 uint32_t PllDivOut = 1, targetFreq = PllFreq_Hz;
	 
	 while (targetFreq < 96000000 && targetFreq <= 200000000 && PllDivOut < 256)
	 {
		 PllDivOut++;
		 targetFreq = PllFreq_Hz*PllDivOut;
	 }
	 
	 if (targetFreq > 200000000 || PllDivOut >= 256)
	 FatalError(ERR_CLOCK_TGT, targetFreq);
	 
	 
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