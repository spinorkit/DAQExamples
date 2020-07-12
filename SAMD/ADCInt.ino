
 void setup() {
  //DDRB|= _BV(PINB5);  //sets up pin 5 portb (led) as an output
     TCCR1B=_BV(CS12)|_BV(CS10);//dived by 1024 timer counter 1
 
  sendstring("serial port working \n\r");	//verify serial port
  ADCSRA = (_BV(ADATE) |_BV(ADIE)|_BV(ADPS1)| _BV(ADPS0)); //divide by 8, ADC prescale bits, divides system clock to get adc clock
  ADCSRB|=_BV(ADTS2)|_BV(ADTS1);     //auto trigger on TC1 overflow
  ADMUX |= _BV(REFS0)| _BV(MUX1);			//select AVcc & channel 0 (PA1) Optical light sensor on ACE Shield
  ADCSRA |= _BV(ADEN);					   	//enable ADC
  sei();
}

void loop() {
   
  	//ADCSRA |= _BV(ADSC);	//start conversion : will not need this if autotriggering is used	
	//PINB =_BV(PINB5);   //this should toggle led, write to PINX register to toggle pin
	//while(!(ADCSRA & _BV(ADIF)));					//use this for polling ADC complete intrupt flag
	if (ADC_complete){
		   
		if (ADMUX & _BV(MUX0)){   //if chan 0 is selected
				ADMUX |=_BV(MUX1);    //turn on chan 1
				ADMUX &=~_BV(MUX0);   //turn off chan 0
			}
			else{
				ADMUX |=_BV(MUX0) ;  //
				ADMUX &= ~_BV(MUX1);
			}
			
		    //stop timer
			ADC_low = ADCL;                                     //read low order byte first, it's a must
			ADC_high = ADCH;	                               //read high order byte
			ADC_datareg=ADC_high<<8|ADC_low;		//save conversion result
			//ADC_datareg=ADCH<<8|ADCL;			//saves 2 instructions
			sprintf(str,"%i counts\r\n", ADC_datareg);	//display conversion result & channel
			sendstring(str);
			_delay_ms(1000);	   			
			ADC_complete=LOW;
			TIFR1|=_BV(TOV1);                             //clear tc1 ovf flag for next measurment
			
	}
	
}
/***************************************************************
     When ADC if finished converting this interrupt routine is run
********************************************************************/
ISR (ADC_vect){
	ADC_complete=HIGH;
	//ADCSRA |= _BV(ADIF);	//reset int. flag note: auto cleared by interrupt routine
}