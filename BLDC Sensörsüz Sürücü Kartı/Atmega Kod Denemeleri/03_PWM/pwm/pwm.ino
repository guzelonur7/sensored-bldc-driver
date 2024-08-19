void setup() {

  // Timer1 module setting: set clock source to clkI/O / 1 (no prescaling)
  TCCR1A = 0;
  TCCR1B = 0x01;
  
  // Timer2 module setting: set clock source to clkI/O / 1 (no prescaling)
  TCCR2A = 0;
  TCCR2B = 0x01;

  OCR1A  = 125;                   // Set pin 9  PWM duty cycle
  OCR1B  = 125;                   // Set pin 10 PWM duty cycle
  OCR2A  = 125;                   // Set pin 11 PWM duty cycle
  
}

void loop() {

  TCCR2A =  0;            //OC2A - D11 normal port. 
  TCCR1A =  0x81;         //OC1A - D9 compare match noninverting mode, downcounting ,PWM 8-bit
  delay(100);
  
  
  TCCR2A =  0;            //OC2A - D11 normal port. 
  TCCR1A =  0x21;         //OC1B - D10 compare match noninverting mode, downcounting ,PWM 8-bit 
  delay(100);

  
  TCCR1A =  0;            // OC1A and OC1B normal port
  TCCR2A =  0x81;         // OC2A - D11 compare match noninverting mode, downcounting ,PWM 8-bit
  delay(100);
  
}
