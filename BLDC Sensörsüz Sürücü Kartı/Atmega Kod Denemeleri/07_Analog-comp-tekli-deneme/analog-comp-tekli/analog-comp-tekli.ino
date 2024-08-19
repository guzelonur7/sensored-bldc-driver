unsigned int i;
unsigned int x=0;
unsigned int y=0;
void setup() {
  
  DDRD  |= B00010000;      // (B00010000) PD4  çıkış ayarlandı. (Low side pinleri)
  PORTD  = 0x00;           // D portu pinleri lojik 0 ayarlandı.

  kenar_secim();
  ACSR |= 0x08;                    // Enable analog comparator interrupt  
  SREG  |= B10000000;       // Global kesmeler aktif.
  
  Serial.begin(9600);     // Seri haberleşme başalatma fonksiyonu.
  PORTD = B00010000;
}

ISR (ANALOG_COMP_vect) {
  for(i = 0; i < 10; i++) {           //We checl the comparator output 10 times to be sure
    if(x & 1)             //If step = odd (0001, 0011, 0101) 1, 3 or 5
    {
      if(!(ACSR & B00100000)) i -= 1; //!B00100000 -> B11011111 ACO = 0 (Analog Comparator Output = 0)
    }
    else                              //else if step is 0, 2 or 4
    {
      if((ACSR & B00100000))  i -= 1; //else if B00100000 -> B11011111 ACO = 1 (Analog Comparator Output = 1)
    }
  }
  Serial.println(x);
  kenar_secim();                    //set the next step of the sequence
  x++;                    //increment step by 1, next part of the sequence of 6
  y %= 2;                 //If step > 5 (equal to 6) then step = 0 and start over
}

void kenar_secim(){

  switch(y){
    
    case 0:
    bemf_falling();
    PORTD = B00000000;
    break;
    
    case 1:
    bemf_rising();
    PORTD = B00010000;
    break;
    }
  
}

void bemf_falling(){
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // Enable MUX select for negative input of comparator
  ADMUX = 0;                // Select A0 as comparator negative input
  ACSR &= ~0x01;            // Set interrupt on falling edge*/  
}

void bemf_rising(){
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // Enable MUX select for negative input of comparator
  ADMUX = 0;                // Select A0 as comparator negative input
  ACSR |= 0x03;             // Set interrupt on rising edge
  
}

void loop() {

}
