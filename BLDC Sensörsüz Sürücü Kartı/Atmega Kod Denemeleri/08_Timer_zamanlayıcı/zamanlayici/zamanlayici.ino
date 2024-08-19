int onceki,x=0;
byte son_durum;
unsigned int sure=0;

void setup() {
  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
  
  pinMode(3,OUTPUT);
  
  PORTB |= B00000001; // B0 İNPUT TANIMLANDI.
  
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0  = 0;
  OCR0A = 19;
  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
  TCCR0A |= (1 << WGM01);
  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
  TCCR0B |= (1 << CS01);
  TIMSK0 |= (1 << OCIE0A);
  /* Timer1 kesmesi aktif hale getirildi */
SREG |= B10000000;            // Global kesmeler aktif.
Serial.begin(38400);
}

ISR(TIMER0_COMPA_vect){

  /*if(x==0){
  digitalWrite(3,HIGH);
  x=1;
  }
  
  else if(x==1){
  digitalWrite(3,LOW);
  x=0;
  }*/
  
  if((PINB & B00000001)){  // D8 HIGH İSE
    x++;
    son_durum=1;      
  } 
  
  else if(!(PINB & B00000001) && son_durum==1 ){
  sure=x*10;
  Serial.println(sure); 
  son_durum=0;
  x=0;
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  

}
