/*
 *
 * 6 Adımı ledler ile kontrol etme çalışması.
 *
 * DİJİTAL 9  HIGH A  DİJİTAL 3 LOW A
 * DİJİTAL 10 HIGH B  DİJİTAL 4 LOW B
 * DİJİTAL 11 HIGH C  DİJİTAL 5 LOW C
 * 
 * Sisteme reciverdan okunan pwm'e göre duty cycle belirlenmesi dahil edilmiştir. çalışıyor.
 * 
*/


byte duty_cycle=3;
byte bldc_adim=0;

byte last_PWM_state=0;
unsigned long current_count,counter_1;  
int PWM_INPUT;



void setup() {
  Serial.begin(38400);
  
  DDRD  |= 0x38;           // B00111000 PD3 PD4 PD5 ÇIKIŞ AYARLANDI. LOW SIDE PINLERI.
  PORTD  = 0x00;           // D PORTU PİNLERİ LOJİK 0 AYARLANDI.
  
  DDRB  |= 0x0E;           // B00001110 PB1 PB2 PB3 ÇIKIŞ AYARLADNI. HIGH SIDE PINLERI.
  PORTB  = 0x00;           // B PORTU PİNLERİ LOJİK 0 AYARLANDI.

  OCR1A  = duty_cycle;                   // Set pin 9  PWM duty cycle
  OCR1B  = duty_cycle;                   // Set pin 10 PWM duty cycle
  OCR2A  = duty_cycle;                   // Set pin 11 PWM duty cycle

  PCICR |= (1 << PCIE0);    //PCMSK0 taramasını etkinleştir.                                                 
  PCMSK0 |= (1 << PCINT0);  //Pin D8'i durum değişikliğinde bir kesmeyi tetikleyin.
  SREG |= B10000000;        // global kesmeler aktif.
  
  Serial.println("setup bitti");
 
}

void loop() {
  
    Serial.println(bldc_adim+1);    
    Serial.println(duty_cycle); 
    switch(bldc_adim){

      case 0: 
      AC();
      BEMF_B_FALLING();
      break;
    
      case 1: 
      AB();
      BEMF_C_RISING();
      break;

      case 2: 
      CB();
      BEMF_A_FALLING();
      break;

      case 3: 
      CA();
      BEMF_B_RISING();
      break;

      case 4: 
      BA();
      BEMF_C_FALLING();
      break;
    
      case 5: 
      BC();
      BEMF_A_RISING();
      break;
    
      }
    delay(duty_cycle);
    if(bldc_adim >= 5)
    bldc_adim=0;
    else 
    bldc_adim+=1;
    
    OCR1A  = duty_cycle;                   // Set pin 9  PWM duty cycle
    OCR1B  = duty_cycle;                   // Set pin 10 PWM duty cycle
    OCR2A  = duty_cycle;                   // Set pin 11 PWM duty cycle
    
} // SONSUZ DONGU SONU.



ISR(PCINT0_vect){
  
  current_count = micros();                          //Micros ile o anki süreyi alıyoruz.
  
  if(PINB & B00000001){                              //Pin high mı kontrol 
    if(last_PWM_state == 0){                         //Son durum 0'sa durum değişikliği var demektir.  
      last_PWM_state = 1;                            //Son durumu high yaptık
      counter_1 = current_count;                     //counter_1'e şuanki süreyi atadık.
    }
  }
  else if(last_PWM_state == 1){                      // Pin low ve last state == 1 ise durum değişikliği var demektir.    
    last_PWM_state = 0;                              // Son durumu low güncelledik.
    PWM_INPUT = (current_count - counter_1)+200;         // Şimdiki değeri counter_1'den çıkardık. 
    duty_cycle = PWM_INPUT/15;
    //Serial.println(duty_cycle);                       // PWM duty değeri ekrana yazıldı. 
    
  }
}


void BEMF_A_RISING(){  
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // Enable MUX select for negative input of comparator
  ADMUX = 2;                // Select A2 as comparator negative input
  ACSR |= 0x03;             // Set interrupt on rising edge*/
}
void BEMF_A_FALLING(){
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // Enable MUX select for negative input of comparator
  ADMUX = 2;                // Select A2 as comparator negative input
  ACSR &= ~0x01;            // Set interrupt on falling edge*/
}
void BEMF_B_RISING(){
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // Enable MUX select for negative input of comparator
  ADMUX = 1;                // Select A1 as comparator negative input
  ACSR |= 0x03;             // Set interrupt on rising edge
}
void BEMF_B_FALLING(){
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // Enable MUX select for negative input of comparator
  ADMUX = 1;                // Select A1 as comparator negative input
  ACSR &= ~0x01;            // Set interrupt on falling edge*/
}
void BEMF_C_RISING(){
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // Enable MUX select for negative input of comparator
  ADMUX = 0;                // Select A0 as comparator negative input
  ACSR |= 0x03;             // Set interrupt on rising edge
}
void BEMF_C_FALLING(){
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // Enable MUX select for negative input of comparator
  ADMUX = 0;                // Select A0 as comparator negative input
  ACSR &= ~0x01;            // Set interrupt on falling edge*/
}


void AB(){
  PORTD = B00010000;      // D4 LOJİK 1.
  TCCR2A =  0;            // OC2A - B3 (DİJİTAL 11) PWM PASİF. 
  TCCR1A =  0x81;         // OC1A - B1 (DİJİTAL 9)PWM AKTİF.
}
void AC(){
  PORTD = B00100000;      // D5 LOJİK 1.
  TCCR2A =  0;            // OC2A - B3 (DİJİTAL 11) PWM PASİF.
  TCCR1A =  0x81;         // OC1A - B1 (DİJİTAL 9) PWM AKTİF.
}
void BC(){
  PORTD = B00100000;      // D5 LOJİK 1.
  TCCR2A =  0;            // OC2A - B3 (DİJİTAL 11) PWM PASİF.
  TCCR1A =  0x21;         // OC1B - B2 (DİJİTAL 10) PWM AKTİF.
}
void BA(){
  PORTD = B00001000;      // D3 LOJİK 1.
  TCCR2A =  0;            // OC2A - B3 (DİJİTAL 11) PWM PASİF. 
  TCCR1A =  0x21;         // OC1B - B2 (DİJİTAL 10) PWM AKTİF.
}
void CA(){
  PORTD = B00001000;      // D3 LOJİK 1.
  TCCR1A =  0;            // OC1A & OC1B (B1 ve B2)(DİJİTAL 9 VE DİJİTAL 10) PWM PASİF.
  TCCR2A =  0x81;         // OC2A - B3 (DİJİTAL 11) PWM AKTİF.
}
void CB(){
  PORTD = B00010000;      // D4 LOJİK 1.
  TCCR1A =  0;            // OC1A & OC1B (B1 ve B2)(DİJİTAL 9 VE DİJİTAL 10) PWM PASİF. 
  TCCR2A =  0x81;         // OC2A - B3 (DİGİTAL 11) PWM AKTİF. 
  }
