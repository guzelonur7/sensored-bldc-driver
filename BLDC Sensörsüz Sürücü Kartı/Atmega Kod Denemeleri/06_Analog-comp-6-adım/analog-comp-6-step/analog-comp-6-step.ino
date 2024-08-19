/*
 * Analog komparator denemesi yapıalcak.
 * 6 adım kodları ve analog komp kesme kodları eklenecek.
 * 
 * DİJİTAL 3  --> LOW C
 * DİJİTAL 4  --> LOW B
 * DİJİTAL 5  --> LOW A
 * DİJİTAL 9  --> HIGH A 
 * DİJİTAL 10 --> HIGH B  
 * DİJİTAL 11 --> HIGH C 
 * 
 * PD6(AIN0) --> KOMPARATOR POZİTİF.
 * A0 --> BEMF C NEGATİF KOMPARATOR.
 * A1 --> BEMF B NEGATİF KOMPARATOR.
 * A2 --> BEMF A NEGATİF KOMPARATOR.
 * 
 * Analog komparator kesmesi içerisinde adımlar artacak.
 * 
 * Komparator çalışması tamamlandı. zıt emk pinlerine kondansatör koyularak filtre sorununun önüne geçildi..
*/

#define PWM_max_value      255
#define PWM_min_value      50

int PWM_IN_MAX = 2200;
int PWM_IN_MIN = 800;
int PWM_INPUT;
int x=0,i;

byte adim=0;
byte last_PWM_state=0;
byte motor_speed;

unsigned long current_count,counter_1;  

void setup() {
  
  Serial.begin(38400);     // Seri haberleşme başalatma fonksiyonu.
  
  DDRD  |= 0x38;           // (B00111000) PD3 PD4 PD5 çıkış ayarlandı. (Low side pinleri)
  PORTD  = 0x00;           // D portu pinleri lojik 0 ayarlandı.
  
  DDRB  &= 0x00;           // b portu pinler giriş.
  DDRB  |= 0x0E;           // (B00001110) PB1 PB2 PB3 çıkış ayarlandı. (High side pinleri)
  
  PORTB  = 0x00;           // B portu pinleri lojik 0 ayarlandı.
  
  
  TCCR1A = 0;              // Timer1 ayarları.
  TCCR1B = 0x01;           // Timer1 ayarları.

  
  TCCR2A = 0;              // Timer2 ayarları.
  TCCR2B = 0x01;           // Timer2 ayarları.
  
  OCR1A  = PWM_min_value;       // Dijital 9  Pini pwm duty cycle değeri ayarlandı.
  OCR1B  = PWM_min_value;       // Dijital 10 Pini pwm duty cycle değeri ayarlandı.
  OCR2A  = PWM_min_value;       // Dijital 11 Pini pwm duty cycle değeri ayarlandı.

  ACSR  = 0x10;                 // Analog komparator kesme bayrağı temizlendi.
  
  /* !!!!! D8 İÇİN KESME AYARI SİLİNDİ EKLENECEK !!!!! */
  
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0  = 0;
  OCR0A = 19;
  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
  TCCR0A |= (1 << WGM01);
  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
  TCCR0B |= (1 << CS01);
  TIMSK0 |= (1 << OCIE0A);
  /* Timer0 kesmesi aktif hale getirildi */

  SREG |= B10000000;            // Global kesmeler aktif.
  
  x=2000;                        
  delay(2500);                  // Başlangıç gecikmesi. 2.5sn
  
  while(x>500){                 // İlk hareketlendirme.  
    delayMicroseconds(x);
    bldc_adim();                // Adım fonksiyonları.
    adim++;                     // Bir sonraki adım için değişken ayarlandı.
    adim %= 6;                  // Eğer adım 6 ise 0'a tekrar ayarlandı.
    x=x-5;
}
 ACSR |= 0x08;                   // Analog interrupt kesmesi aktif.
 /*PCICR |= (1 << PCIE0);        // PCMSK0 taramasını etkinleştir.                                                 
 PCMSK0 |= (1 << PCINT0);      // Pin D8'i durum değişikliğinde bir kesmeyi tetikleyin.*/
 
 Serial.println("SETUP BİTTİ");
}

// Analog komparator kesme fonkisyonu.
ISR (ANALOG_COMP_vect) {
 
  for(i = 0; i < 10; i++) {           // Komparator çıkışı 10 kez kontrol ediliyor.
    if(adim & 1)                      // Eğer adım = tek (0001, 0011, 0101) 1, 3 yada 5 ise;
    {
      if(!(ACSR & B00100000)) i -= 1;        // !B00100000 -> B11011111 ACO = 0 (Analog Comparator Output = 0) (düşen kenar)
      
    }
    else                              // değilse yani eğer adım (0000,0010,0100) 0, 2 yada 4 ise;
    {
      if((ACSR & B00100000))  i -= 1;         // B00100000 -> B11011111 ACO = 1 (Analog Comparator Output = 1) (yükselen kenar)     
    }
  }
  
  bldc_adim();               // BLDC adımı ayarlandı.
  adim++;                    // bir sonraki adım için değişken ayarlandı.
  adim %= 6;                 // eğer adım 6 ise 0'a tekrar ayarlandı.
   
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
    counter_1++;
    last_PWM_state=1;      
  } 
  
  else if(!(PINB & B00000001) && last_PWM_state==1 ){
  PWM_INPUT=counter_1*10;
  //Serial.println(PWM_INPUT); 
      if(PWM_INPUT >= PWM_IN_MAX){
      PWM_INPUT = PWM_IN_MAX;
      }
    else if(PWM_INPUT <= PWM_IN_MIN){
      PWM_INPUT = PWM_IN_MIN;
      }
        motor_speed = ((PWM_INPUT-PWM_IN_MIN)/6.83)+PWM_min_value;
  SET_PWM(motor_speed);
  last_PWM_state=0;
  counter_1=0;
  }
}



/*
ISR(PCINT0_vect){
  
  current_count = micros();                          // Micros ile o anki süreyi alıyoruz.
  
  if(PINB & B00000001){                              // Pin high mı kontrol 
    if(last_PWM_state == 0){                         // Son durum 0'sa durum değişikliği var demektir.  
      last_PWM_state = 1;                            // Son durumu high yaptık
      counter_1 = current_count;                     // counter_1'e şuanki süreyi atadık.
    }
  }
  else if(last_PWM_state == 1){                      // Pin low ve last state == 1 ise durum değişikliği var demektir.    
    last_PWM_state = 0;                              // Son durumu low güncelledik.
    PWM_INPUT = current_count - counter_1;           // Şimdiki değeri counter_1'den çıkardık.    
  }
}
*/
void loop() {


  //Serial.println(motor_speed); 
}

// Motor adım fanksiyonları.
void bldc_adim(){

  switch(adim){

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
}

// ZIT EMK FONKSİYONLARI
void BEMF_A_RISING(){  
  ADCSRA = (0 << ADEN);     // ADC modülü pasif edildi.
  ADCSRB = (1 << ACME);     // Analog komparatorün negatif girişi için MUX aktif edildi.
  ADMUX = 2;                // Komparator negatif girişi için ADC2 girişi seçildi.
  ACSR |= 0x03;             // Kesme yükselen kenara ayarlandı.
}
void BEMF_A_FALLING(){
  ADCSRA = (0 << ADEN);     // ADC modülü pasif edildi.
  ADCSRB = (1 << ACME);     // Analog komparatorün negatif girişi için MUX aktif edildi.
  ADMUX = 2;                // Komparator negatif girişi için ADC2 girişi seçildi.
  ACSR &= ~0x01;            // Kesme düşen kenara ayarlandı.
}
void BEMF_B_RISING(){
  ADCSRA = (0 << ADEN);     // ADC modülü pasif edildi.
  ADCSRB = (1 << ACME);     // Analog komparatorün negatif girişi için MUX aktif edildi.
  ADMUX = 1;                // Komparator negatif girişi için ADC1 girişi seçildi.
  ACSR |= 0x03;             // Kesme yükselen kenara ayarlandı.
}
void BEMF_B_FALLING(){
  ADCSRA = (0 << ADEN);     // ADC modülü pasif edildi.
  ADCSRB = (1 << ACME);     // Analog komparatorün negatif girişi için MUX aktif edildi.
  ADMUX = 1;                // Komparator negatif girişi için ADC1 girişi seçildi.
  ACSR &= ~0x01;            // Kesme düşen kenara ayarlandı.
}
void BEMF_C_RISING(){
  ADCSRA = (0 << ADEN);     // ADC modülü pasif edildi.
  ADCSRB = (1 << ACME);     // Analog komparatorün negatif girişi için MUX aktif edildi.
  ADMUX = 0;                // Komparator negatif girişi için ADC0 girişi seçildi.
  ACSR |= 0x03;             // Kesme yükselen kenara ayarlandı.
}
void BEMF_C_FALLING(){
  ADCSRA = (0 << ADEN);     // ADC modülü pasif edildi.
  ADCSRB = (1 << ACME);     // Analog komparatorün negatif girişi için MUX aktif edildi.
  ADMUX = 0;                // Komparator negatif girişi için ADC0 girişi seçildi.
  ACSR &= ~0x01;            // Kesme düşen kenara ayarlandı.
}


// MOTOR FAZ ADIM FONKSİYONLARI
void AB(){
  PORTD = B00010000;      // D4 lojik 1.
  TCCR2A =  0;            // OC2A - PB3 (dijital 11) pwm pasif. 
  TCCR1A =  0x81;         // OC1A - B1 (dijital 9) pwm aktif.
}
void AC(){
  PORTD = B00001000;      // D3 lojik 1.
  TCCR2A =  0;            // OC2A - PB3 (dijital 11) pwm pasif.
  TCCR1A =  0x81;         // OC1A - PB1 (dijital 9) pwm aktif.
}
void BC(){
  PORTD = B00001000;      // D3 lojik 1.
  TCCR2A =  0;            // OC2A - PB3 (dijital 11) pwm pasif.
  TCCR1A =  0x21;         // OC1B - PB2 (dijital 10) pwm aktif.
}
void BA(){
  PORTD = B00100000;      // D5 lojik 1.
  TCCR2A =  0;            // OC2A - PB3 (dijital 11) pwm pasif. 
  TCCR1A =  0x21;         // OC1B - PB2 (dijital 10) pwm aktif.
}
void CA(){
  PORTD = B00100000;      // D5 lojik 1.
  TCCR1A =  0;            // OC1A & OC1B (PB1 ve PB2)(dijital 9 ve dijital 10) pwm pasif.
  TCCR2A =  0x81;         // OC2A - PB3 (dijital 11) pwm aktif.
}
void CB(){
  PORTD = B00010000;      // D4 lojik 1.
  TCCR1A =  0;            // OC1A & OC1B (PB1 ve PB2)(dijital 9 ve dijital 10) pwm pasif. 
  TCCR2A =  0x81;         // OC2A - PB3 (dijital 11) pwm aktif. 
  }



void SET_PWM(byte width_value){
  OCR1A  = width_value;                   // Set pin 9  PWM duty cycle
  OCR1B  = width_value;                   // Set pin 10 PWM duty cycle
  OCR2A  = width_value;                   // Set pin 11 PWM duty cycle
}
