/*
*   MOTOR HAREKETİ TAMAMLANDI. KUMANDADAN GELEN VERİYİ ANLIK ALARAK MOTORA İLETEBİLİYORUZ.
*/

#define pwm_baslangic_deger 75
#define pwm_min_deger 35
#define pwm_max_deger 250
#define pwm_in_max 2200
#define pwm_in_min 800

int pwm_giris;

byte bldc_adim=0;
byte son_durum_pwm;

unsigned int filtre_sayac=0;
unsigned int filtre_toplam;
unsigned int i;
unsigned int motor_hizi;
unsigned long zaman, simdiki_zaman;

void setup() {
  
  //Serial.begin(115200);
  
  DDRD  |= 0x38;                // PD3 PD4 PD5 çıkış ayarlandı. (Low side pinleri)
  PORTD  = 0x00;                // D portu pinleri lojik 0 ayarlandı.
  DDRB  |= 0x0E;                // PB1 PB2 PB3 çıkış ayarlandı. (High side pinleri)
  PORTB  = 0x00;                // B portu pinleri lojik 0 ayarlandı.
  
  TCCR1A = 0;                   // Timer1 ayarları. 
  TCCR1B = 0x01;                // clkI/O / 1 (8 prescaling)
  TCCR2A = 0;                   // Timer2 ayarları.
  TCCR2B = 0x01;                // clkI/O / 1 (8 prescaling) 

  PWM_AYARLA(pwm_baslangic_deger);  // PWM ayarlandı.
  
  ACSR  = 0x10;                 // Analog komparator kesme bayrağı temizlendi.
  SREG |= B10000000;            // Global kesmeler aktif.

  i=4000;                        
  delay(2500);                  // Başlangıç gecikmesi. 2.5sn
  
  while(i>200){                 // İlk hareketlendirme.  
    delayMicroseconds(i);
    bldc_hareket();             // Adım fonksiyonları.
    bldc_adim++;                // Bir sonraki adım için değişken ayarlandı.
    bldc_adim %= 6;             // Eğer adım 6 ise 0'a tekrar ayarlandı.
    i=i-5;
  }
  ACSR |= 0x08;                 // Analog interrupt kesmesi aktif.
  PCICR |= (1 << PCIE0);        // PCMSK0 taramasını etkinleştir.                                                 
  PCMSK0 |= (1 << PCINT0);      // Pin D8'i durum değişikliğinde bir kesmeyi tetikleyin.
}

ISR (ANALOG_COMP_vect) {
  
  for(i = 0; i < 7; i++) {              // Komparator çıkışı 10 kez kontrol ediliyor.           //// MİNİMUM 5.. MOTOR GÜZEL HAREKET EDİYOR. 35'DEN AŞAĞI HIZ SIKINTI.
    if(bldc_adim & 1)                   // Eğer adım = tek (0001, 0011, 0101) 1, 3 yada 5 ise;
    {
      if(!(ACSR & B00100000)){
        i -= 1;         // !B00100000 -> B11011111 ACO = 0 (Analog Comparator Output = 0) (düşen kenar)      
        if(PCIFR==1) break;
      }
    }
    else  // değilse yani eğer adım (0000,0010,0100) 0, 2 yada 4 ise;
    {
      if((ACSR & B00100000)) {
        i -= 1; // B00100000 -> B11011111 ACO = 1 (Analog Comparator Output = 1) (yükselen kenar) 
        if(PCIFR==1) break;
        }
    }
  }
   
  bldc_hareket();      // BLDC adımı ayarlandı.
  ACSR  = 0x10;        // Analog komparator kesme bayrağı temizlendi.  ////// FLAG TEMİZLENDİ.
  ACSR |= 0x08;        // Analog interrupt kesmesi aktif.              ////// KESME AKTİF EDİLDİ.    
  }



ISR(PCINT0_vect){ 
  
  simdiki_zaman = micros();          // Micros ile o anki süreyi alıyoruz.
  
  if(PINB & B00000001){              // Pin high mı kontrol 
      if(son_durum_pwm == 0){        // Son durum 0'sa durum değişikliği var demektir.  
        son_durum_pwm = 1;           // Son durumu high yaptık
        zaman = simdiki_zaman;       // counter_1'e şuanki süreyi atadık.
      }
  }
  
  else if(son_durum_pwm == 1){                      // Pin low ve last state == 1 ise durum değişikliği var demektir.    
      son_durum_pwm = 0;                            // Son durumu low güncelledik.
      pwm_giris = simdiki_zaman - zaman;                
      pwm_giris = constrain(pwm_giris, pwm_in_min, pwm_in_max);
      motor_hizi = map(pwm_giris,pwm_in_min,pwm_in_max,pwm_min_deger,pwm_max_deger);
      //Serial.println(motor_hizi);
      PWM_AYARLA(motor_hizi);
  }
  PCIFR=0x00;   
}

void loop() {
  
}

// Motor hareket fonksiyonları.
void bldc_hareket(){

  switch(bldc_adim){

    case 0: 
    AC();
    BEMF_B_FALLING(); break;
    
    case 1: 
    AB();
    BEMF_C_RISING(); break;
    
    case 2: 
    CB();
    BEMF_A_FALLING(); break;
    
    case 3: 
    CA();
    BEMF_B_RISING(); break;
   
    case 4: 
    BA(); 
    BEMF_C_FALLING(); break;
    
    case 5: 
    BC();
    BEMF_A_RISING(); break;   
  }
  bldc_adim++;         // bir sonraki adım için değişken ayarlandı.
  bldc_adim %= 6;      // eğer adım 6 ise 0'a tekrar ayarlandı.    
}

// Zıt emk fonksiyonları.
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

// Faz enerjilendirme fonksiyonları.
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

void PWM_AYARLA(byte width_value){
 /* if(width_value < pwm_min_deger)
    width_value  = pwm_min_deger;
  if(width_value > pwm_max_deger)
    width_value  = pwm_max_deger;*/
  OCR1A  = width_value;                   // Set pin 9  PWM duty cycle
  OCR1B  = width_value;                   // Set pin 10 PWM duty cycle
  OCR2A  = width_value;                   // Set pin 11 PWM duty cycle
}
