#define pwm_baslangic_deger 40
#define pwm_min_deger 35
#define pwm_max_deger 250

unsigned int i;
byte bldc_adim=0;
unsigned int motor_hizi;

void setup() {
  
  DDRD  |= 0x38;                // (B00111000) PD3 PD4 PD5 çıkış ayarlandı. (Low side pinleri)
  PORTD  = 0x00;                // D portu pinleri lojik 0 ayarlandı.
  DDRB  |= 0x0E;                // (B00001110) PB1 PB2 PB3 çıkış ayarlandı. (High side pinleri)
  PORTB  = 0x00;                // B portu pinleri lojik 0 ayarlandı.
  
  TCCR1A = 0;                   // Timer1 ayarları. 
  TCCR1B = 0x01;                // Timer1 ayarları. clkI/O / 1 (8 prescaling)
  TCCR2A = 0;                   // Timer2 ayarları.
  TCCR2B = 0x01;                // Timer2 ayarları. clkI/O / 1 (8 prescaling) 
  
  OCR1A  = pwm_baslangic_deger;       // Dijital 9  Pini pwm duty cycle değeri ayarlandı.
  OCR1B  = pwm_baslangic_deger;       // Dijital 10 Pini pwm duty cycle değeri ayarlandı.
  OCR2A  = pwm_baslangic_deger;       // Dijital 11 Pini pwm duty cycle değeri ayarlandı.
  
  ACSR  = 0x10;                 // Analog komparator kesme bayrağı temizlendi.
  SREG |= B10000000;            // Global kesmeler aktif.

  i=3000;                        
  delay(2500);                  // Başlangıç gecikmesi. 2.5sn
  
  while(i>200){                 // İlk hareketlendirme.  
    delayMicroseconds(i);
    bldc_hareket();             // Adım fonksiyonları.
    bldc_adim++;                // Bir sonraki adım için değişken ayarlandı.
    bldc_adim %= 6;             // Eğer adım 6 ise 0'a tekrar ayarlandı.
    i=i-5;
  
  }
  ACSR |= 0x08;                 // Analog interrupt kesmesi aktif.
  OCR1A  = pwm_min_deger;       // Dijital 9  Pini pwm duty cycle değeri ayarlandı.
  OCR1B  = pwm_min_deger;       // Dijital 10 Pini pwm duty cycle değeri ayarlandı.
  OCR2A  = pwm_min_deger;       // Dijital 11 Pini pwm duty cycle değeri ayarlandı.
  motor_hizi=pwm_min_deger;
}

ISR (ANALOG_COMP_vect) {
  
  for(i = 0; i < 5; i++) {           // Komparator çıkışı 10 kez kontrol ediliyor.                //// MİNİMUM 5.. MOTOR GÜZEL HAREKET EDİYOR. 35'DEN AŞAĞI HIZ SIKINTI.
    if(bldc_adim & 1)                      // Eğer adım = tek (0001, 0011, 0101) 1, 3 yada 5 ise;
    {
      if(!(ACSR & B00100000)) i -= 1;        // !B00100000 -> B11011111 ACO = 0 (Analog Comparator Output = 0) (düşen kenar) 
    }
   
    else                              // değilse yani eğer adım (0000,0010,0100) 0, 2 yada 4 ise;
    {
      if((ACSR & B00100000))  i -= 1;         // B00100000 -> B11011111 ACO = 1 (Analog Comparator Output = 1) (yükselen kenar)     
    }   
  }
  
  bldc_hareket();               // BLDC adımı ayarlandı.
  bldc_adim++;                  // bir sonraki adım için değişken ayarlandı.
  bldc_adim %= 6;               // eğer adım 6 ise 0'a tekrar ayarlandı.   
  ACSR  = 0x10;                 // Analog komparator kesme bayrağı temizlendi.  ////// FLAG TEMİZLENDİ.
  ACSR |= 0x08;                 // Analog interrupt kesmesi aktif.              ////// KESME AKTİF EDİLDİ. 
}




void loop() {

  while(motor_hizi < pwm_max_deger){
      
    motor_hizi+=3;
    OCR1A  = motor_hizi;       // Dijital 9  Pini pwm duty cycle değeri ayarlandı.
    OCR1B  = motor_hizi;       // Dijital 10 Pini pwm duty cycle değeri ayarlandı.
    OCR2A  = motor_hizi;       // Dijital 11 Pini pwm duty cycle değeri ayarlandı.
    delay(100);
    }
    delay(3000);

   while(motor_hizi>pwm_min_deger){
    motor_hizi-=3;
    OCR1A  = motor_hizi;       // Dijital 9  Pini pwm duty cycle değeri ayarlandı.
    OCR1B  = motor_hizi;       // Dijital 10 Pini pwm duty cycle değeri ayarlandı.
    OCR2A  = motor_hizi;       // Dijital 11 Pini pwm duty cycle değeri ayarlandı.
    delay(100);
    }
    delay(3000);

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
