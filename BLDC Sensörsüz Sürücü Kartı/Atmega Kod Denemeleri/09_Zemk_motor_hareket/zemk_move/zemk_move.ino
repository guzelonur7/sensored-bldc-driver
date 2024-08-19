/*
 * Y A P I  L A C A K L A R : 
 * Bu çalışmada hız dışardan değil kod ile kontrol edilecektir.
 * Osiloskopla faz gerilimleri ve zıt emk ölçülecektir. 
 * 10nF kondansatör eklenecektir.
 **************************************************************
 * S O N U Ç L A R :
 * KONDANSATÖRLER BAŞARISIZ.
 * ANALOG KOMPARATOR KESME FONK İÇERİSİNDE FLAG TEMİZLENİNCE PİN CHANGE KESMESİ SORUNSUZ ÇALIŞIYOR
 * FAKAT BU KEZ ANALOG KOMPARATOR KESME FONKSİYONUNA GİRMİYOR.
 * 
 * 
 * Ç Ö Z Ü M   D E N E M E S İ 
 * TİMER İLE ZAMANLAYICI KESMESİ KURULACAK.
 * 10uS DE BİR KESME OLACAĞI İÇİN HER KESMEDE ANALOG KOMPARATÖR AKTİF EDİLECEK.
 *  
 * 
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
 * 
*/

// SABİTLER.
#define pwm_max_deger 255
#define pwm_min_deger 75
#define pwm_baslangic 75

// DEĞİŞKENLER.
int pwm_max_giris = 2200;
int pwm_min_giris = 800;
int pwm_giris;

int toplam_sayac=0;

byte bldc_adim=0; 
byte motor_hizi;
byte son_durum_pwm;

unsigned int pwm_toplam;
unsigned int i;
unsigned long zaman, simdiki_zaman;


void setup() {

  Serial.begin(115200);     // Seri haberleşme başalatma fonksiyonu.
  
  DDRD  |= 0x38;           // (B00111000) PD3 PD4 PD5 çıkış ayarlandı. (Low side pinleri)
  PORTD  = 0x00;           // D portu pinleri lojik 0 ayarlandı.
  
  DDRB  &= 0x00;           // b portu pinler giriş.
  DDRB  |= 0x0E;           // (B00001110) PB1 PB2 PB3 çıkış ayarlandı. (High side pinleri)
  
  PORTB  = 0x00;           // B portu pinleri lojik 0 ayarlandı.
  
  TCCR1A = 0;              // Timer1 ayarları. 
  TCCR1B = 0x01;           // Timer1 ayarları. clkI/O / 1 (no prescaling)
  TCCR2A = 0;              // Timer2 ayarları.
  TCCR2B = 0x01;           // Timer2 ayarları. clkI/O / 1 (no prescaling) 
  OCR1A  = pwm_min_deger;       // Dijital 9  Pini pwm duty cycle değeri ayarlandı.
  OCR1B  = pwm_min_deger;       // Dijital 10 Pini pwm duty cycle değeri ayarlandı.
  OCR2A  = pwm_min_deger;       // Dijital 11 Pini pwm duty cycle değeri ayarlandı.
  
  
  /*PCICR |= (1 << PCIE0);        // PCMSK0 taramasını etkinleştir.                                                 
  PCMSK0 |= (1 << PCINT0);      // Pin D8'i durum değişikliğinde bir kesmeyi tetikleyin. 
  */
  ACSR  = 0x10;                 // Analog komparator kesme bayrağı temizlendi.
  SREG |= B10000000;            // Global kesmeler aktif.
  
  motor_hizi=pwm_min_deger;
  
  i=2000;                        
  delay(2500);                  // Başlangıç gecikmesi. 2.5sn
  
  while(i>200){                 // İlk hareketlendirme.  
    delayMicroseconds(i);
    bldc_hareket();                // Adım fonksiyonları.
    bldc_adim++;                   // Bir sonraki adım için değişken ayarlandı.
    bldc_adim %= 6;                // Eğer adım 6 ise 0'a tekrar ayarlandı.
    i=i-1;
  
  }
  ACSR |= 0x08;                 // Analog interrupt kesmesi aktif.
}




ISR (ANALOG_COMP_vect) {
  //ACSR  = 0x10;                 // Analog komparator kesme bayrağı temizlendi.
  for(i = 0; i < 10; i++) {           // Komparator çıkışı 10 kez kontrol ediliyor.
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
  bldc_adim++;                    // bir sonraki adım için değişken ayarlandı.
  bldc_adim %= 6;                 // eğer adım 6 ise 0'a tekrar ayarlandı.   
  //Serial.println(bldc_adim); 
  
}



void loop() {
  //ACSR |= 0x08;                 // Analog interrupt kesmesi aktif.
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


// Hız ayarlama fonksiyonu.
void SET_PWM(byte width_value){
  OCR1A  = width_value;                   // Set pin 9  PWM duty cycle
  OCR1B  = width_value;                   // Set pin 10 PWM duty cycle
  OCR2A  = width_value;                   // Set pin 11 PWM duty cycle
}

/*
ISR(PCINT0_vect){
  
    simdiki_zaman = micros();                      // Micros ile o anki süreyi alıyoruz.
  
    if(PINB & B00000001){                          // Pin high mı kontrol 
      if(son_durum_pwm == 0){                      // Son durum 0'sa durum değişikliği var demektir.  
        son_durum_pwm = 1;                         // Son durumu high yaptık
        zaman = simdiki_zaman;                     // counter_1'e şuanki süreyi atadık.
      }
    }
    else if(son_durum_pwm == 1){                   // Pin low ve last state == 1 ise durum değişikliği var demektir.    
      son_durum_pwm = 0;                           // Son durumu low güncelledik.
      pwm_giris = simdiki_zaman - zaman;           // Şimdiki değeri counter_1'den çıkardık.    
      pwm_toplam =pwm_toplam + pwm_giris;
      toplam_sayac++;

        if(toplam_sayac == 10){
          pwm_giris=pwm_toplam/10;
          
         // pwm_giris = ((pwm_giris-pwm_min_giris)/6.83)+pwm_min_deger;
         // if(pwm_giris>250)
        //  pwm_giris=250;
          
          
          Serial.println(pwm_giris);
          pwm_toplam=0;
          toplam_sayac=0;
          //SET_PWM(pwm_giris);
          }      
    }
}*/
