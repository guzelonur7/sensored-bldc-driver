/*
*  Motor ses uyarı kodları denemeleri. 
*  
*  Beepler çalışıyor. İstenildiği gibi uyarlanabilir.
*  
*/

#include "Beeps.h"

void setup() {      //// setup bir kez çalıştığı için ilk çalışmada max ve min pwm değerleri tanıtılması gerekmektedir.
  
  Serial.begin(115200);
  
  DDRD  |= 0x38;                // PD3 PD4 PD5 çıkış ayarlandı. (Low side pinleri)
  PORTD  = 0x00;                // D portu pinleri lojik 0 ayarlandı.
  DDRB  |= 0x0E;                // PB1 PB2 PB3 çıkış ayarlandı. (High side pinleri)
  PORTB  = 0x00;                // B portu pinleri lojik 0 ayarlandı.
  
  TCCR1A = 0;                   // Timer1 ayarları. 
  TCCR1B = 0x01;                // clkI/O / 1 (8 prescaling)
  TCCR2A = 0;                   // Timer2 ayarları.
  TCCR2B = 0x01;                // clkI/O / 1 (8 prescaling) 


}


void loop() { 
  
  Serial.println("PWM ARALIK MODU");
  beep_1KHZ(100);
  delay(500);
  beep_1KHZ(100);
  delay(500);
  beep_1KHZ(100);
  delay(500);
  beep_1KHZ(100);
  delay(500);  
  
  Serial.println("NORMAL MOD BASLANGIC SESI");
  beep_1KHZ(50);
  delay(100);
  beep_2KHZ(50);
  delay(100);
  beep_3KHZ(50);
  delay(100); 
  delay(5000);

  

   
}
  
