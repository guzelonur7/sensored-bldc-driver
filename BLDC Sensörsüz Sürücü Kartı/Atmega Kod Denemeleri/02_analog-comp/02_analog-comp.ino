/*  Analog comparator uygulaması tamamlandı. 
 *  Kesmeler için comparator interrupt enable biti ACIE ve tüm kesmeler için kontrol registerı SREG kullanıldı. 
 *  Kenar değişiminde kesme meydana gelmemesi için interrupt pasif edilebilir.
 *  PWM oluşlturmaya bakılacak. 
 *  Algoritma oluşturulacak.
 *  PWM okumaya bakılacak. (Reciver'dan gelen sinyal için)
*/


void setup() {

  ACSR = (1 << ACI);  // ACI biti lojik1 yapıldı ve analog comp. kesmesi aktif edildi.
  ACSR = (0 << ACBG); // ACBG biti 0 ayarlandığında pozitif giriş D6 pini olarak ayarlanmış oldu.

  ADCSRA = (0 << ADEN); // Negatif girişin mux kullanılması için ADC modülü pasif edildi. ADEN biti 0 ayarlandı.
  ADCSRB = (1 << ACME); // ACME biti analog comp. multiplexer'ını aktif etmek için 1 ayarlandı.

  ADMUX = 0;  // ADMUX registerına 0-7 arası değer yüklenerek mux0..2 bitleri ayarlanmış ve negatif giriş seçilmiş olur. ADC0 negatif input.

  ACSR |= B00000011; // Analog comparator yükselen kenarda kesme oluşturacak.

  DDRD |=B00011000;  // D3 VE D4 PİNİ REGİSTER İLE ÇIKIŞ AYARLANDI. 

  ACSR = (1 << ACIE);
  SREG |= 0x80 ;
}

void loop() {
 
}

ISR(ANALOG_COMP_vect){

  if(ACSR & B00100000){ // ACSR registerının o anki değeri 00100000 ile toplandı. ACO 1 ise yükselen kenar tespit edildi.  
  PORTD |=B00001000; // D3 pini lojik 1 çıkış ayarlandı. yükselen kenar tespit ledi.
  delay(100000);
  PORTD &= 0x00;  // D3 pini lojik 0 ayarlandı.
  ACSR &= B11111110;// comp. kesmesi  düşen kenar ayarlandı.
  } 

   else if(!(ACSR & B00100000)) // ACO 0 ise düşen kenar tespit edildi demektir.
   {
    PORTD |=B00010000; // D4 pini lojik 1 çıkış ayarlandı. düşen kenar tespit ledi.
    delay(100000);
    PORTD &= 0x00;  // D3 pini lojik 0 ayarlandı.
    ACSR |=B00000011; // COMP. kesmesi yükselen kenar ayarlandı. 
  }
  
}
