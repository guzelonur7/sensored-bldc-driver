void setup() {
  
   //pinMode(3,OUTPUT);  // D3 PİNİ KÜTÜPHANE İLE ÇIKIŞ AYARLANDI.
  DDRD |=B00001000;  // D3 PİNİ REGİSTER İLE ÇIKIŞ AYARLANDI. 

}

void loop() {

  /*digitalWrite(3,HIGH); // D3 PİNİ LOJİK 1 AYARLANDI.
  digitalWrite(3,LOW); // D3 PİNİ LOJİK 1 AYARLANDI.*/ 
  
  PORTD |=B00001000; // D3 PİNİ REGİSTER İLE LOJİK1 AYARLANDI.
  PORTD &=B00000000;

}
