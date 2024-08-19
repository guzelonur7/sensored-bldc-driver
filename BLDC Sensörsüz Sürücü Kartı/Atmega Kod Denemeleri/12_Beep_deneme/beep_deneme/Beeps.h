/* 
 *  MOTOR BUZZER KÜTÜPHANESİ..
 *
 *  LOW-A: PD5    |   LOW-B: PD4    |   LOW-C: PD3 
 *  HIGH-A: PB1   |   HIGH-B: PB2   |   HIGH-C: PB3
 *  
*/



void beep_1KHZ (int milliseconds)
{
  int beep_sayac = 0;
  PORTD = B00001000;                  // LOW-C aktif. PD3 
  while (beep_sayac < milliseconds)
  { 
    PORTB = B00000010;                // HIGH-A aktif. PB1
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-A pasif. PB1
    delayMicroseconds(450);
    
    PORTB = B00000100;                // HIGH-B aktif. PB2
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-B pasif. PB2
    delayMicroseconds(450);
    beep_sayac++;
  }
   PORTD = B00000000;                 // LOW tümü pasif.
   PORTB = B00000000;                 // HIGH tümü pasif.
}


void beep_2KHZ (int milliseconds)
{
  int beep_sayac = 0;
  PORTD = B00001000;                  // LOW-C aktif. PD3 
  while (beep_sayac < milliseconds)
  { 
    PORTB = B00000010;                // HIGH-A aktif. PB1
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-A pasif. PB1
    delayMicroseconds(200);
    
    PORTB = B00000100;                // HIGH-B aktif. PB2 
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-B pasif. PB2
    delayMicroseconds(200);

    PORTB = B00000010;                // HIGH-A aktif. PB1
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-A pasif. PB1
    delayMicroseconds(200);
    
    PORTB = B00000100;                // HIGH-B aktif. PB2
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-B pasif. PB2 
    delayMicroseconds(200);
    beep_sayac++;
  }
   PORTD = B00000000;                 // LOW tümü pasif.
   PORTB = B00000000;                 // HIGH tümü pasif.
}


void beep_3KHZ (int milliseconds)
{
  int beep_sayac = 0;
  PORTD = B00001000;                  //Set D2 (CL) to HIGH and the rest to LOW
  while (beep_sayac < milliseconds)
  { 
    PORTB = B00000010;                // HIGH-A aktif. PB1
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-A pasif. PB1
    delayMicroseconds(150);

    PORTB = B00000010;                // HIGH-A aktif. PB1
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-A pasif. PB1
    delayMicroseconds(150);

    PORTB = B00000010;                // HIGH-A aktif. PB1
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-A pasif. PB1
    delayMicroseconds(150);

    PORTB = B00000010;                // HIGH-A aktif. PB1
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-A pasif. PB1
    delayMicroseconds(150);

    PORTB = B00000010;                // HIGH-A aktif. PB1
    delayMicroseconds(50);
    PORTB = B00000000;                // HIGH-A pasif. PB1
    delayMicroseconds(150);
    beep_sayac++;
  }
   PORTD = B00000000;                 // LOW tümü pasif.
   PORTB = B00000000;                 // HIGH tümü pasif. 
}
