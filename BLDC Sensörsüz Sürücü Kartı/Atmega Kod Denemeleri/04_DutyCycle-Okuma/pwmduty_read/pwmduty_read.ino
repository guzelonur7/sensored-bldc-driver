/*
 * atmega 328p için duty cycle okuma çalışması.
 *  
 *  Harici bir pwm kaynağı ile(pic18f4550) sistem denendi. kod çalışıyor 
 *  pwm hesabında 200 eklendi. osiloskop ile pwm frekansının duty değeri kontrol edilecek.
 *  
 */
 
#define PWM_max_value      255
#define PWM_min_value      50

int PWM_IN_MAX = 2200;
int PWM_IN_MIN = 800;
int PWM_INPUT;
byte last_PWM_state=0, motor_speed;
unsigned long current_count,counter_1; 

void setup() {
  Serial.begin(9600);       // seri monitör hazırlandı.
  
  DDRB  &= 0x00;            // b portu pinler giriş.
  PCICR |= (1 << PCIE0);    //PCMSK0 taramasını etkinleştir.                                                 
  PCMSK0 |= (1 << PCINT0);  //Pin D8'i durum değişikliğinde bir kesmeyi tetikleyin.
  SREG |= B10000000;        // global kesmeler aktif.
}

void loop() {
  if(PWM_INPUT >= PWM_IN_MAX){
      PWM_INPUT = PWM_IN_MAX;
      }
    else if(PWM_INPUT <= PWM_IN_MIN){
      PWM_INPUT = PWM_IN_MIN;
      }
  motor_speed = ((PWM_INPUT-PWM_IN_MIN)/6.83)+PWM_min_value;
  Serial.println(motor_speed);                       // PWM duty değeri ekrana yazıldı. 
}

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
    PWM_INPUT = current_count - counter_1;           // Şimdiki değeri counter_1'den çıkardık. 
  }
}
