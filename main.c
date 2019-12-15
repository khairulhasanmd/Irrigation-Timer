//**************************************************
//  Author    - Md. Khairul Hasan
//  Contact   - +880-1719-328628, 
//  e-mail    - kairulhasanmd@gmail.com
//  Device    - Hall effect sensor based motorcycle gear indicator.
// **************************************************

// avrdude -p atmega8 -c usbasp -t -B 3
// dump eeprom 0 512
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 1000000UL
#include <util/delay.h>


#define SEGMENT_SCAN_DELAY 5
// #define ENCODER_CLK 2       //clk
// #define ENCODER_DATA 3       //dt
// #define ENCODER_SWITCH 4  //push button switch
#define EEPROM_STORAGE_ADDRESS 0x08
#define SEGMENT_DOT PB7
#define POWER_OFF_DETECT PD1
#define CLK PD2
#define DAT PD3
#define BTN PD4
//modes
#define HOUR_SET 1
#define MINUTE_SET 2
#define SECOND_SET 3
#define RUN 4

volatile uint8_t secondFlag;
uint8_t buttonPressed = 0, remHr = 0, remMin = 0, remSec = 0, MODE = 0; 
uint8_t states_of_segment[16] = {~0x3F, ~0x06, ~0x5B, ~0x4F, ~0x66, ~0x6D, ~0x7D, ~0x07, ~0x7F, ~0x6F, ~0x77, ~0x7C, ~0x39, ~0x5E, ~0x79, ~0x71}; //abcdefg
uint8_t segment[4] = {1,2,3,4};
int MSB, LSB, encoded, sum;
int lastEncoded = 0, encoderValue = 0;

void updateEncoder(void){
  MSB = 0;
  LSB = 0;
  if(PIND & (1 << PD2)){
    MSB = 1;
  }
  if(PIND & (1 << PD3)){
    LSB = 1;
  }
  encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number 
  sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
  lastEncoded = encoded; //store this value for next time 
}

void update_display(void){
  //display the values
  if(encoderValue > 1){
    remSec++;
    encoderValue = 0;
  }else if(encoderValue < -1){
    remSec--;
    encoderValue = 0;
  }
  // remSec = encoderValue;
  segment[3] = remSec%10;
  segment[2] = remSec/10;
  segment[1] = remSec%10;
  segment[0] = remSec/10;


  PORTB = 0x00;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[3]]);
  PORTC |= 0b00100000; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);

  PORTB = 0x00;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[2]]);
  PORTC |= 0b00010000; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);

  PORTB = 0x00;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[1]]);
  PORTC |= 0b00001000; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
  PORTB = 0x00;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[0]]);
  PORTC |= 0b00000100; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
}

ISR(TIMER1_COMPA_vect)
{
  secondFlag = 1;
}

ISR (INT0_vect)
{
  updateEncoder();
}

ISR (INT1_vect)
{
  updateEncoder();
}

int main (void)
{
  //initialize the I/O Ports
  DDRD &= ~((1 << CLK)|(1 << DAT)|(1 << BTN)|(1 << POWER_OFF_DETECT));    // switch on pin (GEAR_SHIFT_UP)
  PORTD |= (1 << CLK)|(1 << DAT)|(1 << BTN)|(1 << POWER_OFF_DETECT);    // enable pull-up resistor
  DDRC |= (1 << PC5)|(1 << PC4)|(1 << PC3)|(1 << PC2); //common anode
  DDRB |= (1 << PB0)|(1 << PB1)|(1 << PB2)|(1 << PB3)|(1 << PB4)|(1 << PB5)|(1 << PB6)|(1 << SEGMENT_DOT); //seven segment gbcdefa.

  TCCR1B |= (1 << WGM12);           // Configure timer 1 for CTC mode
  TIMSK |= (1 << OCIE1A);           // Enable CTC interrupt
  OCR1A = 15624;                // Set CTC compare value to 1Hz at 1MHz AVR clock, with a prescaler of 64
  TCCR1B |= ((1 << CS10) | (1 << CS11));  // Start timer at Fcpu/64


  remSec = 99;
  //READ FROM EEPROM
  // valueRead = eeprom_read_byte((uint8_t*)EEPROM_STORAGE_ADDRESS);//read eeprom
  // if (valueRead > 0x0F)//initial value.. omit it 
  // {
  //   gearCount = 0;
  // }else{//has some value and its larger 
  //   gearCount = valueRead;
  // }
  MCUCR |= (1 << ISC00);    // set INT0 to trigger on ANY logic change
  GICR |= ((1 << INT0)|(1 << INT1));      // Turns on INT0 & int1

  sei();//Enable Global Interrupts

  while (1)//main loop 
  {
    if (secondFlag){
      secondFlag = 0;
      // remSec--;
      // PORTB ^= (1 << SEGMENT_DOT);    // toggles the led
    }
    update_display();
  //   //hall effect sensor functionality
  //   if ((PIND & (1 << GEAR_SHIFT_UP)) && upState == 1){//reset up debounce
  //     upState = 0;
  //   }else if ((PIND & (1 << GEAR_SHIFT_DOWN)) && dwnState == 1){ //reset down debounce
  //     dwnState = 0;
  //   } else if ((!(PIND & (1 << GEAR_SHIFT_UP))) && upState == 0){  //if shift up detected
  //     gearCount ++;
  //     if(gearCount > 15){
  //       gearCount = 0;
  //     }
  //     PORTB |= (1 << SEGMENT_DOT);    // light up the dot marker
  //     upState = 1;
  //     update_display();
  //   }else if ((!(PIND & (1 << GEAR_SHIFT_DOWN))) && dwnState == 0){  //if shift down detected
  //     gearCount --;
  //     if(gearCount < 0 || gearCount > 15){
  //       gearCount = 15;
  //     }
  //     PORTB |= (1 << SEGMENT_DOT);    // light up the dot marker
  //     dwnState = 1;
  //     update_display();
  //   }else if (!(PIND & (1 << RESET_ALL))){  //if reset button pressed
  //     //reset the gear state stored in eeprom if it has any previous value
  //     cli();//Disable Global Interrupts
  //     valueRead = eeprom_read_byte((uint8_t*)EEPROM_STORAGE_ADDRESS);//read eeprom
  //     eeprom_write_byte((uint8_t*)EEPROM_STORAGE_ADDRESS,0);//write eeprom
  //     gearCount = 0;
  //     PORTB |= (1 << SEGMENT_DOT);    // light up the dot marker
  //     _delay_ms(1000);
  //     sei();//Enable Global Interrupts
  //   }

  //   //-------------------------------POWER DOWN, SAVE IMMEDIATELY TO EEPROM
  //   if (!(PIND & (1 << POWER_OFF_DETECT))){ //active low
  //     cli();//Disable Global Interrupts
  //     eeprom_write_byte((uint8_t*)EEPROM_STORAGE_ADDRESS, gearCount);//write eeprom
  //     PORTB |= (1 << SEGMENT_DOT);    // light up the dot marker
  //     _delay_ms(500); //delay till the capacitor discharge
  //     sei();//Enable Global Interrupts
  //   }
  }
}