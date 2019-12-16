//**************************************************
//  Author    - Md. Khairul Hasan
//  Contact   - +880-1719-328628, 
//  e-mail    - kairulhasanmd@gmail.com
//  Device    - seven segment and rotary encoder based industrial timer.
// **************************************************

// avrdude -p atmega8 -c usbasp -t -B 3
// dump eeprom 0 512
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 1000000UL
#include <util/delay.h>

#define SEGMENT_SCAN_DELAY 5
#define SEGMENT_TOTAL_TIME 30
#define SEGMENT_BLINK_DELAY 15

#define EEPROM_SECOND_SET_ADDRESS 0x00
#define EEPROM_MINUTE_SET_ADDRESS 0x01
#define EEPROM_HOUR_SET_ADDRESS 0x02

#define SEGMENT_DOT PB7
#define POWER_OFF_DETECT PD1
#define CLK PD2
#define DAT PD3
#define BTN PD4
//modes
#define RUN 0
#define SECOND_SET 1
#define MINUTE_SET 2
#define HOUR_SET 3

volatile uint8_t secondFlag;
uint8_t buttonPressed = 0, curHr = 0, curMin = 0, curSec = 0, STATE = 0; 
uint8_t states_of_segment[16] = {~0x3F, ~0x06, ~0x5B, ~0x4F, ~0x66, ~0x6D, ~0x7D, ~0x07, ~0x7F, ~0x6F, ~0x77, ~0x7C, ~0x39, ~0x5E, ~0x79, ~0x71}; //abcdefg
uint8_t segment[4] = {0,0,0,0};
int MSB, LSB, encoded, sum;
int lastEncoded = 0, encoderValue = 0, segmentBlink = 0;

void waitForRelease(){
  while(!(PIND & (1 << BTN))){//button held down
      // lcd.clear();
      _delay_ms(10);
  }
}

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
  // last 0088 segment
  segment[3] = curSec%10;
  segment[2] = curSec/10;
  //first 8800 segment
  segment[1] = curMin%10;
  segment[0] = curMin/10;
  PORTB &= 0x80;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[3]]);
  PORTC |= 0b00100000; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
  PORTB &= 0x80;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[2]]);
  PORTC |= 0b00010000; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
  PORTB &= 0x80;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[1]]);
  PORTC |= 0b00001000; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
  PORTB &= 0x80;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[0]]);
  PORTC |= 0b00000100; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
}

void blink_display_last2digit(void){
  segmentBlink ++;
  if(segmentBlink > SEGMENT_TOTAL_TIME){
    segmentBlink = 0;
  }
  // last 0088 segment
  segment[3] = curSec%10;
  segment[2] = curSec/10;
  if (segmentBlink < SEGMENT_BLINK_DELAY){//on time
    PORTB &= 0x80;
    PORTC &= 0b00000011; //common anode
    PORTB |= (states_of_segment[segment[3]]);
    PORTC |= 0b00100000; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
    PORTB &= 0x80;
    PORTC &= 0b00000011; //common anode
    PORTB |= (states_of_segment[segment[2]]);
    PORTC |= 0b00010000; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
  }else if(segmentBlink > SEGMENT_BLINK_DELAY){//off time
    PORTB &= 0x80;
    PORTC &= 0b00000011; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
    PORTB &= 0x80;
    PORTC &= 0b00000011; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
  }
  PORTB &= 0x80;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[1]]);
  PORTC |= 0b00001000; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
  PORTB &= 0x80;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[0]]);
  PORTC |= 0b00000100; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
}

void blink_display_first2digit(void){
  segmentBlink ++;
  if(segmentBlink > SEGMENT_TOTAL_TIME){
    segmentBlink = 0;
  }
  //first 8800 segment
  segment[1] = curMin%10;
  segment[0] = curMin/10;
  PORTB &= 0x80;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[3]]);
  PORTC |= 0b00100000; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
  PORTB &= 0x80;
  PORTC &= 0b00000011; //common anode
  PORTB |= (states_of_segment[segment[2]]);
  PORTC |= 0b00010000; //common anode
  _delay_ms(SEGMENT_SCAN_DELAY);
  if (segmentBlink < SEGMENT_BLINK_DELAY){//on time
    PORTB &= 0x80;
    PORTC &= 0b00000011; //common anode
    PORTB |= (states_of_segment[segment[1]]);
    PORTC |= 0b00001000; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
    PORTB &= 0x80;
    PORTC &= 0b00000011; //common anode
    PORTB |= (states_of_segment[segment[0]]);
    PORTC |= 0b00000100; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
  }else if(segmentBlink > SEGMENT_BLINK_DELAY){//off time
    PORTB &= 0x80;
    PORTC &= 0b00000011; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
    PORTB &= 0x80;
    PORTC &= 0b00000011; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
  }
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

  STATE = RUN;
  //READ FROM EEPROM
  curSec = eeprom_read_byte((uint8_t*)EEPROM_SECOND_SET_ADDRESS);//read eeprom
  curMin = eeprom_read_byte((uint8_t*)EEPROM_MINUTE_SET_ADDRESS);//read eeprom
  curHr = eeprom_read_byte((uint8_t*)EEPROM_HOUR_SET_ADDRESS);//read eeprom
  if (curSec > 99) curSec = 0;//initial value.. omit it 
  if (curMin > 99) curMin = 0;//initial value.. omit it 
  if (curHr > 99) curHr = 0;//initial value.. omit it 

  //interrupt
  MCUCR |= (1 << ISC00);    // set INT0 to trigger on ANY logic change
  GICR |= ((1 << INT0)|(1 << INT1));      // Turns on INT0 & int1
  sei();//Enable Global Interrupts

  while (1)//main loop ---------------------------------------------------------------------------------------
  {
    if(!(PIND & (1 << BTN))){//button pressed
      STATE++;
      if (STATE > MINUTE_SET){ //FIX IT WHEN HOUR ENABLED
        STATE = RUN;
        eeprom_write_byte((uint8_t*)EEPROM_SECOND_SET_ADDRESS, curSec);//write eeprom
        eeprom_write_byte((uint8_t*)EEPROM_MINUTE_SET_ADDRESS, curMin);//write eeprom
        eeprom_write_byte((uint8_t*)EEPROM_HOUR_SET_ADDRESS, curHr);//write eeprom
      }
      waitForRelease();
    }

    if (STATE == RUN){
      if (secondFlag){
        secondFlag = 0;
        // PORTB ^= (1 << SEGMENT_DOT);    // toggles the led
        if(curSec > 0 ){
          curSec--;
        }else if(curMin > 0 && curSec == 0){
          curSec = 59;
          curMin --;
        }else if(curSec == 0 && curMin == 0){
          //do anything
        }
      }
      update_display();
    } else if (STATE == SECOND_SET){
      if(encoderValue > 1){
        curSec++;
        encoderValue = 0;
        segmentBlink = 0;
      }else if(encoderValue < -1){
        curSec--;
        encoderValue = 0;
        segmentBlink = 0;
      }
      blink_display_last2digit();
    }else if (STATE == MINUTE_SET){
      if(encoderValue > 1){
        curMin++;
        encoderValue = 0;
      }else if(encoderValue < -1){
        curMin--;
        encoderValue = 0;
      }
      blink_display_first2digit();
    }else if (STATE == HOUR_SET){
      if(encoderValue > 1){
        curHr++;
        encoderValue = 0;
      }else if(encoderValue < -1){
        curHr--;
        encoderValue = 0;
      }
      blink_display_first2digit();//needs fix
    }

    //-------------------------------POWER DOWN, SAVE IMMEDIATELY TO EEPROM
    // if (!(PIND & (1 << POWER_OFF_DETECT))){ //active low
    //   cli();//Disable Global Interrupts
    //   eeprom_write_byte((uint8_t*)EEPROM_SECOND_SET_ADDRESS, curSec);//write eeprom
    //   eeprom_write_byte((uint8_t*)EEPROM_MINUTE_SET_ADDRESS, curMin);//write eeprom
    //   eeprom_write_byte((uint8_t*)EEPROM_HOUR_SET_ADDRESS, curHr);//write eeprom
    //   PORTB |= (1 << SEGMENT_DOT);    // light up the dot marker
    //   _delay_ms(500); //delay till the capacitor discharge
    //   sei();//Enable Global Interrupts
    // }
  }
}