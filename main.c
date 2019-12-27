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
#define F_CPU                             1000000UL
#include <util/delay.h>

#define SEGMENT_SCAN_DELAY                3
#define SEGMENT_TOTAL_TIME                30
#define SEGMENT_BLINK_DELAY               15

#define RELAY                             PD0
#define POWER_OFF_DETECT                  PD1
#define CLK                               PD2
#define DAT                               PD3
#define BTN                               PD4
#define START_BUTTON                      PD5

#define EEPROM_SECOND_SET_ADDRESS         0x00
#define EEPROM_MINUTE_SET_ADDRESS         0x01
#define EEPROM_HOUR_SET_ADDRESS           0x02
#define EEPROM_TEMP_SECOND_SET_ADDRESS    0x03
#define EEPROM_TEMP_MINUTE_SET_ADDRESS    0x04
#define EEPROM_TEMP_HOUR_SET_ADDRESS      0x05

#define RUN                               0
#define HOUR_SET                          1
#define MINUTE_SET                        2
#define SECOND_SET                        3
#define STANDBY                           4

#define POWER_WAS_NOT_GONE                0
#define POWER_WAS_GONE                    1

#define SECOND_ONE                        5
#define SECOND_TEN                        4
#define MINUTE_ONE                        3
#define MINUTE_TEN                        2
#define HOUR_ONE                          1
#define HOUR_TEN                          0

#define DISP_SECOND_ONE                   2
#define DISP_SECOND_TEN                   3
#define DISP_MINUTE_ONE                   4
#define DISP_MINUTE_TEN                   5
#define DISP_HOUR_ONE                     6
#define DISP_HOUR_TEN                     7

#define SEGMENT_DOT_PIN                   0X80

volatile uint8_t secondFlag;
uint8_t buttonPressed = 0, powerState = 0, dotSegment = 0; 
int curHr = 0, curMin = 0, curSec = 0, STATE = 0; 
int savedHr = 0, savedMin = 0, savedSec = 0; 
const uint8_t states_of_segment[16] = {~0x3F, ~0x06, ~0x5B, ~0x4F, ~0x66, ~0x6D, ~0x7D, ~0x07, ~0x7F, ~0x6F, ~0x77, ~0x7C, ~0x39, ~0x5E, ~0x79, ~0x71}; //abcdefg
uint8_t segment[6] = {0,0,0,0,0,0};
int MSB = 0, LSB = 0, encoded = 0, sum = 0, lastEncoded = 0, encoderValue = 0, segmentBlink = 0;

void waitForRelease(void){
  while(!(PIND & (1 << BTN))){//button held down;
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

void calculate_segment_digits(void){
  // last 0088 segment
  segment[5] = curSec%10;
  segment[4] = curSec/10;
  // last 0088 segment
  segment[3] = curMin%10;
  segment[2] = curMin/10;
  //first 8800 segment
  segment[1] = curHr%10;
  segment[0] = curHr/10;
}

void display_second_digits(void){
  PORTC = 0b00000000; //common anode
  PORTB = (states_of_segment[segment[SECOND_TEN]]);
  PORTC = 0b00010000; //common anode
  PORTB &= ~(SEGMENT_DOT_PIN & (dotSegment << SECOND_TEN));
  _delay_ms(SEGMENT_SCAN_DELAY);
  PORTC = 0b00000000; //common anode
  PORTB = (states_of_segment[segment[SECOND_ONE]]);
  PORTC = 0b00100000; //common anode
  PORTB &= ~(SEGMENT_DOT_PIN & (dotSegment << SECOND_ONE));
  _delay_ms(SEGMENT_SCAN_DELAY);
}

void display_minute_digits(void){
  PORTC = 0b00000000; //common anode
  PORTB = (states_of_segment[segment[MINUTE_TEN]]);
  PORTC = 0b00000100; //common anode
  PORTB &= ~(SEGMENT_DOT_PIN & (dotSegment << MINUTE_TEN));
  _delay_ms(SEGMENT_SCAN_DELAY);
  PORTC = 0b00000000; //common anode
  PORTB = (states_of_segment[segment[MINUTE_ONE]]);
  PORTC = 0b00001000; //common anode
  PORTB &= ~(SEGMENT_DOT_PIN & (dotSegment << MINUTE_ONE));
  _delay_ms(SEGMENT_SCAN_DELAY);
}

void display_hour_digits(void){
  PORTC = 0b00000000; //common anode
  PORTB = (states_of_segment[segment[HOUR_TEN]]);
  PORTC = 0b00000001; //common anode
  PORTB &= ~(SEGMENT_DOT_PIN & (dotSegment << HOUR_TEN));
  _delay_ms(SEGMENT_SCAN_DELAY);
  PORTC = 0b00000000; //common anode
  PORTB = (states_of_segment[segment[HOUR_ONE]]);
  PORTC = 0b00000010; //common anode
  PORTB &= ~(SEGMENT_DOT_PIN & (dotSegment << HOUR_ONE));
  _delay_ms(SEGMENT_SCAN_DELAY);
}

void update_display(void){
  calculate_segment_digits();
  display_hour_digits();
  display_minute_digits();
  display_second_digits();
}

void blink_display_second(void){
  calculate_segment_digits();
  dotSegment = ((1 << DISP_SECOND_TEN) | (1 << DISP_SECOND_ONE));
  segmentBlink ++;
  if(segmentBlink > SEGMENT_TOTAL_TIME){
    segmentBlink = 0;
  }
  display_hour_digits();
  display_minute_digits();
  if (segmentBlink < SEGMENT_BLINK_DELAY){//on time
    display_second_digits();
  }else if(segmentBlink > SEGMENT_BLINK_DELAY){//off time
    PORTB &= 0x80;
    PORTC &= 0b00000000; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
    PORTB &= 0x80;
    PORTC &= 0b00000000; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
  }
}

void blink_display_minute(void){
  calculate_segment_digits();
  dotSegment = ((1 << DISP_MINUTE_TEN) | (1 << DISP_MINUTE_ONE));
  segmentBlink ++;
  if(segmentBlink > SEGMENT_TOTAL_TIME){
    segmentBlink = 0;
  }
  display_hour_digits();
  if (segmentBlink < SEGMENT_BLINK_DELAY){//on time
    display_minute_digits();
  }else if(segmentBlink > SEGMENT_BLINK_DELAY){//off time
    PORTB &= 0x80;
    PORTC &= 0b00000000; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
    PORTB &= 0x80;
    PORTC &= 0b00000000; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
  }
  display_second_digits();
}

void blink_display_hour(void){
  calculate_segment_digits();
  dotSegment = ((1 << DISP_HOUR_TEN) | (1 << DISP_HOUR_ONE));
  segmentBlink ++;
  if(segmentBlink > SEGMENT_TOTAL_TIME){
    segmentBlink = 0;
  }
  if (segmentBlink < SEGMENT_BLINK_DELAY){//on time
    display_hour_digits();
  }else if(segmentBlink > SEGMENT_BLINK_DELAY){//off time
    PORTB &= 0x80;
    PORTC &= 0b00000000; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
    PORTB &= 0x80;
    PORTC &= 0b00000000; //common anode
    _delay_ms(SEGMENT_SCAN_DELAY);
  }
  display_minute_digits();
  display_second_digits();
}

void clear_loadshedding_remaining_time(void){
  cli();//Disable Global Interrupts
  eeprom_write_byte((uint8_t*)EEPROM_TEMP_SECOND_SET_ADDRESS, 0);//clear all lodshedding val
  eeprom_write_byte((uint8_t*)EEPROM_TEMP_MINUTE_SET_ADDRESS, 0);//clear all lodshedding val
  eeprom_write_byte((uint8_t*)EEPROM_TEMP_HOUR_SET_ADDRESS, 0);//clear all lodshedding val
  sei();//Enable Global Interrupts
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
  DDRD &= ~((1 << CLK)|(1 << DAT)|(1 << BTN)|(1 << POWER_OFF_DETECT)|(1 << START_BUTTON));    // input
  DDRD |= (1 << RELAY);    // load
  PORTD |= (1 << CLK)|(1 << DAT)|(1 << BTN)|(1 << POWER_OFF_DETECT)|(1 << START_BUTTON);    // enable pull-up resistor
  DDRC |= (1 << PC5)|(1 << PC4)|(1 << PC3)|(1 << PC2)|(1 << PC1)|(1 << PC0); //common anode
  DDRB |= (1 << PB0)|(1 << PB1)|(1 << PB2)|(1 << PB3)|(1 << PB4)|(1 << PB5)|(1 << PB6)|(1 << PB7); //seven segment gbcdefa.

  TCCR1B |= (1 << WGM12);           // Configure timer 1 for CTC mode
  TIMSK |= (1 << OCIE1A);           // Enable CTC interrupt
  OCR1A = 15624;                // Set CTC compare value to 1Hz at 1MHz AVR clock, with a prescaler of 64
  TCCR1B |= ((1 << CS10) | (1 << CS11));  // Start timer at Fcpu/64
  
  savedSec = eeprom_read_byte((uint8_t*)EEPROM_SECOND_SET_ADDRESS);//read eeprom
  savedMin = eeprom_read_byte((uint8_t*)EEPROM_MINUTE_SET_ADDRESS);//read eeprom
  savedHr = eeprom_read_byte((uint8_t*)EEPROM_HOUR_SET_ADDRESS);//read eeprom
  if (savedSec > 59) savedSec = 0;//initial value.. omit it 
  if (savedMin > 59) savedMin = 0;//initial value.. omit it 
  if (savedHr > 99) savedHr = 0;//initial value.. omit it 
  //load loadshedding value
  curSec = eeprom_read_byte((uint8_t*)EEPROM_TEMP_SECOND_SET_ADDRESS);//read eeprom
  curMin = eeprom_read_byte((uint8_t*)EEPROM_TEMP_MINUTE_SET_ADDRESS);//read eeprom
  curHr = eeprom_read_byte((uint8_t*)EEPROM_TEMP_HOUR_SET_ADDRESS);//read eeprom
  if (curSec > 59) curSec = 0;//initial value.. omit it 
  if (curMin > 59) curMin = 0;//initial value.. omit it 
  if (curHr > 99) curHr = 0;//initial value.. omit it 

  if(curSec > 0 || curMin > 0 || curHr > 0){//loadshedding
    STATE = RUN;
    powerState = POWER_WAS_GONE;
  }else{                                    //no loadshedding
    STATE = STANDBY;
    powerState = POWER_WAS_NOT_GONE;
  }

  dotSegment = 0xff;

  //interrupt
  MCUCR |= (1 << ISC00);    // set INT0 to trigger on ANY logic change
  GICR |= ((1 << INT0)|(1 << INT1));      // Turns on INT0 & int1
  sei();//Enable Global Interrupts

  while (1)//main loop ---------------------------------------------------------------------------------------
  {
    if(!(PIND & (1 << BTN))){//button pressed
      STATE++;
      if (STATE == HOUR_SET){
        curSec = savedSec;
        curMin = savedMin;
        curHr = savedHr;
        if(powerState == POWER_WAS_GONE){
          powerState = POWER_WAS_NOT_GONE;
          clear_loadshedding_remaining_time();
        }
      }
      if (STATE == STANDBY){//set everything
        savedSec = curSec;
        savedMin = curMin;
        savedHr = curHr;
        cli();//Disable Global Interrupts
        eeprom_write_byte((uint8_t*)EEPROM_SECOND_SET_ADDRESS, curSec);//write eeprom
        eeprom_write_byte((uint8_t*)EEPROM_MINUTE_SET_ADDRESS, curMin);//write eeprom
        eeprom_write_byte((uint8_t*)EEPROM_HOUR_SET_ADDRESS, curHr);//write eeprom
        sei();//Enable Global Interrupts
      }
      if (STATE > STANDBY){
        STATE = HOUR_SET;
      }
      waitForRelease();
    }

    if (STATE == RUN){
      if (secondFlag){  //turn on relay
        secondFlag = 0;
        if(curSec > 0 ){
          curSec--;
          PORTD |= (1 << RELAY);
        }else if(curMin > 0 && curSec == 0){ //turn on relay
          PORTD |= (1 << RELAY);
          curSec = 59;
          curMin --;
        }else if(curHr > 0 && curMin == 0 && curSec == 0){ //turn on relay
          PORTD |= (1 << RELAY);
          curSec = 59;
          curMin = 59;
          curHr --;
        }else if(curHr == 0 && curMin == 0 && curSec == 0){ //Turn off relay
          PORTD &= ~(1 << RELAY);
          if(powerState == POWER_WAS_GONE){
            powerState = POWER_WAS_NOT_GONE;
            STATE = STANDBY;
            clear_loadshedding_remaining_time();
          }
        }
      }
      update_display();
    } else if (STATE == STANDBY){
      //no work to be done. just standby and check for start button press
      curSec = 0;
      curMin = 0;
      curHr = 0;
      if (!(PIND & (1 << START_BUTTON))){
        curSec = savedSec;
        curMin = savedMin;
        curHr = savedHr;
        STATE = RUN;
      }
      update_display();
    } else if (STATE == SECOND_SET){
      if(encoderValue > 1){
        curSec++;
        if (curSec > 59){
          curSec = 0;
        }
        encoderValue = 0;
        segmentBlink = 0;
      }else if(encoderValue < -1){
        curSec--;
        if (curSec < 0){
          curSec = 59;
        }
        encoderValue = 0;
        segmentBlink = 0;
      }
      blink_display_second();
    }else if (STATE == MINUTE_SET){
      if(encoderValue > 1){
        curMin++;
        if (curMin > 59){
          curMin = 0;
        }
        encoderValue = 0;
        segmentBlink = 0;
      }else if(encoderValue < -1){
        curMin--;
        if (curMin < 0){
          curMin = 59;
        }
        encoderValue = 0;
        segmentBlink = 0;
      }
      blink_display_minute();
    }else if (STATE == HOUR_SET){
      if(encoderValue > 1){
        curHr++;
        if (curHr > 99){
          curHr = 0;
        }
        encoderValue = 0;
        segmentBlink = 0;
      }else if(encoderValue < -1){
        curHr--;
        if (curHr < 0){
          curHr = 99;
        }
        encoderValue = 0;
        segmentBlink = 0;
      }
      blink_display_hour();
    }

    //-------------------------------POWER DOWN, SAVE IMMEDIATELY TO EEPROM
    if (!(PIND & (1 << POWER_OFF_DETECT))){ //active low
      if(curSec > 0 || curMin > 0 || curHr > 0){//only if we have value
        cli();//Disable Global Interrupts
        eeprom_write_byte((uint8_t*)EEPROM_TEMP_SECOND_SET_ADDRESS, curSec);//write eeprom
        eeprom_write_byte((uint8_t*)EEPROM_TEMP_MINUTE_SET_ADDRESS, curMin);//write eeprom
        eeprom_write_byte((uint8_t*)EEPROM_TEMP_HOUR_SET_ADDRESS, curHr);//write eeprom
        _delay_ms(500); //delay till the capacitor discharge
        sei();//Enable Global Interrupts
      }
    }
  }
}