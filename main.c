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
#include <util/delay.h>

#define F_CPU 1000000UL
#define EEPROM_STORAGE_ADDRESS 0x08
#define POWER_OFF_DETECT PD4
#define SEGMENT_DOT PB7
#define GEAR_SHIFT_UP PD0
#define GEAR_SHIFT_DOWN PD1
#define RESET_ALL PD2

volatile uint8_t secondFlag;
uint8_t upState = 0, dwnState = 0;//for simple debounce purpose
uint8_t gearCount = 0;
uint8_t valueRead = 0;
uint8_t buttonPressed = 0; 
uint8_t states_of_segment[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71}; //abcdefg

ISR(TIMER1_COMPA_vect)
{
  secondFlag = 1;
}

void update_display()
{
  //display the values
  PORTB &= 0b10000000;
  PORTB |= states_of_segment[gearCount];
}

int main (void)
{
  //initialize the I/O Ports
  DDRC |= (1 << PC5)|(1 << PC4)|(1 << PC3)|(1 << PC2); //led in pc5,4,3,2,pins
  DDRB |= (1 << PB0)|(1 << PB1)|(1 << PB2)|(1 << PB3)|(1 << PB4)|(1 << PB5)|(1 << PB6)|(1 << SEGMENT_DOT); //seven segment gfedcba.
  DDRD &= ~((1 << GEAR_SHIFT_UP)|(1 << GEAR_SHIFT_DOWN)|(1 << RESET_ALL)|(1 << POWER_OFF_DETECT));    // switch on pin (GEAR_SHIFT_UP)
  PORTD |= (1 << GEAR_SHIFT_UP)|(1 << GEAR_SHIFT_DOWN)|(1 << RESET_ALL)|(1 << POWER_OFF_DETECT);    // enable pull-up resistor

  TCCR1B |= (1 << WGM12);           // Configure timer 1 for CTC mode
  TIMSK |= (1 << OCIE1A);           // Enable CTC interrupt
  OCR1A = 15624;                // Set CTC compare value to 1Hz at 1MHz AVR clock, with a prescaler of 64
  TCCR1B |= ((1 << CS10) | (1 << CS11));  // Start timer at Fcpu/64

  //READ FROM EEPROM
  valueRead = eeprom_read_byte((uint8_t*)EEPROM_STORAGE_ADDRESS);//read eeprom
  if (valueRead == 0xFFFF)//initial value.. omit it 
  {
    gearCount = 0;
  }else{//has some value and its larger 
    gearCount = valueRead;
  }

  sei();//Enable Global Interrupts

  while (1)//main loop 
  {
    if (secondFlag){
      secondFlag = 0;
      update_display();
      PORTB ^= (1 << SEGMENT_DOT);    // toggles the led
    }

    //hall effect sensor functionality
    if ((PIND & (1 << GEAR_SHIFT_UP)) && upState == 1){//reset up debounce
      upState = 0;
    }else if ((PIND & (1 << GEAR_SHIFT_DOWN)) && dwnState == 1){ //reset down debounce
      dwnState = 0;
    } else if ((!(PIND & (1 << GEAR_SHIFT_UP))) && upState == 0){  //if shift up detected
      gearCount ++;
      if(gearCount > 15){
        gearCount = 0;
      }
      PORTB |= (1 << SEGMENT_DOT);    // light up the dot marker
      upState = 1;
      update_display();
    }else if ((!(PIND & (1 << GEAR_SHIFT_DOWN))) && dwnState == 0){  //if shift down detected
      gearCount --;
      if(gearCount < 0){
        gearCount = 15;
      }
      PORTB |= (1 << SEGMENT_DOT);    // light up the dot marker
      dwnState = 1;
      update_display();
    }else if (!(PIND & (1 << RESET_ALL))){  //if reset button pressed
      //reset the gear state stored in eeprom if it has any previous value
      cli();//Disable Global Interrupts
      valueRead = eeprom_read_byte((uint8_t*)EEPROM_STORAGE_ADDRESS);//read eeprom
      eeprom_write_byte((uint8_t*)EEPROM_STORAGE_ADDRESS,0);//write eeprom
      gearCount = 0;
      PORTB |= (1 << SEGMENT_DOT);    // light up the dot marker
      _delay_ms(1000);
      sei();//Enable Global Interrupts
    }

    //-------------------------------POWER DOWN, SAVE IMMEDIATELY TO EEPROM
    if (!(PIND & (1 << POWER_OFF_DETECT))){ //active low
      cli();//Disable Global Interrupts
      eeprom_write_byte((uint8_t*)EEPROM_STORAGE_ADDRESS, gearCount);//write eeprom
      PORTB |= (1 << SEGMENT_DOT);    // light up the dot marker
      _delay_ms(500); //delay till the capacitor discharge
      sei();//Enable Global Interrupts
    }
  }
}