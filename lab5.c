// lab5.c 
// Cruz M. Solano-Nieblas
// 11.28.21

//#define DEBUG
#define TEST

#define TRUE 1
#define FALSE 0
#define SEGNUMS 4
#define COLONPOS 2
#define BUTTONS 8
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] = {0xFF};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[15] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 
			   0x82, 0xF8, 0x80, 0x98, 0xFF, //0, 1, 2, 3, 4, 5, 6, 7, 8, 9, (blank)
			   0x07,0x04, 0x03 , 0x00}; //(colon blank), (colon on), (colon blank w/ alarm), (colon on w/ alarm)

enum encoder_state{IDLE, STATE01, DETENT, STATE10};  // four states for the encoder. STATE01 and STATE10 are in between IDLE and DETENT states

volatile uint8_t i; //general-purpose counter variable
volatile uint8_t mode; //user interface
uint8_t alarm_set = 0; //flag to indicate when the alarm is set
volatile uint16_t timer_count = 0, currentTime = 1200, testTime = 1200, setTime = 0; //current clock time

volatile int8_t setSeconds = 0, setMinutes = 0, setHours = 0;
volatile uint8_t seconds = 0, minutes = 0, hours = 12;

//encoder variables
uint8_t encoder_data = 0xFF; //data being read from the encoder pins

//encoder 1
volatile enum encoder_state encoder1 = IDLE; //init encoder1 state
volatile int8_t encoder1_count = 0; //counter to track the encoder1 state machine
volatile int8_t encoder1_direction = 0; //tracks whether encoder 1 rotated clockwise or counter-clockwise
volatile uint8_t pinA1 = 1, pinB1 = 1;
volatile uint8_t oldPinA1 = 1, oldPinB1 = 1; //hold pin values for encoder1

//encoder 2
volatile enum encoder_state encoder2 = IDLE; //init encoder2 state
volatile int8_t encoder2_count = 0; //counter to track the encoder2 state machine
volatile int8_t encoder2_direction = 0; //tracks whether encoder 2 rotated clockwise or counter-clockwise
volatile uint8_t pinA2 = 1, pinB2 = 1;
volatile uint8_t oldPinA2 = 1, oldPinB2 = 1; //hold pin values for encoder2

uint8_t  write_lcd = 0;
char     lcd_str[16];  //holds string to send to lcd  

uint8_t           ui;
volatile uint8_t  rcv_rdy;
char              rx_char; 
char              temperature[16];  //holds local temperature string 
char		  rx_temperature[16];  //holds remote temperature
uint8_t           send_seq=0;         //transmit sequence number
char              lcd_string[3];      //holds value of sequence number

extern uint8_t lm73_wr_buf[2] = {0};
extern uint8_t lm73_rd_buf[2] = {0};

//******************************************************************************
//				spi_init
//                     Initializes spi operation 				
//

void spi_init(void){
  /* Run this code before attempting to write to the LCD.*/
  DDRF  |= 0x08;  //port F bit 3 is enabling for LCD
  PORTF &= 0xF7;  //port F bit 3 is initially low

  DDRB |= (1<<PB0 | 1<<PB1 | 1<<PB2); //output mode for SS, MOSI, SCLK
  SPCR |= (1<<SPE | 1<<MSTR); //master mode, clk low on idle, leading edge sample
  SPSR |= 1<<SPI2X; //choose double speed operation

}//spi_init

//******************************************************************************
//				spi_read
//          Reads data from MISO pin connected to the encoders 				
//

uint8_t spi_read(void){

	PORTE &= ~(1<<PE6); // parallel load encoder pins
	_delay_us(100); //need a delay for buffer to change states and PORTA to read the buttons
	PORTE |= 1<<PE6; // disable parallel load to enable serial shifting
	_delay_us(100); //need a delay for buffer to change states and PORTA to read the buttons

	SPDR = 0x00; // dummy transmission to start receive
	while (bit_is_clear(SPSR, SPIF)){} // spin until transmission is complete

	return SPDR;


}//spi_read

//******************************************************************************
//				spi_write
//          Writes data to MOSI pin connected to the bar graph 				
//

void spi_write(uint8_t data){
	
	SPDR = data;
	while (bit_is_clear(SPSR, SPIF)){} // spin until transmission is complete
	PORTD |= 1<<PD2;
	PORTD &= ~(1<<PD2);
	
}//spi_write

//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//

uint8_t chk_buttons(uint8_t button) {
static uint16_t states[8] = {0}; // an array to store the states of all buttons on the button board
			// states[0] corresponds to S1 on the board and states[7] corresponds to S8
states[button] = (states[button]<<1 | (! bit_is_clear(PINA, button)) | 0xE000); //first extract the bit that corresponds to the button
									      //then shift the state back to the 1's place
if (states[button] == 0xF000) {return TRUE;}
return FALSE;

}//chk_buttons

// interrupt generated at 512Hz
// 1 sec = (32768) / (2^6 * 512)
ISR(TIMER0_COMP_vect){
	static uint8_t display_counter = 0;

	timer_count++;
	if ((timer_count % 256) == 0){
		segment_data[COLONPOS] = alarm_set ? dec_to_7seg[13] : dec_to_7seg[11];

	}
	if ((timer_count % 512) == 0){
		seconds++;
		segment_data[COLONPOS] = alarm_set ? dec_to_7seg[14] : dec_to_7seg[12];
		write_lcd = 1;

	}
	if (seconds > 59){
		minutes++;
		seconds = 0;
	
	}
	if (minutes > 59){
		hours++;
		minutes = 0;

	}	
	if (hours > 12){
		hours = 1;

	}
	currentTime = 100*hours + minutes;
	testTime = 100*minutes + seconds;

	//make PORTA an input port with pullups 
	DDRA = 0x00; //inputs
	PORTA = 0xFF; //pullups enabled

	//enable tristate buffer for pushbutton switches
	PORTB |= 1<<PB4 | 1<<PB5 | 1<<PB6; //decoder outputs logic low DEC7 to active low tri state buffer

	_delay_us(0.1); //need a delay for buffer to change states and PORTA to read the buttons
	//now check each button and increment the count as needed

	if (chk_buttons(0)){mode ^= 1;} //toggle the bit on the bar graph that corresponds to the button
	if (chk_buttons(1)){mode ^= 1<<1;} //toggle the bit on the bar graph that corresponds to the button
	if (chk_buttons(6)){mode ^= 1<<6;} //toggle the bit on the bar graph that corresponds to the button
	if (chk_buttons(7)){mode ^= 1<<7;} //toggle the bit on the bar graph that corresponds to the button
	
	encoder_data = spi_read(); //read encoder pins from spi

	pinA1 = ((encoder_data & 0x01) == 0) ? 0 : 1; //sample pinA from encoder 1
	pinB1 = ((encoder_data & 0x02) == 0) ? 0 : 1; //sample pinB from encoder 1
	//encoder1 state machine
	switch (encoder1){
		case IDLE:
		      //check if encoder1 has gone through all states of the state machine
		      if (encoder1_count == 3){
			      setHours += 1;
		      }
		      else if (encoder1_count == -3){
			      setHours -= 1; 
		      }
		      encoder1_count = 0;
		      if ((pinA1 != oldPinA1) || (pinB1 != oldPinB1)){ //if movement detected
			      if ((pinA1 == 0) && (pinB1 == 1)){ //CW movement
				      if (oldPinA1 == 1){
					      encoder1 = STATE01;
					      encoder1_count++;
				      }
			      }
			      else if ((pinA1 == 1) && (pinB1 == 0)){ //CCW movement
				      if (oldPinB1 == 1){
					      encoder1 = STATE10;
					      encoder1_count--;
				      }
			      }
		      }
		      break;

		case STATE01:
		      if ((pinA1 == 0) && (pinB1 == 0)){ //CW movement
			      if (oldPinB1 == 1){
				      encoder1 = DETENT;
				      encoder1_count++;
			      }
		      }
		      else if ((pinA1 == 1) && (pinB1 == 1)){ //CCW movement
			      if (oldPinA1 == 0){
				      encoder1 = IDLE;
			      }
		      }
		      break;

		case DETENT:
		      if ((pinA1 == 1) && (pinB1 == 0)){ //CW movement
			      if (oldPinA1 == 0){
				      encoder1 = STATE10;
				      encoder1_count++;
			      }
		      }
		      else if ((pinA1 == 0) && (pinB1 == 1)){ //CCW movement
			      if (oldPinB1 == 0){
				      encoder1 = STATE01;
				      encoder1_count--;
			      }
		      }
		      break;

		case STATE10:
		      if ((pinA1 == 1) && (pinB1 == 1)){ //CW movement
			      if (oldPinB1 == 0){
				      encoder1 = IDLE;
			      }
		      }
		      else if ((pinA1 == 0) && (pinB1 == 0)){ //CCW movement
			      if (oldPinA1 == 1){
				      encoder1 = DETENT;
				      encoder1_count--;
			      }
		      }
		      break;

	}//end switch
	oldPinA1 = pinA1;
	oldPinB1 = pinB1;
	
	pinA2 = ((encoder_data & 0x04) == 0) ? 0 : 1; //sample pinA from encoder 2
	pinB2 = ((encoder_data & 0x08) == 0) ? 0 : 1; //sample pinB from encoder 2
	//encoder 2 state machine
        switch (encoder2){
		case IDLE:
		      //check if encoder2 has gone through all states of the state machine
		      if (encoder2_count == 3){
			      setMinutes += 1;
		      }
		      else if (encoder2_count == -3){
			      setMinutes -= 1;
		      }
		      encoder2_count = 0;
		      if ((pinA2 != oldPinA2) || (pinB2 != oldPinB2)){ //if movement detected
			      if ((pinA2 == 0) && (pinB2 == 1)){ //CW movement
				      if (oldPinA2 == 1){
					      encoder2 = STATE01;
					      encoder2_count++;
				      }
			      }
			      else if ((pinA2 == 1) && (pinB2 == 0)){ //CCW movement
				      if (oldPinB2 == 1){
					      encoder2 = STATE10;
					      encoder2_count--;
				      }
			      }
		      }
		      break;

		case STATE01:
		      if ((pinA2 == 0) && (pinB2 == 0)){ //CW movement
			      if (oldPinB2 == 1){
				      encoder2 = DETENT;
				      encoder2_count++;
			      }
		      }
		      else if ((pinA2 == 1) && (pinB2 == 1)){ //CCW movement
			      if (oldPinA2 == 0){
				      encoder2 = IDLE;
			      }
		      }
		      break;

		case DETENT:
		      if ((pinA2 == 1) && (pinB2 == 0)){ //CW movement
			      if (oldPinA2 == 0){
				      encoder2 = STATE10;
				      encoder2_count++;
			      }
		      }
		      else if ((pinA2 == 0) && (pinB2 == 1)){ //CCW movement
			      if (oldPinB2 == 0){
				      encoder2 = STATE01;
				      encoder2_count--;
			      }
		      }
		      break;

		case STATE10:
		      if ((pinA2 == 1) && (pinB2 == 1)){ //CW movement
			      if (oldPinB2 == 0){
				      encoder2 = IDLE;
			      }
		      }
		      else if ((pinA2 == 0) && (pinB2 == 0)){ //CCW movement
			      if (oldPinA2 == 1){
				      encoder2 = DETENT;
				      encoder2_count--;
			      }
		      }
		      break;

	}//end switch
	oldPinA2 = pinA2;
	oldPinB2 = pinB2;

	if (setMinutes > 59){
		setHours++;
		setMinutes -= 60;

	}
	else if (setMinutes < 0){
		setHours--;
		setMinutes = 59;

	}
	if (setHours > 12){
		setHours = 1;

	}
	else if (setHours < 1){
		setHours = 12;	

	}
	setTime = 100*setHours + setMinutes;

	DDRA = 0xFF; //make PORTA an output port

	//bound a counter (0-4) to keep track of digit to display 
	if (display_counter == 5){display_counter = 0;} //first digit
	PORTA = segment_data[display_counter]; //send 7 segment code to LED segments
	PORTB = display_counter << 4;
	//_delay_ms(1);

	//send PORTB the next digit to display
	display_counter++;

}//end ISR

//alarm pin interrupt
ISR(TIMER1_COMPA_vect)
{
	PORTC ^= 1<<PC0;

}//end ISR

//ADC conversion complete interrupt
ISR(ADC_vect){
	//adc_result = ADC;
	OCR2 = ADCH;

}

//UART receive interrupt
ISR(USART0_RX_vect){
static  uint8_t  ui;
  rx_char = UDR0;              //get character
  rx_temperature[ui++]=rx_char;  //store in array 
 //if entire string has arrived, set flag, reset index
  if(rx_char == '\0'){
    rcv_rdy=1; 
    rx_temperature[--ui]  = (' ');     //clear the count field
    rx_temperature[ui+1]  = (' ');
    rx_temperature[ui+2]  = (' ');
    ui=0;  
  }
}//end ISR

//******************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  //int i; //for loop variable
  //determine how many digits there are 
  int digits = 0; //stores the number of digits in sum
  uint8_t digit = 0; //stores a digit in sum
  uint16_t number = sum;
  if (number == 0){digits = 1;}
  else{
  while (number != 0) //divide number out until you get zero
  {
	number /= 10;
	digits++; // increase digits count after every loop iteration

  } 

  }

  //break up decimal sum into 4 digit-segments
  for (i = 0; i < digits+1; i++)
  {
	if (i == COLONPOS){i++;}
	digit = sum % 10; //extract least significant digit from the sum
	segment_data[i] = dec_to_7seg[digit]; //convert digit to BCD code and store in segment_data array
	sum /= 10; //remove last digit;

  }	

  //blank out leading zero digits 
  if (digits < SEGNUMS) //if there are less digits than segment numbers
  {
	for (i = digits+1; i < SEGNUMS+1; i++)
	{
		if (i == COLONPOS){i++;}
		segment_data[i] = dec_to_7seg[10]; //blank them out	
	
	}

  }

}//segment_sum

//***********************************************************************************
//				init_timers
//initializes all the timers
void init_timers(){
	//timer counter 0 setup
	ASSR |= 1<<AS0; //select 32KHz clock
	TIMSK |= 1<<OCIE0; //generate interrupt on compare match
	OCR0 = 63; //compare value
	segment_data[COLONPOS] = dec_to_7seg[12];
	TCCR0 |= (1<<WGM01 | 1<<CS00);  //no prescaling, ctc mode

	//timer counter 1 setup
	DDRC |= 1; //PCO will be used as the alarm pin
	TCCR1A = 0x00; //normal mode
	TIMSK |= 1<<OCIE1A; //enable output compare match interrupt
	OCR1A = 18517; //compare value
	TCCR1B |= (1<<WGM12); //ctc mode, don't enable clock yet

	//timer counter 2 setup
	DDRB |= 1<<PB7; //PB7 will used to drive the segment display and bar graph
	OCR2 = 0xFE;
	TCCR2 |= (1<<WGM21 | 1<<WGM20 | 1<<COM21 | 1<<CS20); //fast pwm mode
						//clear on compare watch, no prescaler
	//comare value will be updated by the ADC conversion complete ISR

	//timer counter 3 setup
	DDRE |= 1<<PE3; //PE3 will be used as a pwm output pin
	TCCR3A |= (1<<COM3A1 | 1<<WGM31); //clear PE3 on compare match, Fast-PWM
	ICR3 = 0x000A; //top
	OCR3A = 0x0000; //0% duty cycle
	TCCR3B |= (1<<WGM33 | 1<<WGM32 | 1<<CS30); //using ICR3 to define top, no prescaling


}//init_timers

//************************************************************************************
//				alarm_on
//turns on the alarm
void alarm_on(){
	TCCR1B |= 1<<CS10;	
	OCR3A = 0x0005;

}//alarm_on

//************************************************************************************
//				alarm_off
//turns off the alarm
void alarm_off(){
	TCCR1B &= ~(1<<CS10);	
	OCR3A = 0x0000;

}//alarm_off

//**************************************************************************************'
//				init_adc
//Initialize ADC
void init_adc(){
	//Initalize ADC and its ports
	DDRF  &= ~(_BV(DDF7)); //make port F bit 7 the ADC input  
	PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off

	ADMUX = 0x07 | 1<<REFS0 | 1<<ADLAR;                 //single-ended input, PORTF bit 7, left adjusted, 10 bits
						 //reference is AVCC

	ADCSRA = 1<<ADEN | 1<<ADIE | 0x07;                 //ADC enabled, don't start yet, single shot mode 



}


//***********************************************************************************
uint8_t main()
{
uint16_t alarmTime = 0; //alarm time set
uint16_t lm73_temp;  //a place to assemble the temperature from the lm73

//set port A as outputs
DDRA = 0xFF; 

//set port B bits 4-6 as outputs
DDRB |= (1<<PB4 | 1<<PB5 | 1<<PB6);
PORTB &= ~(0x70); //init Port B, select first digit

//uart init
DDRF |= 0x08; //lcd strobe bit
uart_init();  

//bar graph and encoder init
DDRE |= 1<<PE6;
PORTE |= 1<<PE6;
spi_init();
DDRD |= 1<<PD2;

//init lcd
lcd_init();

//init twi
init_twi(); //initalize TWI (twi_master.h)  

//init timers
init_timers();

//init ADC
init_adc();

sei(); //enable global interrupt flag

//set LM73 mode for reading temperature by loading pointer register
lm73_wr_buf[0] = LM73_PTR_TEMP; //load lm73_wr_buf[0] with temperature pointer address
twi_start_wr(LM73_WRITE, lm73_wr_buf, 2); //start the TWI write process
_delay_ms(2);    //wait for the xfer to finish

clear_display(); //clean up the display
cursor_home();
strcpy(lcd_str, "Temp: L-   R-");
string2lcd(lcd_str);

while(1){

  twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
  _delay_ms(2);    //wait for it to finish
  lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
  lm73_temp = lm73_temp << 8; //shift it into upper byte 
  lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
  itoa(lm73_temp>>7, temperature, 10); //convert to string in array with itoa() from avr-libc 

  if (write_lcd){
	  //**************  start rcv portion ***************
	  if(rcv_rdy==1){
	    set_cursor(1, 13);
	    string2lcd(rx_temperature);  //write out string if its ready
	    rcv_rdy=0;
	  }//if 
	  //**************  end rcv portion ***************

	  set_cursor(1,8); 
	  string2lcd(temperature); //send the string to LCD (lcd_functions)
	  write_lcd = 0;
 
  }

  ADCSRA |= 1<<ADSC; //poke the ADSC bit and start conversion

  spi_write(mode); //show what mode the user is in
  switch (mode){
  	case 0x01: //set current real-time clock
		setHours = hours;
		setMinutes = minutes;
		setTime = 100*setHours + setMinutes;
		while (mode & 0x01){ //set time using the encoders
			segsum(setTime);
			mode &= ~(0xFE);
		}
		hours = setHours;
		minutes = setMinutes;
		currentTime = 100*hours + minutes;
		break;

	case 0x02: //set alarm clock
		setHours = hours;
		setMinutes = minutes;
		setTime = 100*setHours + setMinutes;
		while (mode & 0x02){ //set time using the encoders
			segsum(setTime);
			mode &= ~(0xFD);
		}
		alarmTime = setTime;
		alarm_set = 1;
		strcpy(lcd_str, "ALARM SET");
		set_cursor(2, 0);
		string2lcd(lcd_str);
		segment_data[COLONPOS] = dec_to_7seg[13];
		break;

	case (1<<6): //disarm alarm
		alarm_off();
		alarm_set = 0;
		set_cursor(2, 0);
		strcpy(lcd_str, "           ");
		string2lcd(lcd_str);
		mode &= ~(1<<6);
		break;

	case (1<<7): //snooze alarm
		if (alarm_set){
			#ifdef TEST
				setMinutes = minutes;
				setSeconds = seconds + 10;
				if (setSeconds > 59){
					setMinutes++;
					setSeconds -= 60;

				}
				if (setMinutes > 59){
					setMinutes = 0;

				}
				alarmTime = 100*setMinutes + setSeconds;
			#else
				setHours = hours;
				setMinutes = minutes + 10;
				if (setMinutes > 59){
					setHours++;
					setMinutes -= 60;

				}
				if (setHours > 12){
					setHours = 1;

				}
				alarmTime = 100*setHours + setMinutes;
			#endif

			alarm_off();
		}
		mode &= ~(1<<7);
		break;
	
	default: //display current time
		segsum(currentTime);
		#ifdef TEST
			if (alarm_set & (currentTime == alarmTime)){
				alarm_on();
			}
			if (alarm_set & (testTime == alarmTime)){
				alarm_on();
			}

		#else
			if (alarm_set & (currentTime == alarmTime)){
				alarm_on();
			}

		#endif


  }


  }//while
}//main
