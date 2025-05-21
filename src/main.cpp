#include <avr/io.h>
#include <avr/interrupt.h>
#include "util/delay.h"
#include <stdlib.h>

#define bitSet(reg, n) (reg |= 1 << n)
#define bitRead(reg, n) (reg >> n & 1)
#define bitClear(reg, n) (reg &= ~(1 << n))
#define bitCheck(reg, n) (reg & 1 << n)
#define bitToggle(reg, n) (reg ^= 1 << n)

//Sound
#define max 255
#define velSound 343

//sonar
#define pinTrigger PD4

//Mode Abstraction
#define MODE_Day 0
#define MODE_Night 1
#define MODE_off 2

//USART Declerations
void usart_init(float baud);
void usart_send_byte(unsigned char data);
void usart_send_string(char *pstr);
void usart_send_num(float num, char num_int, char num_decimal);
void usart_init_v2(float baud);
void usart_flush(void);


//function declarations
void IC_init();
void AC_init();
void sonar();
void adc_init();

//global variables

//ADC
volatile uint16_t adc;
volatile float temp;

//IC
float TimeOverflow_1;
volatile float timePerClick_tc1;

//IC_Inturpt
volatile uint16_t numOV_1 = 0;
uint16_t icr1;
float tFall = 0;
float tRise = 0;
volatile float tHigh;
volatile float tLow;


// Sonar Variables
volatile uint8_t *ddr_sonar = &DDRD;
volatile uint8_t *port_sonar = &PORTD;
volatile uint8_t *pin_sonar = &PIND;
volatile float dmm = 0;

// USART READLINE VARIABLES
#define BUF_SIZE 50
char usart_buf[BUF_SIZE] = {0};
char *pbuf_usart = usart_buf;
bool flag_read_done = 0;

ISR(USART_RX_vect) {
  char tmp;
  char *ptr = usart_buf;
  while(1) {
    while(!bitCheck(UCSR0A, RXC0));

    tmp = UDR0;
    if(tmp == '\r' || tmp == '\n') {
      *ptr = '\0';
      flag_read_done = 1;
      usart_flush();
      return;
    }
    else {
      *ptr++ = tmp;
    }
  }
}

//INTURUPTS

ISR(TIMER1_CAPT_vect) {
  icr1 = ICR1;
  float timeStamp = numOV_1 * TimeOverflow_1 + icr1 * timePerClick_tc1;

  if(!bitCheck (TCCR1B, ICES1)) {
    tFall = timeStamp;
    tHigh = tFall - tRise; 
  }
  else {
    tRise = timeStamp;
    tLow = tRise - tFall;
    // usart_send_string("Fall");
    // usart_send_num(ICR1, 6, 6);
    dmm = tLow * 343 / 2. * 1000.;
  }
  bitToggle(TCCR1B, ICES1);
}

ISR(ADC_vect) {
  adc = ADC;
  float resistance = 10000.0 * adc / (1023.0 - adc); // R_t

  // Constants
  float B = 3950.0;
  float T0 = 298.15; // 25°C in Kelvin
  float R0 = 10000.0;

  // Temperature calculation
  float tempK = 1.0 / (log(resistance / R0) / B + (1.0 / T0));
  float tempC = tempK - 273.15;

  usart_send_num(tempC, 3, 2); // Send Celsius with 2 decimal places
  usart_send_string("\n");
  //ADCSRA |= (1 << ADSC);

  _delay_us(20000);
  
  
}






int main() {
  usart_init_v2(9600);
  sei();

}

//Functions

void sonar() {
  numOV_1 = 0;

  bitSet(DDRD, pinTrigger);
  bitClear(PORTD, pinTrigger);
  _delay_us(2);
  bitSet(PORTD, pinTrigger);
  _delay_us(12);
  bitClear(PORTD, pinTrigger);
}

//INITs
void IC_init() {
  TCCR1A = 0;
  TCCR1B = 0;

  bitClear(TCCR1B, ICES1); // rising
  bitSet(TCCR1B, ICNC1); // Noise Cancaleation
  bitSet(TIMSK1, TOIE1); // overflow interupt enable 
  bitSet(TIMSK1, ICIE1); // Input Capture interupt enable
  TCCR1B = 0b100; // Prescaler 1024

  float timer1_clock_freq = 16.0e6 / 256;
  TimeOverflow_1 = 65536 / timer1_clock_freq; // max value of 16 bit / the clock freqencey = the overflow timer
  timePerClick_tc1 = 1. / timer1_clock_freq; // time per click
}
void AC_init() {
  bitClear(DDRD, PD6); // Set PD6 as input
  bitClear(DDRD, PD7); // Set PD7 as input
  //bitSet(ACSR, ACBG); // Bandgap Refrence enabled
  bitSet(ACSR, ACIC); //Input Capture inturprs enabled
}

void adc_init() {
  ADMUX |= (1 << REFS0); //Seting the ADC multiplexier register to ref0 for 5vv

  // This sets ADC2 without affecting ref bits only touches the last four bit

  bitSet(ADMUX, MUX1);

  //ADIE is bit 3 and it enables inturept
  //ADEN is bit 7 and it enables ADC
  //ADPS2 - ADPS0 are prescaler bits

  ADCSRA |= (1 << ADIE) | (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0) | (1 << ADPS1); // ADCSRA is the control and status register
  
  //They divide the system clock from 16mghz, 
  // ADPS2	ADPS1	ADPS0	Division Factor
  // 1	      1	    1 	 128
  // 1	      1	    0	   64
  // 1	      0	    1	   32

  ADCSRA |= (1 << ADSC);
}



/*USART FUNCTIONS
*
*
*
*/

void usart_init(float baud) {

  float ubrr0 = 1.0e6 / baud;
  int ubrr0a = (int)ubrr0;

  if(ubrr0 - ubrr0a >= 0.5) {
    ubrr0a = ubrr0a + 1;
  }

  UBRR0 = ubrr0a;
  bitSet(UCSR0B, TXEN0);
  UCSR0C |= 3 << UCSZ00;
}

void usart_send_byte(unsigned char data) {
  while(!bitCheck(UCSR0A, UDRE0));
  UDR0 = data;
}

void usart_send_string(char *pstr) {
  while(*pstr != '\0') {
    usart_send_byte(*pstr);
    pstr++;
  }
}

void usart_send_num(float num, char num_int, char num_decimal) {
  char str[20];
  if(num_decimal == 0) {
    dtostrf(num, num_int, num_decimal, str);
  }
  else {
    dtostrf(num, num_int + num_decimal + 1, num_decimal, str);
  }
  str[num_int + num_decimal + 1] = '\0';
  usart_send_string(str);
}

//USART initialization with RX enabled
void usart_init_v2(float baud) {
  usart_init(baud);
  bitSet(UCSR0B, RXCIE0);
  bitSet(UCSR0B, RXEN0);
}

void usart_flush(void) {
  char dummy;
  while(bitCheck(UCSR0A, RXC0)) {
    dummy = UDR0;
  }
}