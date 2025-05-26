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

//buzzer
#define pinBuzzer PD0

//sonar
#define pinTrigger PD4

//buttons
#define calThreshBut PB4 //calibrate threshold button
#define flipLogBut PD7 //flip logic button

//Mode Abstraction
#define MODE_Day 0
#define MODE_Night 1
#define MODE_off 2

//thresholds
#define LIGHT_THRESHOLD 700
#define DISTANCE_THRESHOLD 200

typedef enum{MODE_DAY, MODE_NIGHT, MODE_OFF} MODE;
MODE currentState = MODE_OFF;


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
void button_init();
void timer2_init();
void buzzer_on();
void buzzer_off();

//global variables
bool debug_mode = 0;
volatile uint8_t alarm_mode = 0;
volatile uint8_t alarm_active = 0;

//timer 2
volatile uint8_t matchCount = 0;

//ADC
volatile uint16_t adc;
volatile float temp;
volatile float lightMeasure;
volatile uint8_t adcChannel = 0;

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

      int mode_input = atoi(usart_buf); //reads input
      if(mode_input == 0 || mode_input == 1) { //checks if input is valid
        debug_mode = mode_input; //if valid, set debug mode
      }

      return;
    }
    else {
      *ptr++ = tmp;
    }
  }
}

//INTURUPTS

ISR(INT0_vect) {
  alarm_mode ^= 1; // Toggle mode

}
ISR(INT1_vect) {
  currentState = (MODE)((currentState + 1) % 3);
  if (debug_mode) {
    usart_send_string(">INT1: Mode toggled to ");
    usart_send_num(currentState, 1, 0);
    usart_send_string("\n");
  }
}

ISR(TIMER2_COMPA_vect) {
  matchCount++;
  if(matchCount >= 100) {
    matchCount = 0;
    if (!bitCheck(ADCSRA, ADSC)) {
      ADCSRA |= (1 << ADSC);
    }
  }

}

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
    
    dmm = tLow * 343 / 2. * 1000.;

    if(debug_mode) { //if debug mode
      usart_send_string(">Dmm:");
      usart_send_num(dmm, 3, 3);
      usart_send_string("\n");
    }
  }
  bitToggle(TCCR1B, ICES1);
}

ISR(ADC_vect) {
  
  if (adcChannel == 0) {
    
    lightMeasure = adc;

    ADMUX = (ADMUX & 0xF0) | 0b0101;
    adcChannel = 1;

    if(debug_mode) { //if in debug mode
    usart_send_string(">light:");
    usart_send_num(lightMeasure, 3, 2); // Send Celsius with 2 decimal places
    usart_send_string("\n");
  }

  }

  else {
    
    adc = ADC;
    //float resistance = 10000.0 * adc / (1023.0 - adc); // R_t
    float resistance = log(10000.0 * ((1023.0 / adc - 1)));

    // Constants
    float B = 3950.0;
    float T0 = 298.15; // 25°C in Kelvin
    float R0 = 10000.0;

    // Temperature calculation
    //float tempK = 1.0 / (log(resistance / R0) / B + (1.0 / T0));
    float tempK = 1.0 / (0.001129148 + (0.000234125 + (0.0000000876741 * resistance * resistance)) * resistance);
    float tempC = tempK - 273.15;
    
    if(debug_mode) { //if in debug mode
      usart_send_string(">tempC:");
      usart_send_num(tempC, 3, 2); // Send Celsius with 2 decimal places
      usart_send_string("\n");
    }

    ADMUX = (ADMUX & 0xF0) | 0b0100;
    adcChannel = 0;

  }

  _delay_us(20000);
}

/*ISR(PCINT0_vect) {
  _delay_ms(10);
  if(!bitRead(PINB, calThreshBut)) {
    //do threshold stuff here
    //likely something like:
    //threshold = adc
  }
}



/*ISR(PCINT2_vect) {
  _delay_ms(10);
  if(!bitRead(PIND, flipLogBut)) {
    currentState = (MODE)((currentState + 1) % 3);
  }
} */



int main() {
  usart_init_v2(9600);
  button_init();
  sei();
  IC_init();
  //AC_init();
  buzzer_init();
  adc_init();
  button_init();
  timer2_init();
  ext_interrupt_init();
  timer0_fast_pwm_buzzer();
  bitClear(TCCR0A, COM0A1);
  bitClear(TCCR0A, COM0B1);

  while (1) {
  uint8_t lightTrigger = 0;

  if (currentState == MODE_DAY && lightMeasure > LIGHT_THRESHOLD) {
    lightTrigger = 1;
  } else if (currentState == MODE_NIGHT && lightMeasure < LIGHT_THRESHOLD) {
    lightTrigger = 1;
  }

  if (lightTrigger && currentState != MODE_OFF) {
    sonar(); // update distance

    if (dmm >= DISTANCE_THRESHOLD && !alarm_active) {
      trigger_alarm();
    } else if (dmm < DISTANCE_THRESHOLD && alarm_active) {
      stop_alarm();
    }
  } else {
    if (alarm_active) {
      stop_alarm();
    }
  }

  _delay_ms(100);
}

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

void trigger_alarm() {
  alarm_active = 1;
  if (alarm_mode == 0) {
    timer0_fast_pwm_buzzer();
  } else {
    timer0_phase_correct_pwm_led();
  }
  if (debug_mode) usart_send_string(">Alarm ON\n");
}

void stop_alarm() {
  alarm_active = 0;
  TCCR0A &= ~((1 << COM0A1) | (1 << COM0B1));
  if (debug_mode) usart_send_string(">Alarm OFF\n");
}

//INITs

void timer0_phase_correct_pwm_led() {
  bitSet(DDRD, PD5); // OC0B = output
  TCCR0A = (1 << WGM00);                     // Phase-Correct PWM
  TCCR0A |= (1 << COM0B1);                   // Non-inverting OC0B
  TCCR0B = (1 << CS01) | (1 << CS00);        // Prescaler 64
  OCR0B = 200; // Visible LED flash
}


void timer0_fast_pwm_buzzer() {
  bitSet(DDRD, PD6); // OC0A = output
  TCCR0A = (1 << WGM00) | (1 << WGM01);           // Fast PWM
  TCCR0A |= (1 << COM0A1);                        // Non-inverting OC0A
  TCCR0B = (1 << CS01) | (1 << CS00);             // Prescaler 64
  OCR0A = 128; // 50% duty
}

void ext_interrupt_init() {
  
  bitClear(DDRD, PD2);         // INT0 input
  bitSet(EIMSK, INT0);         // Enable INT0
  EICRA |= (1 << ISC01);       // Falling edge
  EICRA &= ~(1 << ISC00);

  bitClear(DDRD, PD3);
  bitSet(PORTD, PD3); // pull-up
  bitSet(EIMSK, INT1);
  EICRA |= (1 << ISC11);  // falling edge
  EICRA &= ~(1 << ISC10);
}


void buzzer_init(){
  bitSet(DDRD, pinBuzzer);
}

void timer2_init() {

  bitSet(TCCR2A, WGM21);

  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);

  OCR2B = 156;

  bitSet(TIMSK2, OCIE2A);


}
void IC_init() {
  TCCR1A = 0;
  TCCR1B = 0;

  bitClear(TCCR1B, ICES1); // rising
  bitSet(TCCR1B, ICNC1); // Noise Cancaleation
  bitSet(TIMSK1, TOIE1); // overflow interupt enable 
  bitSet(TIMSK1, ICIE1); // Input Capture interupt enable
  TCCR1B |= (1 << CS12); // Prescaler 256

  float timer1_clock_freq = 16.0e6 / 256;
  TimeOverflow_1 = 65536 / timer1_clock_freq; // max value of 16 bit / the clock freqencey = the overflow timer
  timePerClick_tc1 = 1. / timer1_clock_freq; // time per click
}
void AC_init() {
  bitClear(DDRD, PD6); // Set PD6 as input
  bitClear(DDRD, PD7); // Set PD7 as input
  //bitSet(ACSR, ACBG); // Bandgap Refrence enabled
  //bitSet(ACSR, ACIC); //Input Capture inturprs enabled
}

void adc_init() {
  ADMUX |= (1 << REFS0); //Seting the ADC multiplexier register to ref0 for 5vv

  // This sets ADC2 without affecting ref bits only touches the last four bit

  bitSet(ADMUX, MUX1);
  bitSet(ADMUX, MUX2);

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

void button_init() {
  /*bitClear(DDRB, calThreshBut);
  bitClear(DDRD, flipLogBut);

  bitSet(PORTB, calThreshBut);
  bitSet(PORTD, flipLogBut);

  PCICR |= 1 << PCIE0;
  PCMSK0 |= 1 << PCINT4;

  PCICR |= 1 << PCIE2;
  PCMSK2 |= 1 << PCINT23; */
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