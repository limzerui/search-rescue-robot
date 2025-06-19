#include <AFMotor.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define S0_PORT PORTG
#define S0_DDR DDRG
#define S0_BIT 0

#define S1_PORT PORTL
#define S1_DDR DDRL
#define S1_BIT 6

#define S2_PORT PORTC
#define S2_DDR DDRC
#define S2_BIT 0

#define S3_PORT PORTG
#define S3_DDR DDRG
#define S3_BIT 2

#define SENSOR_PORT PORTD
#define SENSOR_DDR DDRD
#define SENSOR_PIN PIND
#define SENSOR_BIT 1

AF_DCMotor br(1), bl(2), fr(4), fl(3);
volatile unsigned int green, red, edges, lastEdge, ms, pulse;

ISR(INT1_vect) {
  unsigned int now = ms * 1000 + TCNT2 * 4;
  long delta = (long)now - (long)lastEdge;
  pulse = (delta < 0) ? delta + 0xFFFF : delta;
  lastEdge = now;
  edges++;
}

ISR(TIMER2_COMPA_vect) {
  ms++;
  // runs every 33ms
  if (!(ms % 33)) {
    // update reading, alternate colour filter
    if ((ms / 33) % 2) {
      green = (edges >= 3) ? pulse : 65535;
      setColourFilter(0, 0);
    } else {
      red = (edges >= 3) ? pulse : 65535;
      setColourFilter(1, 1);
    }
    edges = 0;
  }
}

void setupMotors() {
  DDRB |= _BV(PB5);            // D11
  DDRE |= _BV(PE5) | _BV(PE3); // D5, D3
  DDRH |= _BV(PH3);            // D6

  // Enable phase-correct PWM output for motor PWM pins (TOP = 0xFF)
  TCCR1A = (1 << COM1A1) | (1 << WGM10);
  TCCR1B = (1 << CS11) | (1 << CS10);
  TCCR3A = (1 << COM3A1) | (1 << COM3C1) | (1 << WGM30);
  TCCR3B = (1 << CS31) | (1 << CS30);
  TCCR4A = (1 << COM4A1) | (1 << WGM40);
  TCCR4B = (1 << CS41) | (1 << CS40);
}

void setupServos() {
  DDRL |= _BV(PL5) | _BV(PL4) | _BV(PL3); // D44, D45, D46

  // Phase-correct PWM at 50Hz (TOP = 20000)
  TCCR5A = (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1) | (1 << WGM11);
  TCCR5B = (1 << WGM13) | (1 << CS11);
  ICR5 = 20000;

  OCR5A = 1500;
  OCR5B = 1500;
  OCR5C = 1500;
}

void setupColour() {
  S0_DDR |= _BV(S0_BIT);
  S1_DDR |= _BV(S1_BIT);
  S2_DDR |= _BV(S2_BIT);
  S3_DDR |= _BV(S3_BIT);

  SENSOR_DDR &= ~_BV(SENSOR_BIT);

  S0_PORT |= _BV(S0_BIT);
  S1_PORT &= ~_BV(S1_BIT);

  EICRA |= _BV(ISC10);
  EIMSK |= _BV(INT1);

  TCCR2A = (1 << WGM21);
  TCCR2B |= (1 << CS22);
  TIMSK2 = (1 << OCIE2A);
  OCR2A = 249;
  TCNT2 = 0;

  setColourFilter(0, 0);
}

void setupUART2(uint32_t baud) {
  uint16_t ubrr = (F_CPU / 4 / baud - 1) / 2;
  UCSR2A = _BV(U2X2);
  UCSR2B = _BV(RXEN2) | _BV(TXEN2);
  UCSR2C = _BV(UCSZ21) | _BV(UCSZ20);
  UBRR2H = ubrr >> 8;
  UBRR2L = ubrr;
}

uint8_t UART2Available() { return (UCSR2A & _BV(RXC2)); }

uint8_t UART2BlockingRead() {
  while (!(UCSR2A & _BV(RXC2))) {
  }
  return UDR2;
}

void UART2BlockingWrite(uint8_t c) {
  while (!(UCSR2A & _BV(UDRE2))) {
  }
  UDR2 = c;
}

void delayMs(int delay_ms) {
  unsigned int last_time = ms;
  long delta = 0;
  while (delta < delay_ms) {
    delta = (long)ms - (long)last_time;
    delta = (delta < 0) ? (delta + 0xFFFF) : delta;
  }
}

void setColourFilter(bool s2, bool s3) {
  if (s2)
    S2_PORT |= (1 << S2_BIT);
  else
    S2_PORT &= ~(1 << S2_BIT);

  if (s3)
    S3_PORT |= (1 << S3_BIT);
  else
    S3_PORT &= ~(1 << S3_BIT);
}

uint8_t getColourStatus() {
  const unsigned int threshold = 6000;
  if ((green < threshold || red < threshold) && green < red)
    return 0b01; // green
  else if ((green < threshold || red < threshold) && red < green)
    return 0b00; // red
  else
    return 0b10; // black/unknown
}

void translate(bool dir, uint8_t speed) {
  br.run(dir ? FORWARD : BACKWARD);
  bl.run(dir ? BACKWARD : FORWARD);
  fr.run(dir ? FORWARD : BACKWARD);
  fl.run(dir ? BACKWARD : FORWARD);
  OCR1A = speed;
  OCR3C = speed;
  OCR3A = speed;
  OCR4A = speed;
}

void rotate(bool dir, uint8_t speed) {
  br.run(dir ? FORWARD : BACKWARD);
  bl.run(dir ? FORWARD : BACKWARD);
  fr.run(dir ? FORWARD : BACKWARD);
  fl.run(dir ? FORWARD : BACKWARD);
  OCR1A = speed;
  OCR3C = speed;
  OCR3A = speed;
  OCR4A = speed;
}

void stop() {
  OCR1A = 0;
  OCR3C = 0;
  OCR3A = 0;
  OCR4A = 0;
}

void openClaw() {
  OCR5B = 1800;
  OCR5C = 1300;
}

void closeClaw() {
  OCR5B = 1100;
  OCR5C = 2000;
}

void holdMedpack() { OCR5A = 1050; }

void depositMedpack() { OCR5A = 1650; }

void setup() {
  cli();
  setupMotors();
  setupServos();
  setupColour();
  setupUART2(115200);
  sei();
  openClaw();
  holdMedpack();
}

void loop() {
  int last_time = ms;

  while (!UART2Available()) {
    long delta = (long)ms - (long)last_time;
    delta = (delta < 0) ? (delta + 0xFFFF) : delta;
    if (delta > 500)
      stop();
  }

  uint8_t cmd = UART2BlockingRead();
  switch (cmd) {
  case 0:
    holdMedpack();
    break;
  case 1:
    translate(true, 255);
    break;
  case 2:
    stop();
    break;
  case 3:
    rotate(true, 255);
    break;
  case 4:
    rotate(false, 255);
    break;
  case 5:
    translate(false, 255);
    break;
  case 6:
    openClaw();
    break;
  case 7:
    closeClaw();
    break;
  case 8:
    depositMedpack();
    break;
  case 9:
    rotate(true, 255);
    delayMs(25);
    stop();
    break;
  case 10:
    rotate(false, 255);
    delayMs(25);
    stop();
    break;
  case 11:
    translate(true, 255);
    delayMs(25);
    stop();
    break;
  case 12:
    translate(false, 255);
    delayMs(25);
    stop();
    break;
  default:
    break;
  }

  uint8_t col = getColourStatus();
  UART2BlockingWrite(col);
}