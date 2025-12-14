/*
 *======================================================================================================================
 * WindSpeedAtTiny85 - 
 *   Board Type : Atmel Dip-8 ATTINY85-20PU https://www.amazon.com/dp/B06W9JBJJ6 
 *   Programmer : Tiny AVR Programmer for Arduino https://www.amazon.com/dp/B0BK9S6BXN
 *   Arduino Settings
 *      Programmer: USBtinyISP (ATTinyCore), SLOW, for new or 1Hz parts
 *      Board: AtTTinyCore -> ATtiny25/45/85 (No bootloader)
 *   Description: ATtiny85 Anemoneter Rotation Counter. 
 *                Maintains count of interrupts from SS451A
 *                When this counter's 2 bytes are read via I2C, the count is reset
 *   Author: Robert Bubon
 *   Date:   2025-10-18 RJB Initial
 *======================================================================================================================
 */


#include <TinyWireS.h>

// Code for the ATtiny85 Anemoneter Rotation Counter

// ATtiny Pinout
// Pin Desc    To
// 1   RESET   Not in use
// 2   [3] A3  Not in use
// 3   [4] A2  Not in use
// 4   GND     I2C GND & SS451A GND       Pin 2 (Center Pin)
// 5   [0] PWM I2C SDA
// 6   [1] PWM           SS451A Interrupt Pin 3 (Right)
// 7   [2] A1  I2C SCL
// 8   VCC     I2C VCC & SS451A VCC       Pin 1 (Left)


#define I2C_SLAVE_ADDRESS 0x8   // Address of the slave
#define INTERRUPT_PIN PB1       // Physical pin 6, Port B bit 1, Aka 1

volatile uint16_t pulseCount = 0;
volatile bool updated = false;

ISR(PCINT0_vect) {
  static uint32_t last_interrupt_time = 0;
  uint32_t current_time = micros();

  // The PINB register holds the current logical state (HIGH or LOW) of the physical
  // pins on Port B of the microcontroller at the moment you read it/
  //    Faster than "if (digitalRead(INTERRUPT_PIN) == LOW)"

  // Check if pin PB1 is LOW (falling edge)
  if ((PINB & (1 << INTERRUPT_PIN)) == 0) {  
    // Debounce: check if 2 ms has passed since last interrupt
    if (current_time - last_interrupt_time > 2000) {
      pulseCount++;
      updated = true;
      last_interrupt_time = current_time;
    }
  }
}

void requestEvent() {
  // Send 2 bytes (high byte first)
  TinyWireS.send((pulseCount >> 8) & 0xFF);
  TinyWireS.send(pulseCount & 0xFF);
  pulseCount = 0;  // reset count after reading
  updated = false;
}

void setup() {
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
 
  // Attach pin change interrupt for pulse counting
  GIMSK |= (1 << PCIE);      // Enable pin change interrupts globally
  PCMSK |= (1 << PCINT1);    // Enable PCINT1 interrupt for PB1 (pin 6)

  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onRequest(requestEvent);

  sei(); // enable global interrupts
}

void loop() {
  TinyWireS_stop_check();  // Required to handle I2C slave requests
}
