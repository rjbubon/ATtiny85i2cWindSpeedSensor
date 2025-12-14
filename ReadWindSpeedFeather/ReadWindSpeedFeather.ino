/*
 *======================================================================================================================
 * ReadWindSpeedFeather - 
 *   Board Type : Adafruit Feather M0
 *   Description: Read ATtiny85 Anemometer Rotation Count 
 *   Author: Robert Bubon
 *   Date:   2025-10-18 RJB Initial
 *======================================================================================================================
 */

#include <Wire.h>

#define ANEMO_ADDR 0x08     // Same address as defined in TinyWireS.begin()

/* 
 *=======================================================================================================================
 * I2C_Device_Exist - does i2c device exist at address
 * 
 *  The i2c_scanner uses the return value of the Write.endTransmission to see 
 *  if a device did acknowledge to the address.
 *=======================================================================================================================
 */
bool I2C_Device_Exist(byte address) {
  byte error;

  Wire.begin();                     // Connect to I2C as Master (no addess is passed to signal being a slave)

  Wire.beginTransmission(address);  // Begin a transmission to the I2C slave device with the given address. 
                                    // Subsequently, queue bytes for transmission with the write() function 
                                    // and transmit them by calling endTransmission(). 

  error = Wire.endTransmission();   // Ends a transmission to a slave device that was begun by beginTransmission() 
                                    // and transmits the bytes that were queued by write()
                                    // By default, endTransmission() sends a stop message after transmission, 
                                    // releasing the I2C bus.

  // endTransmission() returns a byte, which indicates the status of the transmission
  //  0:success
  //  1:data too long to fit in transmit buffer
  //  2:received NACK on transmit of address
  //  3:received NACK on transmit of data
  //  4:other error 

  // Partice Library Return values
  // SEE https://docs.particle.io/cards/firmware/wire-i2c/endtransmission/
  // 0: success
  // 1: busy timeout upon entering endTransmission()
  // 2: START bit generation timeout
  // 3: end of address transmission timeout
  // 4: data byte transfer timeout
  // 5: data byte transfer succeeded, busy timeout immediately after
  // 6: timeout waiting for peripheral to clear stop bit

  if (error == 0) {
    return (true);
  }
  else {
    // sprintf (msgbuf, "I2CERR: %d", error);
    // Output (msgbuf);
    return (false);
  }
}


void setup() {

  int countdown=30; // Wait N seconds for serial connection, then move on.
  Serial.begin(9600);
  while (!Serial && countdown) {
      countdown--;
      delay (1000);
  }
  Serial.println("ATtiny85 Anemometer Reader Start");    
  Wire.begin();             // Join I2C bus as master

  if (I2C_Device_Exist(ANEMO_ADDR)) {
    Serial.println("ATtiny85 Anemometer Reader Initialized");
  }
  
}

void loop() {
  // Request 2 bytes from the slave device
  Wire.requestFrom(ANEMO_ADDR, (uint8_t)2);

  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read();   // First byte = high byte
    uint8_t lowByte  = Wire.read();   // Second byte = low byte

    // Combine into 16-bit integer
    uint16_t pulseCount = ((uint16_t)highByte << 8) | lowByte;

    // Print value
    Serial.print("Pulse count: ");
    Serial.println(pulseCount);
  } else {
    Serial.println("No data received from ATtiny85!");
  }

  delay(1000);  // 1 second between readings
}
