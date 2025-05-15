//-------------------------------------------------------
// Applicatie : Demo nRF24 control led, monitor expired time
// Description: This application is a demo example for the Aaadlander module controllers
//              The Aaadlander module communicates with the Aaad sateliet by nRF24 wireless communication.
//              The AAAD_ARO and AAAD_MODULE number define the sateliet and the module
//              This demo transmits the expired time from the Arduino processor and is able to switch
//              a light on port LED_PIN by receiving a specific payload via nRF24 communication 
//              
//              When a payload can not be delivered to the receiver the NO_ACK led will lit to indicate
//              bad transmission quality and lost of payload packages.
// Version    : 1.0.0
// Author     : Bart Linsen
// Date       : 20250402
//
// Revisions  : 1.0.0 - 20250403 First application.
//              
//-------------------------------------------------------
#include "lpp.h"
#include "Led.h"
#include <Arduino.h>
#include <printf.h>
#include <SPI.h>
#include <nRF24L01.h>   // to handle this particular modem driver
#include "RF24.h"       // the library which helps us to control the radio modem
#include <Wire.h>
#include <VL53L0X.h>

#define LEDPIN 3        // Ditital pin connected to the LED.
#define NOACK_PIN 4     // Digital pin to flag NOACK received by nRF24
#define CE_PIN 7        // RF-NANO usb-c, Arduino-uno -> 7, RF-NANO micro-usb -> 9
#define CSN_PIN 8       // RF-NANO usb-c, Arduino-uno -> 8, RF-NANO micro-usb -> 10


// Initialise Sensors
VL53L0X vl53l0x_Sensor;

// Initialise Actuators
Led led;
int ledState = LOW;			              // ledState used to set the LED

Led nRF24Led;

#define RF24_PAYLOAD_SIZE 32
#define AAAD_ARO 2
#define AAAD_MODULE 5

// Create an RF24 object
RF24 radio(CE_PIN,CSN_PIN);

const uint8_t rf24_channel[] = {100,105,110,115,120}; // Radio channels set depending on satellite number
const uint64_t addresses[] = { 0x4141414430LL, 0x4141414431LL, 0x4141414432LL, 0x4141414433LL, 0x4141414434LL, 0x4141414435LL };  //with radioNumber set to zero, the tx pipe will be 'AAAD0', which is basically HEX'4141414430', which is remote DESTINATION address for our transmitted data. The rx pipe code is the local receive address, which is what the remote device needs to set for the remote devices 'tx' pipe code.
uint8_t txData[RF24_PAYLOAD_SIZE];
uint8_t rxData[RF24_PAYLOAD_SIZE];
uint8_t bytes;

// HC-SR04 Inputs
const int trigPin = 9;
const int echoPin = 6;

long measurementDuration;
float measurementDistance;

// Timing configuration
unsigned long previousMillis = 0;     // will store last time LED was updated
unsigned long currentMillis;
unsigned long sampleTime = 5000;   // milliseconds of on-time

// int to hex converter
void printHex2(unsigned v) {
    Serial.print("0123456789ABCDEF"[v>>4]);
    Serial.print("0123456789ABCDEF"[v&0xF]);
}

void setup() {
  Serial.begin(9600);

  Serial.println("\n\nnRF24 Application ARO" + String(AAAD_ARO) + ", Module" + String(AAAD_MODULE) + " Started!\n");

// Activate actuators
  led.begin(LEDPIN);
  led.setState(ledState);

  nRF24Led.begin(NOACK_PIN);
  nRF24Led.setState(LOW);

  Wire.begin();

  vl53l0x_Sensor.init();
  vl53l0x_Sensor.setTimeout(500);

  // Activate Radio
  printf_begin();
  SPI.begin();
  radio.begin();                     // Ativate the modem
  radio.setAddressWidth(5);          // Set Address width
  radio.setRetries(15,15);           // delaytime 4000uS and retry count is 15
  radio.setPayloadSize(RF24_PAYLOAD_SIZE);
  radio.setPALevel(RF24_PA_HIGH,1);  // Set the PA Level low to prevent power supply related issues
                                     // since this is a getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setDataRate(RF24_250KBPS);   // choosing 1 Mega bit per second radio frequency data rate
                                     // radio frequency data rate choices are:  //RF24_250KBPS    //RF24_2MBPS  //RF24_1MBPS
  radio.setChannel(rf24_channel[AAAD_ARO]);
  radio.openWritingPipe(addresses[AAAD_MODULE]);
  radio.openReadingPipe(1, addresses[AAAD_MODULE]);
  
  Serial.println("\nnRF24 Setup Initialized");

  radio.printDetails();
  delay(500);

  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  // check to see if it's time to change the state of the LED
  currentMillis = millis();

  if(currentMillis - previousMillis >= sampleTime) {
    // VL53L0X meting defineren
    int vl53l0x_distance = vl53l0x_Sensor.readRangeSingleMillimeters();

    Serial.print("Distance: ");
    Serial.print(vl53l0x_distance);
    Serial.println(" mm");

    delay(500); // Wait half a second before next reading
    
    /////////////////////////////////////////////////////////

    unsigned long timeStamp = millis()/1000;
    uint8_t cursor = 0;

    // 1. Add timestamp
    txData[cursor++] = 1; // channel number for timestamp
    txData[cursor++] = LPP_UNIXTIME;
    txData[cursor++] = timeStamp >> 24;
    txData[cursor++] = timeStamp >> 16;
    txData[cursor++] = timeStamp >> 8;
    txData[cursor++] = timeStamp;

    int16_t distance_cm = vl53l0x_distance / 10; // mm to cm

    txData[cursor++] = 2; // channel number for distance
    txData[cursor++] = LPP_DISTANCE;
    txData[cursor++] = vl53l0x_distance >> 24;
    txData[cursor++] = vl53l0x_distance >> 16;
    txData[cursor++] = vl53l0x_distance >> 8;
    txData[cursor++] = vl53l0x_distance;

    // Fill rest of buffer with zeros
    while (cursor < RF24_PAYLOAD_SIZE) {
      txData[cursor++] = 0;
    }

    /****************** Transmit Mode ***************************/
    Serial.print("txData: ");
    for (size_t i=0; i<cursor; ++i) {
      if (i != 0) Serial.print(" ");
      printHex2(txData[i]);
    }
    Serial.println();

    radio.stopListening();
    if (radio.write(&txData, sizeof(txData))) {
      Serial.println("ACK received!");
      nRF24Led.setState(LOW);
    } else {
      Serial.println("No ACK received!");
      nRF24Led.setState(HIGH);
    }
    radio.startListening();
    previousMillis = currentMillis;
  }


  /****************** Receive Mode ***************************/

  if (radio.available()) {      //'available' means whether valid bytes have been received and are waiting to be read from the receive buffer
    // Receive data from radio
    while (radio.available()) { // While there is data ready
        bytes = radio.getPayloadSize();
        radio.read(rxData, bytes); // read value from the configured pipe radio.read(&rxData, sizeof(rxData));  // Get the payload
    }
    // if (rxData[0]!=0) {  // Filter no usefull data
      // Print received data in Hex format
      Serial.print("rxData: ");
      for (size_t i=0; i<RF24_PAYLOAD_SIZE; ++i) {
        if (i != 0) Serial.print(" ");
        printHex2(rxData[i]);
      }
    Serial.println();
    // }
    // Switch led on Received command
    if(rxData[0]==1 && rxData[1]==LPP_DIGITAL_OUTPUT) { // channelnumber == 0 and Datatype is LPP_DIGITAL_OUTPUT
        if (rxData[2]==0xFF) {
        Serial.println("Led=ON");
        led.setState(HIGH);
      }
      if (rxData[2]==0x7F) {
        Serial.println("Led=OFF");
        led.setState(LOW);
      }
    }
  }

}  // Loop