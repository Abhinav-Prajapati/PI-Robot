#ifndef HEADERS_H
#define HEADERS_H

// Imports
#include <FastLED.h>
#include <Wire.h>

#include <BTS_7980.h>
#include <Kinematics.h>

// Slave PICO Address
#define SLAVE_ADDRESS 0x09

// Status Led init
#define LED_PIN 17
#define NUM_LEDS 2
#define BRIGHTNESS 200
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
int warning_led_blink_rate = 200; // In Millisecond

bool CONNECTED = false; // Stores the Drive Hub connection status
bool FIRST_BOOT = true; // Stores the Drive Hub first connection status
const int time_out = 100;
// Set everything to 0 if no new data is received within the next 100 ms of receiving data

unsigned long previousMillis = 0;            // Stores the last time the LED was updated
unsigned long previousMillis_discon_led = 0; // Stores the last time the LED was updated
bool connection_led_state = false;

/*
 6 byte for 3 axes (2 for each axes after byte splitting) i for 8 buttons 1 for lead status
 Number of bytes to read from i2c bus (plus 1 cause 0 is recived as first byte)
*/

int bytes_to_read = 8 + 1;

struct I2cData // current size 8
{
    int x_linear = 0;
    int y_linear = 0;
    int z_angular = 0;
    byte button_1 = 0;
    byte button_2 = 0;
    byte button_3 = 0;
    byte button_4 = 0;
    byte led_mode = 1;
} BusData;

// MotorDrive init
Motor motor_1(2, 3);
Motor motor_2(4, 5);
Motor motor_3(6, 7);
Motor motor_4(8, 9);

#endif