#include <Headers.h>

#ifndef utils
#define utils

void I2C_RxHandler(int byteCount)
{
    if (byteCount >= bytes_to_read)
    {
        CONNECTED = true;
        previousMillis = millis(); // Update the previous time (for fail-safe)

        int temp = Wire.read(); // First recived byte is 0 (dont know the reason may be encoding..issue)

        BusData.x_linear = ((Wire.read() << 8) | Wire.read()) - 1023;
        BusData.y_linear = ((Wire.read() << 8) | Wire.read()) - 1023;
        BusData.z_angular = ((Wire.read() << 8) | Wire.read()) - 1023;

        int buttons = Wire.read();

        BusData.button_1 = buttons >> 0 & 1;
        BusData.button_2 = buttons >> 1 & 1;
        BusData.button_3 = buttons >> 2 & 1;
        BusData.button_4 = buttons >> 3 & 1;

        BusData.led_mode = Wire.read();
    }
}

void updateLed(byte mode)
{
    switch (mode)
    {
    case 0: // Black
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        break;
    case 1:                          // Violet
        leds[0] = CRGB(148, 0, 211); // Violet color in CRGB format.
        leds[1] = CRGB(148, 0, 211);
        break;
    case 2:                         // Indigo
        leds[0] = CRGB(75, 0, 130); // Indigo color in CRGB format.
        leds[1] = CRGB(75, 0, 130);
        break;
    case 3: // Blue
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Blue;
        break;
    case 4: // Green
        leds[0] = CRGB::Green;
        leds[1] = CRGB::Green;
        break;
    case 5: // Yellow
        leds[0] = CRGB::Yellow;
        leds[1] = CRGB::Yellow;
        break;
    case 6: // Orange
        leds[0] = CRGB::Orange;
        leds[1] = CRGB::Orange;
        break;
    case 7: // Red
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
        break;
    default:
        // Code for handling other cases
        break;
    }
    FastLED.show();
}

void printI2cData(const I2cData &data)
{
    Serial.println("I2C Data:");
    Serial.print("X Linear: ");
    Serial.println(data.x_linear);
    Serial.print("Y Linear: ");
    Serial.println(data.y_linear);
    Serial.print("Z Angular: ");
    Serial.println(data.z_angular);
    Serial.print("Button 1: ");
    Serial.println(data.button_1);
    Serial.print("Button 2: ");
    Serial.println(data.button_2);
    Serial.print("Button 3: ");
    Serial.println(data.button_3);
    Serial.print("Button 4: ");
    Serial.println(data.button_4);
    Serial.print("LED Mode: ");
    Serial.println(data.led_mode);
    Serial.println("-----------------------------");
}

#endif