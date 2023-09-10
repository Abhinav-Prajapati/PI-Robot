/*
TODO

- [x] Differnt led color on first boot (solid voilate on start)
- [x] Different led color on drive move (orange) when stop (green)
- [x]
- [x]
- [x]
- [x]
- [x]
- [x]
*/
#include <Arduino.h>
#include <Headers.h>
#include <Kinematics.h>
#include <utils.h>

void updateRoutine()
{
  updateLed(BusData.led_mode);
  if (true)
    printI2cData(BusData);
}

void setup()
{
  Serial.begin(9600);

  // i2c init
  Wire.setSDA(0); // These function is pico specific
  Wire.setSCL(1);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(I2C_RxHandler);

  // Status led init
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  BusData.led_mode = 1; // turn off led on startup
}

void loop()
{
  unsigned long currentMillis = millis();
  // Power up the motor if DrveHub is connected
  CONNECTED = (currentMillis - previousMillis <= time_out) ? true : false;

  if (CONNECTED)
  {
    updateRoutine();

    double radius = 0.20;     // Replace with your desired wheel radius
    double separation = 0.50; // Replace with your desired wheel separation

    OmniWheelDrive omniDrive(radius, separation);

    // Now you can use the omniDrive object to calculate wheel speeds
    double xVel = BusData.x_linear;      // Replace with your desired x velocity
    double yVel = BusData.y_linear;      // Replace with your desired y velocity
    double zAngular = BusData.z_angular; // Replace with your desired angular velocity

    int Speed[3];

    omniDrive.calculateWheelSpeeds(xVel, yVel, zAngular, Speed);

    Serial.println("Wheel Speeds:");

    Serial.print("Wheel 1: ");
    Serial.println(Speed[0]);

    Serial.print("Wheel 2: ");
    Serial.println(Speed[1]);

    Serial.print("Wheel 3:");
    Serial.println(Speed[2]);

    Serial.println("-----------------------------");

    motor_1.setPWM(0);
    motor_2.setPWM(0);
    motor_3.setPWM(0);
  }

  // Handle Disconnection
  if (!CONNECTED)
  {
    motor_1.setPWM(0);
    motor_2.setPWM(0);
    motor_3.setPWM(0);
    // motor_1.setPWM(0);

    // Disconnection Led Warning
    if (currentMillis - previousMillis_discon_led >= warning_led_blink_rate)
    {
      connection_led_state = !connection_led_state;

      if (connection_led_state == true)
      {
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
      }
      else
      {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
      }
      FastLED.show();
      previousMillis_discon_led = currentMillis;
    }
    // Stop all motor
    // stopMotor();
    // Empty the DATA_BUS array
  }
  delay(10);
}
// motor_1.setSpeed(BusData.motor_1_power);
// motor_2.setSpeed(BusData.motor_2_power);
// motor_3.setSpeed(BusData.motor_3_power);
// motor_4.setSpeed(BusData.motor_4_power);

// CytronMD motor_1(PWM_PWM, 2, 3);  // PWM 1A = Pin 3, PWM 1B = Pin 9.
// CytronMD motor_2(PWM_PWM, 4, 5); // PWM 2A = Pin 10, PWM 2B = Pin 11.
// CytronMD motor_3(PWM_PWM, 6, 7);  // PWM 1A = Pin 3, PWM 1B = Pin 9.
// CytronMD motor_4(PWM_PWM, 8, 9);  // PWM 2A = Pin 10, PWM 2B = Pin 11.

/*******************************************************************************
 *
 ********************************************************************************
 * DESCRIPTION:
 *
 * DriverHub code to controll 4 Motors .
 *
 *
 * CONNECTIONS:
 *
 * PICO GP0 - I2C SDA
 * PICO GP1 - I2C SCL
 *
 * PICO GP2 - Motor Driver PWM M1 Input
 * PICO GP3 - Motor Driver PWM M1 Input
 *
 * PICO GP4 - Motor Driver PWM M2 Input
 * PICO GP5 - Motor Driver PWM M2 Input
 *
 * PICO GP6 - Motor Driver PWM M3 Input
 * PICO GP7 - Motor Driver PWM M3 Input
 *
 * PICO GP8 - Motor Driver PWM M4 Input
 * PICO GP9 - Motor Driver PWM M4 Input
 *
 * PICO GP17 - Status Led Data
 *
 * PICO GND - Motor Driver GND
 *
 * AUTHOR   : Abhinav Prajapati
 * EMAIL    : <abhinavprajapati2001@gmail.com>
 *
 * v 2.1.0
 *
 *******************************************************************************/