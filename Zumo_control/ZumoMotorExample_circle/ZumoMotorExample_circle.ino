/*
   This example uses the ZumoMotors library to drive each motor on the Zumo
   forward, then backward. The yellow user LED is on when a motor should be
   running forward and off when a motor should be running backward. If a
   motor on your Zumo has been flipped, you can correct its direction by
   uncommenting the call to flipLeftMotor() or flipRightMotor() in the setup()
   function.
*/

#include <Wire.h>
#include <ZumoShield.h>

#define LED_PIN 13

ZumoMotors motors;
int num = 0;
uint32_t pre_time = 0;
uint32_t now_time = 0;
bool active_flag = true;

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
  pre_time = millis();
}

void loop()
{
  if (active_flag == true) {
    // run left motor forward
    digitalWrite(LED_PIN, HIGH);
    Serial.println("ZUMO is active-state!");

Serial.println("ZUMO turns left forward.");
    for (int speed = 0; speed <= 400; speed++)
    {
      motors.setLeftSpeed(speed);
      delay(2);
    }

    for (int speed = 400; speed >= 0; speed--)
    {
      motors.setLeftSpeed(speed);
      delay(2);
    }

    // run left motor backward
Serial.println("ZUMO turns left backward.");
    digitalWrite(LED_PIN, LOW);

    for (int speed = 0; speed >= -400; speed--)
    {
      motors.setLeftSpeed(speed);
      delay(2);
    }

    for (int speed = -400; speed <= 0; speed++)
    {
      motors.setLeftSpeed(speed);
      delay(2);
    }

    // run right motor forward
Serial.println("ZUMO turns right forward.");
    digitalWrite(LED_PIN, HIGH);

    for (int speed = 0; speed <= 400; speed++)
    {
      motors.setRightSpeed(speed);
      delay(2);
    }

    for (int speed = 400; speed >= 0; speed--)
    {
      motors.setRightSpeed(speed);
      delay(2);
    }

    // run right motor backward
Serial.println("ZUMO turns left forward.");
    digitalWrite(LED_PIN, LOW);

    for (int speed = 0; speed >= -400; speed--)
    {
      motors.setRightSpeed(speed);
      delay(2);
    }

    for (int speed = -400; speed <= 0; speed++)
    {
      motors.setRightSpeed(speed);
      delay(2);
    }

    delay (500);
    num++;
  }
  now_time = millis();

  if ((now_time - pre_time) > 300000 && (now_time - pre_time) < 600000) {
    if (active_flag == true) {
      pre_time = millis();
    }
    active_flag = false;
    Serial.println("ZUMO is rest-state.");
  }
  else if ((now_time - pre_time) > 600000) {
    if (active_flag == false) {
      pre_time = millis();
    }
    active_flag = true;
  }
}
