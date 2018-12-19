# include <Wire.h>
# include <Adafruit_MotorShield.h>
# include <Arduino.h>
# include <Adafruit_BLE.h>
# include <Adafruit_BluefruitLE_SPI.h>
# include "Adafruit_BLE.h"
# include "Adafruit_BluefruitLE_SPI.h"
# include "Adafruit_BluefruitLE_UART.h"

# include "BluefruitConfig.h"


# define r_motor          1
# define l_motor          2
# define l_led            9
# define r_led            11

/*  
------------
    BLE
------------
 */

// serial interface for bluetooth
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
// bluetooth function prototype from packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
// bluetooth packet buffer
extern uint8_t packetbuffer[];
char buf[60];
String BROADCAST_NAME = "Contains Nuts";
String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);
/*  
------------
   MOTORS
------------
 */
// motor shield with default address (0x60)
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(l_motor);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(r_motor);

float motor_left_throttle, motor_right_throttle;
float motor_left_mod, motor_right_mod;
bool started = false;
bool turning_left = false;
bool turning_right = false;

/*
------------
    LEDs
------------
 */
long last_update_time;
// amount of time between led updates (ms)
int refresh_time = 15;
int target_brightness_left, target_brightness_right;
int left_brightness, right_brightness;
bool left_increasing, right_increasing = false;


void setup() {
  Serial.begin(115200);
  Serial.print("Setting up...");
  // turn on LED for setup
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  // initialize motor shield
  AFMS.begin();
  
  // setup bluetooth
  BLEsetup();

  // done, turn off led
  digitalWrite(13, LOW);
  // turn on motors
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

  // initialize the LEDs
  left_brightness = right_brightness = 10;
  lightLEDs();
  last_update_time = millis();
}

void loop() {
  // 1. read from bluetooth
  readController();
  if (started){
  // 2. drive motors
  applyMotorSpeeds();
  }
  else{
    stopMotors();
  }
  // 3. update LEDs
  updateLEDs();
}

// helper for bluetooth error catching
void error(const __FlashStringHelper*err) {
    Serial.println(err);
    while (1);
}

void BLEsetup(){
  Serial.print("bluetooth...");
  if ( !ble.begin(VERBOSE_MODE) )
  {
      error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  // attempt a factory reset.. this takes a while but makes sure
  // the bluetooth starts fresh reducing the chance of an error
  if (! ble.factoryReset() ){
     error(F("Couldn't factory reset"));
  }

  // convert name change command to char array and update name
  BROADCAST_CMD.toCharArray(buf, 60);

  if(! ble.sendCommandCheckOK(buf)){
    error(F("Couldn't complete name change."));
  }

  // give time to process and then reset
  delay(250);
  if(! ble.sendCommandCheckOK("ATZ")){
    error(F("Error resetting after name change."));
  }
  delay(250);

  // confirm name change
  ble.sendCommandCheckOK("AT+GAPDEVNAME");

  // disable command echo
  ble.echo(false);
  // print ble info
  ble.info();

  Serial.println("\tready for a ble connection");
  ble.verbose(false);

  // wait for connection and flash light
  while (! ble.isConnected()) {
    digitalWrite(13, LOW);
    delay(250);
    digitalWrite(13, HIGH);
    delay(250);
  }
  
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println("DONE");
}

void readController(){
  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

  if (len == 0) {return;}
  // otherwise we got a new packet
  if (packetbuffer[1] == 'B') {
    bool pressed = packetbuffer[3]-'0';
    if (pressed){
      uint8_t buttnum = packetbuffer[2]-'0';
      if(buttnum == 1){
        // give more power to right motor
        motor_right_mod += 1.0;
        motor_left_mod -= 1.0;
        ble.println("right: "+String(motor_right_mod)+" - left: "+String(motor_left_mod));
      }

      if(buttnum == 2){
        // give more power to left motor
        motor_right_mod -= 1.0;
        motor_left_mod += 1.0;
        ble.println("right mod: "+String(motor_right_mod)+" - left mod: "+String(motor_left_mod));
      }

      if(buttnum == 3){
        ble.println("start");
        started = true;
      }

      if(buttnum == 4){
        ble.println("stop");
        started = false;
      }

      if(buttnum == 5){
        turning_left = turning_right = true;
        ble.println("forward");
        motor_right_throttle = minMax(motor_right_throttle+10.0, -255, 255);
        motor_left_throttle = minMax(motor_left_throttle+10.0, -255, 255);
        ble.println("right: "+String(motor_right_throttle)+" - left: "+String(motor_left_throttle));
      }

      if(buttnum == 6){
        turning_left = turning_right = true;
        ble.println("backward");
        motor_right_throttle = minMax(motor_right_throttle-10.0, -255, 255);
        motor_left_throttle = minMax(motor_left_throttle-10.0, -255, 255);
        ble.println("right: "+String(motor_right_throttle)+" - left: "+String(motor_left_throttle));
      }

      if(buttnum == 7){
        turning_right = false;
        turning_left = true;
        ble.println("left");
        if (motor_right_throttle < 0 && motor_left_throttle < 0){
          // we are moving backwards
          // a rightward motion increases left motor
          motor_right_throttle = minMax(motor_right_throttle+10.0, -255, 255);
        }
        else{
          motor_left_throttle = minMax(motor_left_throttle+10.0, -255, 255);
        }
      }

      if(buttnum == 8){
        turning_left = false;
        turning_right = true;
        ble.println("right");
        if (motor_right_throttle < 0 && motor_left_throttle < 0){
          // we are moving backwards
          // a rightward motion increases left motor
          motor_left_throttle = minMax(motor_left_throttle+10.0, -255, 255);
        }
        else{
          motor_right_throttle = minMax(motor_right_throttle+10.0, -255, 255);
        }
      }
    }
  }
}

void applyMotorSpeeds(){
  int  left_absolute = abs(motor_left_throttle + motor_left_mod);
  left_absolute = minMax(left_absolute, 0, 255);
  int  right_absolute = abs(motor_left_throttle + motor_left_mod);
  right_absolute = minMax(left_absolute, 0, 255);
  
  leftMotor->setSpeed(left_absolute);
  rightMotor->setSpeed(right_absolute);
  if(turning_right){
    if (motor_right_throttle < 0){
      rightMotor->run(BACKWARD);
    }
    else{
      rightMotor->run(FORWARD);
    }
  }
  else{
    rightMotor->setSpeed(0);
  }
  if (turning_left){
    if (motor_left_throttle < 0){
      leftMotor->run(BACKWARD);
    }
    else{
      leftMotor->run(FORWARD);
    }
  }
  else{
    leftMotor->setSpeed(0);
  }
}

void stopMotors(){
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
}

float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

void updateLEDs(){
  if (millis()-last_update_time >= refresh_time){
    target_brightness_left = motor_left_throttle;
    target_brightness_right = motor_right_throttle;

    if (left_brightness > motor_left_throttle || left_brightness > 175){
      left_increasing = false;
    }
    else if (left_brightness == 0){
      left_increasing = true;
    }
    if (right_brightness > motor_right_throttle || right_brightness > 175){
      right_increasing = false;
    }
    else if (right_brightness == 0){
      right_increasing = true;
    }

    if(left_increasing){
      left_brightness += (1 + motor_left_throttle/50);
    }
    else{
      left_brightness = left_brightness - (1 + motor_left_throttle/50);
    }
    if(right_increasing){
      right_brightness += (1 + motor_right_throttle/50);
    }
    else{
      right_brightness = right_brightness - (1 + motor_right_throttle/50);
    }

    right_brightness = minMax(right_brightness, 0, 175);
    left_brightness = minMax(left_brightness, 0, 175);

    lightLEDs();
    last_update_time = millis();
  }
}
void lightLEDs(){
  Serial.print(left_brightness);
  analogWrite(l_led, left_brightness);
  analogWrite(r_led, right_brightness);
}
