#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "comms/i2c/I2CCommanderMaster.h"


#define TARGET_I2C_ADDRESS 0x60
I2CCommanderMaster commander;

float angle_read[1];
uint8_t target = 0;
int enabeled = 0;
uint8_t reg_report[32];

SimpleFOCRegister current_reg = REG_STATUS;


void onReceive(int numBytes) {
  Serial.println(numBytes);
  while(Wire.available())         // slave may send less than requested
  {
    char c = Wire.read();         // receive a byte as character
    Serial.print(c);              // print the character
  }
}

void setup() {
  Serial.begin(115200);
    
    // ...other setup code

    //Wire.setClock(400000);          // use same speed on target device!
    Wire.begin();                   // initialize i2c in controller mode
    commander.addI2CMotors(TARGET_I2C_ADDRESS, 1, &Wire);            // add target device, it has 1 motor
    //commander.addI2CMotors(TARGET_I2C_ADDRESS2, 1);         // you could add another target device on the same bus
    //commander.addI2CMotors(TARGET_I2C_ADDRESS, 1, &wire2);  // or on a different i2c bus
    commander.init();               // init commander
    // Wire.onReceive(onReceive);      // connect the interrupt handlers
    //Wire.onRequest(onRequest);

}

void loop(){
  delay(4000);
  Serial.println("\n\n\nStarting MasterCommanderI2C cycle:");
  Serial.println("1. Read current target:");
  int sz = sizeof(target);
  int ret = commander.readRegister(0, current_reg, &target, sz);
  if(!ret){
    Serial.println("--> Reading failed!");
  }
  else{
    Serial.print("--> Reading succes! The current target is: ");
    Serial.println(target);
    Serial.println("2. Writing new target:");
    if(target>10){
      target = 0;
    }
    target = target + 1;
    int ret2 = commander.writeRegister(0, current_reg, &target, 4);
    if(ret2){
      Serial.print("--> Writing succes! The value of the new target should be: ");
      Serial.println (target);
      Serial.println("3. Reading current target again to check if the value changed.");
      int ret = commander.readRegister(0, current_reg, &target, sz);
      if(!ret){
        Serial.println("--> Reading failed!");
      }
      else{
        Serial.print("--> Reading succes! The current target is: ");
        Serial.println(target);
      }
    }
    else{
      Serial.println("--> Writing failed!");
    }
  }
  // Serial.println(sz);
  // if(!enabeled){
  //   enabeled = 1;
  //   int ret1 = commander.writeRegister(0, REG_ENABLE, &enabeled, 4);
  //   Serial.println(REG_ENABLE);
  // }
  // int ret2 = commander.writeRegister(0, REG_TARGET, &target, 4);
  //int ret = commander.readRegister(0, REG_REPORT, reg_report, sz);
  // uint8_t ret = Wire.requestFrom(TARGET_I2C_ADDRESS, 4);
  // while(Wire.available())    // slave may send less than requested
  // { 
  //   char c = Wire.read(); // receive a byte as character
  //   Serial.print(c);         // print the character
  // }
  // Serial.println(ret2);
  // Wire.beginTransmission(TARGET_I2C_ADDRESS);
  // Wire.write(1);
  // uint8_t error = Wire.endTransmission(true);
  // Serial.printf("endTransmission: %u\n", error);
  //target = target + 1;
  //Serial.println(angle_read[0]);
}

