/**
 *
 * STM32 Bluepill position motion control example with magnetic sensor
 *
 * The same example can be ran with any STM32 board - just make sure that put right pin numbers.
 *
 */
#include <SimpleFOC.h>
#include <Wire.h>

#include "SimpleFOCDrivers.h"
#include "comms/i2c/I2CCommander.h"



// SPI Magnetic sensor instance (AS5047U example)
// MISO PA7
// MOSI PA6
// SCK PA5
//MagneticSensorSPI sensor = MagneticSensorSPI(PA4, 14, 0x3FFF);

// I2C Magnetic sensor instance (AS5600 example)
// make sure to use the pull-ups!!
// SDA PB7
// SCL PB6
MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0X0C, 4);
// Motor instance
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM(IN1, IN2, IN3, enable(optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(PA7, PB0, PB1, PA5);
// BLDCDriver6PWM(IN1_H, IN1_L, IN2_H, IN2_L, IN3_H, IN3_L, enable(optional))
//BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15, PB12);


// angle set point variable
float target_angle = 0;


// example of stm32 defining 2nd bus
TwoWire Wire1(PB11, PB10);
//TwoWire Wire0(PB7, PB6);

// commander instance
uint8_t i2c_addr = 0x60;  // can be anything you choose
I2CCommander commander(&Wire1);
// interrupt callbacks

void onReceive(int numBytes) {
  // while(1 < Wire1.available()) // loop through all but the last
  // {
  //   char c = Wire1.read(); // receive byte as a character
  // }
  commander.onReceive(numBytes);
  // while(1 < Wire1.available()) // loop through all but the last
  // {
  //   char c = Wire1.read(); // receive byte as a character
  //   //Serial.print(c);         // print the character
  // }
  // int x = Wire1.read(); 
}
void onRequest(){
  float d = 1.2;
  // commander.onReceive(1);
  // Wire1.write( 4);
  commander.onRequest();

}


void setup() {
  Wire1.begin(i2c_addr, true);     // initialize i2c in target mode
  Wire1.onReceive(onReceive);      // connect the interrupt handlers
  Wire1.onRequest(onRequest);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn off the led

  // initialise magnetic sensor hardware
  sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 20;
  // maximal velocity of the position control
  motor.velocity_limit = 40;

  // use monitoring with serial
  //Serial.begin(115200);
  // comment out if not needed
  //motor.useMonitoring(Serial);

  //Wire1.setClock(400000);          // use same speed on controller device
  commander.addMotor(&motor);     // add a motor
  commander.init(i2c_addr);       // initialize commander

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();




  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  //motor.move(target_angle);

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  //motor.move(target_angle);


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  //commander.run();
}