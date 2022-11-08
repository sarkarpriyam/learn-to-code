---
title: Priyam Sarkar
---

# OP IN THE CHAT BOX

MY name is Priyam Sarkar


# 😎Welcome 

Hello and welcome to my free coding site.

# My Project
## Gestokar
* #### Abstract
The proposed work presents a Hand Gesture Controlled car which can be controlled by simple
hand gesture using Arduino-Nano and IMU sensors instead of using button control. The robot
and the gesture instrument are connected wirelessly through radio waves. User can interact with
the car/machine in a more friendly way due to the wireless communication. Here the most
significant device is IMU sensor. The hand position is sensed and the coordinates generated is
considered as the parameter and if necessary conditions are met, the statement prescribed in the
Arduino code is executed and the direction of the car is changed accordingly. The data is
transmitted wirelessly via a trans-receiver module to a microcontroller and the received signals
are then classified to one of six control commands for navigating a car-robot.
By the impact of this project, it will benefit various areas including applications in military, home
automation and high security bases.

* #### Keywords: Arduino Technology, Gesture, wireless module.


* #### Introduction 
Wireless cars/machines are playing an essential role in automation across all sectors like in
construction, military, medical, manufacturing, etc. In many fields, it is very risky and
complicated to handle the machines through switches or remote and sometimes the operator may
get confused in the switch control and button control, so this new concept of gesture control is
introduced to control the machine/car with the movement of hand. Gesture means the movement
of the part of the body to express an idea or a meaning. The main gesture movement or
communication is done through a hand or head. The 2 parts of the body (hand and arm) have
most attention among those people who study gestures in fact much reference for gesture
recognition. Gesture recognition enables humans to interact and communicate with the machine
without any mechanical devices. With the help of a data glove which contains an accelerometer
(inertial sensors) the human gesture are sent. Data gloves are the devices worn on the hands in
order to measure their position and their movements. Inertial sensors are used to recognize the
human gestures and those actions are replicated by the car/machine. An Arduino-Nano
microcontroller is used in the transmitter section and is coded in such a way such that it does the
required actions for the human gestures. The Arduino-Nano reads the analogue output values
from the accelerometer and converts the analogue value to respective digital value. This system
also uses a remote control system and nRF24L01 trans-receiver mode for wire-less
communication. Instead of working with both accelerometers and gyroscopes separately since
they are not as accurate as when they are combined, here MPU6050 module is used as an inertial
sensor based on MEMS (Micro Electro Mechanical Systems) technology in which both the
accelerometer and the gyroscope is integrated inside a single chip. Instead of using separate
transmitter and receiver module we are using nRF24L01 which is a single chip 2.4GHz transreceiver module. In this chip encoder, decoder, transmitter and receiver are inbuilt.
In this system, a gesture driven vehicle is developed, in which how the vehicle is moving i.e.,
control and handling is depend on user gesture. This control make switching system more
3 | P a g e
real and give more freedom to user and has wide applications in medical (gesture controlled
wheel chair for disabled people) and military applications.

* #### Literature survey:
The paper 3 focuses on the development of hand gestures recognition sensor based on
accelerometer and gyroscope for controlling arm of underwater remotely operated robots. In this
proposed work hand gesture sensor depends on accelerometer and gyroscope. Gyroscope is the
sensor which is used to capture the position of the operator hand when he is working in
underwater operated vehicle and it is attached with a hand. This system has two main parts,
ground station and aquatic remotely operated robot arm. This device can be operated without any
training. Underwater application can be easily done with this device.
The paper 1 ,2 focuses on the development of the robotic Arm by using Flex Sensor, and DC
motor which are connected to the Arduino Uno. It is controlled by processing software. These
robotic Arm are cheap and easily available which makes it free from unnecessary wire
connection, reducing its complexity. But still there is a requirement of adding new ideas and
functionality.
In paper 4 superior approach is used for adjusting the clap sound gesture commands from
kinetic sensor related to the computer and mobile phone is hooked up via RF hyper link. The
hardware is predicted on microcontroller code to keep away from unessential motion of the
robots. The clap sound is to actuate the gesture tracking mode to transport the robot and
deactivate the gesture monitoring mode after last ceasing the robot.
The paper 5 explain about the implementation and design of gesture controlled robot by using
Flex Sensor, Ultra sonic Sensor, and accelerometer connected to Microcontroller. The research
paper describes the Robot, which is controlled wirelessly via Bluetooth via hand glove. The
project is developed by using sensor, LCD Display, a Bluetooth Device, NXT Microcontroller,
Motor and Camera.



* #### Objectives:
The objective of the project is to control the direction of movement of simple car/machine with
hand gestures. In this project, user is also able to control motions of the car by wearing controller
glove and performing predefined gestures. Arduino microcontroller and MPU6050 are the two
major components. MPU6050 signals are received and assisted with wired correspondence and
the car moves depending upon the signal made by our hand. In this project we describe
approximately the gesture manage car which may be managed through our everyday hand gesture
and the program is designed by using Arduino-Nano.



* #### Methodology:
The project is categorised into 2 sections that are “transmitter section and another is receiver
section”.
The transmitter section consists of one Arduino Nano, one MPU6050 module and two RF transreceiver (nRF24L01) module. The receiver section includes RF receiver module, one motor
driver IC.
When the car/machine is powered on, the transmitter part, which consists of Arduino-Nano takes
the analog output values and convert analog value to the respective digital value and will
continuously monitor the MPU6050 sensor.
Based on the orientation of the MPU6050 sensor, this data is captured by the Arduino, which
then transmits a corresponding data to the trans-receiver nRF24L01 module.
At the receiver section, the trans-receiver nRF24L01 module receives the data and this data is
fed to the motor driver IC. Based on the data, the movement of the motors occurs, and hence the
movement of the car is defined.
Based on the input, the car/machine will behave as follows:
 Moves in forward direction
 Moves in reverse direction
 It can even turn left or right while moving forward or in reverse direction
 On the spot left or right turn to pass through the narrow space
 We have also added head light, back light and turning lights



* ### Block Diagram:


* #### Transmitter code

```cpp
#include<SPI.h>
#include<RF24.h>
#include<nrf24L01.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

RF24 radio(9,10); // object of class rf24 with object name radio with parameters CE and CSN
const byte address[]="00001"; // 5 letter character
float sndmpu[2];

void setup() {
  // put your setup code here, to run once:
radio.begin();
radio.openWritingPipe(address);
radio.setPALevel(RF24_PA_MIN);
radio.stopListening();


Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:

    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(50);

  
sndmpu[1]=a.acceleration.z;
Serial.print("sendmpudata_z: ");
Serial.print(sndmpu[1]);
Serial.println();

sndmpu[0]=a.acceleration.y;
Serial.print("sendmpudata_y: ");
Serial.print(sndmpu[0]);
Serial.println();


radio.write(&sndmpu,sizeof(sndmpu));
}
```


*  #### Recevier Code

```cpp
#include<SPI.h>
#include<RF24.h>
#include<nRF24L01.h>

RF24 radio(D4,D2); // CE and CSN of esp8266
const byte address1[]="00001";
float recvmpudata[2];

int IN1=D0;
int IN2=D1;
int IN3=D3;
int IN4=D8;

int ena=D9;
int enb=D10;

void setup() {
  // put your setup code here, to run once:

Serial.begin(115200);

pinMode(IN1,OUTPUT);
pinMode(IN2,OUTPUT);
pinMode(IN3,OUTPUT);
pinMode(IN4,OUTPUT);

pinMode(ena,OUTPUT);
pinMode(enb,OUTPUT);


radio.begin();
radio.openReadingPipe(0,address1);// reading pipe range 0 to 5
radio.setPALevel(RF24_PA_MIN);
radio.startListening();

}

void loop() {
  // put your main code here, to run repeatedly:
if(radio.available()){
  radio.read(&recvmpudata,sizeof(recvmpudata));

//forward
if((recvmpudata[0] >= -10.00)&&(recvmpudata[0] <= -4.00)){
Serial.println("forward");
digitalWrite(ena,HIGH);
digitalWrite(enb,HIGH);

digitalWrite(IN1,HIGH);
digitalWrite(IN2,LOW);
digitalWrite(IN3,HIGH);
digitalWrite(IN4,LOW);

delay(20);
}

//backward
else if((recvmpudata[0] >= +4.00)&&(recvmpudata[0] <= +10.00)){
Serial.println("backward");
digitalWrite(ena,HIGH);
digitalWrite(enb,HIGH);

digitalWrite(IN1,LOW);
digitalWrite(IN2,HIGH);
digitalWrite(IN3,LOW);
digitalWrite(IN4,HIGH);

delay(20);
}

//left
else if((recvmpudata[1] >= -10.00)&&(recvmpudata[1] <= -4.00)){
Serial.println("left");
digitalWrite(ena,HIGH);
digitalWrite(enb,HIGH);

digitalWrite(IN1,LOW);
digitalWrite(IN2,LOW);
digitalWrite(IN3,HIGH);
digitalWrite(IN4,LOW);

delay(20);
}

//right
else if((recvmpudata[1] >= +4.00)&&(recvmpudata[1] <= +10.00)){
Serial.println("right");
digitalWrite(ena,HIGH);
digitalWrite(enb,HIGH);

digitalWrite(IN1,HIGH);
digitalWrite(IN2,LOW);
digitalWrite(IN3,LOW);
digitalWrite(IN4,LOW);

delay(20);
}

//stop
else if((recvmpudata[0] >= -5.00)&&(recvmpudata[0] <= +5.00)){
Serial.println("stop");
digitalWrite(ena,LOW);
digitalWrite(enb,LOW);

digitalWrite(IN1,LOW);
digitalWrite(IN2,LOW);
digitalWrite(IN3,LOW);
digitalWrite(IN4,LOW);

delay(20);
}
else{
  Serial.println("!!!!!!!!!!Wrong Input or Output!!!!!!!!!!");
}
}
}
```