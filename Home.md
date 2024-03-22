# Lab 9 Report - App Development

Group members:
* Anastasia Myers
* Lauren Shackleford
* Rebecca Stacy

Date: 3/21/2024

## Summary
The purpose of this lab was to introduce us to the PID controller. PID is short for 'Proportional-Integral-Derivative". A PID controller is a type of feedback controller; it regulates the system by comparing actual outputs to desired outputs. By comparing actual vs. ideal outputs, a PID controller can be used to help maintain a steady state in a controlled system. This is done by first measuring, then comparing, and then if necessary it will signal a corrective action. This process is often constant to maintain ideal outputs. 

For this lab we worked with a robot capable of sensing its distance from an object. Our goal was to utilize a PID controller so that the robot would maintain a constant distance from any obstacle that it sensed. This meant that the robot would sense its distance from any obstacle, then it would either move backwards to increase its distance or move forwards to decrease the distance. To use our PID controller we also worked with libraries found in arduino IDE.

By the end of the lab we had created a robot capable of maintaining a fixed distance from an obstacle. The robot also operated in real time. Something notable about this lab is that the working robot was constantly making adjustments to its position even if the obstacle in front of it was fixed.

## Equipment

- Computer to run Arduino IDE
- Access to App Inventor
- Android phone
- Adaptor
- Sparkfun Inventor's Kit
  - RedBoard
  - Ultrasonic sensor HC-SR04
  - Dual TB6612FNG, H-Bridge Motor Driver
  - Two gear motors
  - Wheel for each motor

## Testing, Design, and Methods

### Prelab - Creation of Robot
1. Build the circuit below, adding the ultrasonic sensor to the circuit on the breadboard and excluding the switch. The pins for the sensor can be changed to accommodate the additional circuit (i.e. move echo to pin 6 and trig to pin 5)

|![image](https://github.com/laurshack/BAE305-SP24-Lab6/blob/main/Screenshot%202024-02-22%20131720.png)|
|:---:|
| *Figure 1.* Robot Circuit (via https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40/circuit-5b-remote-controlled-robot) |

2. Create or locate the initial code to be modified, this will consist of two main components

* Code used to gather sensor information regarding the distance of the obstacle from the robot
* Code used to move the robot back and forth

The original code used to measure distance from the sensor was from lab 6

(https://projecthub.arduino.cc/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-7cabe1)

``` c++
/*
 * HC-SR04 example sketch
 *
 * https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380
 *
 * by Isaac100
 */

const int trigPin = 9;
const int echoPin = 10;

float duration, distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
}

```
The original code used to move the robot back and forth on command was from lab 6

( https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40/circuit-5b-remote-controlled-robot)

``` c++

const int AIN1 = 13;           
const int AIN2 = 12;           
const int PWMA = 11;           

const int PWMB = 10;           
const int BIN2 = 9;           
const int BIN1 = 8;         

const int driveTime = 20;                              
const int turnTime = 8;   

String botDirection;           
String distance;               

/********************************************************************************/
void setup()
{

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.begin(9600);          

  Serial.println("Enter a direction followed by a distance.");
  Serial.println("f = forward, b = backward, r = turn right, l = turn left");
  Serial.println("Example command: f 50");
}

/********************************************************************************/
void loop()
{
  if (digitalRead(7) == LOW)
  {                                                     
    if (Serial.available() > 0)                         
    {
      botDirection = Serial.readStringUntil(' ');       
      distance = Serial.readStringUntil(' ');           

      Serial.print(botDirection);
      Serial.print(" ");
      Serial.println(distance.toInt());

      if (botDirection == "f")                         
      {
        rightMotor(200);                               
        leftMotor(200);                                 
        delay(driveTime * distance.toInt());           
        rightMotor(0);                                  
        leftMotor(0);                                  
      }
      else if (botDirection == "b")                    
      {
        rightMotor(-200);                              
        leftMotor(-200);                               
        delay(driveTime * distance.toInt());           
        rightMotor(0);                                  
        leftMotor(0);                                  
      }
      else if (botDirection == "r")                     
      {
        rightMotor(-200);                               
        leftMotor(255);                                 
        delay(turnTime * distance.toInt());             
        rightMotor(0);                                  
        leftMotor(0);                                  
      }
      else if (botDirection == "l")                  
      {
        rightMotor(255);                                
        leftMotor(-200);                                
        delay(turnTime * distance.toInt());             
        rightMotor(0);                                  
        leftMotor(0);                                   
      }
    }
  }
  else
  {
    rightMotor(0);                                  
    leftMotor(0);                                   
  }
}
/********************************************************************************/
void rightMotor(int motorSpeed)                       
{
  if (motorSpeed > 0)                                 
  {
    digitalWrite(AIN1, HIGH);                         
    digitalWrite(AIN2, LOW);                          
  }
  else if (motorSpeed < 0)                            
  {
    digitalWrite(AIN1, LOW);                          
    digitalWrite(AIN2, HIGH);                         
  }
  else                                                
  {
    digitalWrite(AIN1, LOW);                          
    digitalWrite(AIN2, LOW);                          
  }
  analogWrite(PWMA, abs(motorSpeed));                 
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        
{
  if (motorSpeed > 0)                                 
  {
    digitalWrite(BIN1, HIGH);                         
    digitalWrite(BIN2, LOW);                         
  }
  else if (motorSpeed < 0)                            
  {
    digitalWrite(BIN1, LOW);                          
    digitalWrite(BIN2, HIGH);                         
  }
  else                                                
  {
    digitalWrite(BIN1, LOW);                         
    digitalWrite(BIN2, LOW);                          
  }
  analogWrite(PWMB, abs(motorSpeed));                 
                       
  }

```

### Part 1 - PID Use
1. Install PID_V2 library by Brett Beauregard on Arduino IDE
2. Modify robot sketch to include the following:
   - Include PID_V2 library 
 
``` c++
  #include <PID_V2.h>
```
   - Define the setpoint, measurement, output, Kp, Ki, and Kd variables as type double.
``` c++
  double setpoint = 0;
  double measurement = 0;
  double output = 0;
  double Kp = 0;
  double Ki = 0;
  double Kd = 0;
```
   - Create the PID instance before setup function
``` c++
  PID myPID(&measurement, &output, &setpoint, Kp, Ki, Kd, DIRECT);
```
   - Initialize PID within setup function
``` c++
 myPID.SetTunings(Kp, Ki, Kd);
 myPID.SetMode(AUTOMATIC);
```
   - Run PID within loop function
``` c++
 myPID.Compute();
```
   - Serial write variable values
``` c++
 Serial.print(measurement);
 Serial.print(output);
 Serial.print(setpoint);
 Serial.print(Kp);
 Serial.print(Ki);
 Serial.print(Kd);
```

The full code for part 1 is as follows:
``` c++

#include <PID_v2.h>


//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor

const int driveTime = 20;      //this is the number of milliseconds that it takes the robot to drive 1 inch
                               //it is set so that if you tell the robot to drive forward 25 units, the robot drives about 25 inches

const int turnTime = 8;        //this is the number of milliseconds that it takes to turn the robot 1 degree
                               //it is set so that if you tell the robot to turn right 90 units, the robot turns about 90 degrees

                               //Note: these numbers will vary a little bit based on how you mount your motors, the friction of the
                               //surface that your driving on, and fluctuations in the power to the motors.
                               //You can change the driveTime and turnTime to make them more accurate

const int trigPin = 6;
const int echoPin = 7;

float duration, distance; 

String botDirection;           //the direction that the robot will drive in (this change which direction the two motors spin in)
//String distance;               //the distance to travel in each direction
String speed;                  //the speed that the robot will travel

double measurement = 0;
double output = 0;
double setpoint = 0;
double Kp = 0;
double Ki = 0;
double Kd = 0;

PID myPID(&measurement, &output, &setpoint, Kp, Ki, Kd, DIRECT); 

/********************************************************************************/
void setup()
{
//  pinMode(switchPin, INPUT_PULLUP);   //set this as a pullup to sense whether the switch is flipped

  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.begin(9600);           //begin serial communication with the computer

  //prompt the user to enter a command
  Serial.println("Enter a direction followed by a distance and a speed.");
  Serial.println("f = forward, b = backward, r = turn right, l = turn left");
  Serial.println("1 = low, 2 = medium, 3 = high");
  Serial.println("Example command: f 1");

  pinMode(trigPin, OUTPUT);  
  pinMode(echoPin, INPUT); 

  Serial.print(measurement);
  Serial.print(output);
  Serial.print(setpoint);
  Serial.print(Kp);
  Serial.print(Ki);
  Serial.print(Kd);
  
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC);
}

/********************************************************************************/
void loop()
{
  if (true)
  {                                                     //if the switch is in the ON position
    if (Serial.available() > 0)                         //if the user has sent a command to the RedBoard
    {
      botDirection = Serial.readStringUntil(' ');       //read the characters in the command until you reach the first space
      //distance = Serial.readStringUntil(' ');           //read the characters in the command until you reach the second space
      speed = Serial.readStringUntil(' ');              //read the characters in the command until you reach the third space
      //print the command that was just received in the serial monitor
      Serial.print(botDirection);
      Serial.print(" ");
      //Serial.println(distance.toInt());
      //Serial.print(" ");
      Serial.println(speed);


      int mSpeed = 0;
      if (speed == "1") {
        mSpeed = 100; // Set low speed
      } else if (speed == "2") {
        mSpeed = 150; // Set medium speed
      } else if (speed == "3") {
        mSpeed = 200; // Set high speed
      }

      myPID.Compute ();
      Serial.print(Kp);
      Serial.print(Ki);
      Serial.print(Kd);


      if (botDirection == "f")                         //if the entered direction is forward
      {
        rightMotor(mSpeed);                                //drive the right wheel forward
        leftMotor(mSpeed);                                 //drive the left wheel forward
        //delay(driveTime * distance.toInt());            //drive the motors long enough travel the entered distance
        //rightMotor(0);                                  //turn the right motor off
        //leftMotor(0);                                   //turn the left motor off
      }
      else if (botDirection == "b")                    //if the entered direction is backward
      {
        rightMotor(-mSpeed);                               //drive the right wheel forward
        leftMotor(-mSpeed);                                //drive the left wheel forward
        //delay(driveTime * distance.toInt());            //drive the motors long enough travel the entered distance
        //rightMotor(0);                                  //turn the right motor off
        //leftMotor(0);                                   //turn the left motor off
      }
      else if (botDirection == "r")                     //if the entered direction is right
      {
        rightMotor(mSpeed);                               //drive the right wheel forward
        leftMotor(-mSpeed);                                 //drive the left wheel forward
        //delay(turnTime * distance.toInt());             //drive the motors long enough turn the entered distance
        //rightMotor(0);                                  //turn the right motor off
        //leftMotor(0);                                   //turn the left motor off
      }
      else if (botDirection == "l")                   //if the entered direction is left
      {
        rightMotor(-mSpeed);                                //drive the right wheel forward
        leftMotor(mSpeed);                                //drive the left wheel forward
        //delay(turnTime * distance.toInt());             //drive the motors long enough turn the entered distance
        //rightMotor(0);                                  //turn the right motor off
        //leftMotor(0);                                   //turn the left motor off
      }
    }
  }
  else
  {
    rightMotor(0);                                  //turn the right motor off
    leftMotor(0);                                   //turn the left motor off
  }
}
/********************************************************************************/



void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed

                           //turn the left motor off
  }
```

### Part 2 - Keep Your Distance
1. Write a function to move the robot forward and backward depending on distance from an object. 
2. Tune the system as needed by adjusting the values of Kp, Ki, and Kd.

The final code should look similar to the following:
``` c++
#include <PID_v2.h>


//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor

const int driveTime = 20;      //this is the number of milliseconds that it takes the robot to drive 1 inch
                               //it is set so that if you tell the robot to drive forward 25 units, the robot drives about 25 inches

const int turnTime = 8;        //this is the number of milliseconds that it takes to turn the robot 1 degree
                               //it is set so that if you tell the robot to turn right 90 units, the robot turns about 90 degrees

                               //Note: these numbers will vary a little bit based on how you mount your motors, the friction of the
                               //surface that your driving on, and fluctuations in the power to the motors.
                               //You can change the driveTime and turnTime to make them more accurate

const int trigPin = 6;
const int echoPin = 7;

float duration, distance; 

String botDirection;           //the direction that the robot will drive in (this change which direction the two motors spin in)
//String distance;               //the distance to travel in each direction
String mSpeedStr;                  //the speed that the robot will travel

int mSpeed;

double measurement = 0;
double output = 0;
double setpoint = 0;
double Kp = 0;
double Ki = 0;
double Kd = 0;

PID myPID(&measurement, &output, &setpoint, Kp, Ki, Kd, DIRECT); 

/********************************************************************************/
void setup()
{
//  pinMode(switchPin, INPUT_PULLUP);   //set this as a pullup to sense whether the switch is flipped

  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.begin(9600);           //begin serial communication with the computer

  //prompt the user to enter a command
  Serial.println("Enter a direction followed by a distance and a speed.");
  Serial.println("f = forward, b = backward, r = turn right, l = turn left");
  Serial.println("1 = low, 2 = medium, 3 = high");
  Serial.println("Example command: f 1");

  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT); 

 
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC);
}

/********************************************************************************/
void loop()
{
  if (true)
  {                                                     //if the switch is in the ON position
    if (Serial.available() > 0)                         //if the user has sent a command to the RedBoard
    {
      botDirection = Serial.readStringUntil(' ');       //read the characters in the command until you reach the first space
      //distance = Serial.readStringUntil(' ');           //read the characters in the command until you reach the second space
      //speed = Serial.readStringUntil(' ');              //read the characters in the command until you reach the third space
      //print the command that was just received in the serial monitor
      Serial.print(botDirection);
      Serial.print(" ");
      //Serial.println(distance.toInt());
      //Serial.print(" ");
      //Serial.println(speed);


      int mSpeed = 0;
      //if (speed == "1") {
      //  mSpeed = 100; // Set low speed
      //} else if (speed == "2") {
      //  mSpeed = 150; // Set medium speed
      //} else if (speed == "3") {
      //  mSpeed = 200; // Set high speed
      }

      myPID.Compute();
  Serial.print(measurement);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(Kp);
  Serial.print(",");
  Serial.print(Ki);
  Serial.print(",");
  Serial.println(Kd);

      if (Serial.available() > 0)
      {
        //botDirection = Serial.readStringUntil(' ');
        //mSpeedStr = Serial.readStringUntil('');
        //mSpeed = mSpeedStr.toInt();
      }

      if (true)
      {
        mSpeed = output;
        if (measurement > setpoint)
        {
          botDirection = "f";
        }
      


      if (botDirection == "f")                         //if the entered direction is forward
      {
        rightMotor(mSpeed);                                //drive the right wheel forward
        leftMotor(mSpeed);                                 //drive the left wheel forward
        //delay(driveTime * distance.toInt());            //drive the motors long enough travel the entered distance
        //rightMotor(0);                                  //turn the right motor off
        //leftMotor(0);                                   //turn the left motor off
      }
      else if (botDirection == "b")                    //if the entered direction is backward
      {
        rightMotor(-mSpeed);                               //drive the right wheel forward
        leftMotor(-mSpeed);                                //drive the left wheel forward
        //delay(driveTime * distance.toInt());            //drive the motors long enough travel the entered distance
        //rightMotor(0);                                  //turn the right motor off
        //leftMotor(0);                                   //turn the left motor off
      }
      else if (botDirection == "r")                     //if the entered direction is right
      {
        rightMotor(mSpeed);                               //drive the right wheel forward
        leftMotor(-mSpeed);                                 //drive the left wheel forward
        //delay(turnTime * distance.toInt());             //drive the motors long enough turn the entered distance
        //rightMotor(0);                                  //turn the right motor off
        //leftMotor(0);                                   //turn the left motor off
      }
      else if (botDirection == "l")                   //if the entered direction is left
      {
        rightMotor(-mSpeed);                                //drive the right wheel forward
        leftMotor(mSpeed);                                //drive the left wheel forward
        //delay(turnTime * distance.toInt());             //drive the motors long enough turn the entered distance
        //rightMotor(0);                                  //turn the right motor off
        //leftMotor(0);                                   //turn the left motor off
      }
    }
  }
  else
  {
    rightMotor(0);                                  //turn the right motor off
    leftMotor(0);                                   //turn the left motor off
  }
}
/********************************************************************************/



void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed

                           //turn the left motor off
  }

```

## Discussion of Results

While we were able to implement our PID control, we had some issues with overshoot in part 2. The robot was able to sense an object in front of it and stop moving forward if the object was less than the set distance from the sensor. It could then move backwards to maintain that distance if the object was moved closer to the sensor. However, if the object was stationary, it would still continue to move back and forth. While this is expected to some extent with the use of a PID controller, our robot was consistently moving a few centimeters back and forth, which is more overshoot than should be present. We were able to slightly reduce this distance by adjusting the Ki value, but not to the expected amount. 

## Conclusion

This lab introduced us to the PID controller. As previously mentioned PID stands for 'Proportional, Integral, Derivative'. Proportional is related to the amount of error correlated to the current measurement. Error to a PID is the amount that the measured output differs from the ideal output. Integral observes previous errors working to eliminate steady-state error. The derivative term works to predict error by assessing the measured error rate of change. This is done to resist an overshoot in the systems response.

The PID controller is widely used in industry to maintain controlled conditions. For example, a common use of a PID controller is to sustain a steady temperature, this use can be applied in many different ways. A few ways might be to maintain temperature within a food processing plant or an oven, even the temperature in our homes. One application that we had discussed in class is also the use of a PID for cruise control within a car. The takeaway from this was that PID controllers are ingrained within our day-to-day lives. 

Something interesting about this lab was that the controlled car would continue to move back and forth even when placed in front of an unmoving obstacle. This was a good visualization of "overshoot" where the PID controller overcorrects and causes the error to oscillate. This also occurs because the car does not stop moving so it must move back and forth to compensate and maintain the steady distance.

The main takeaway of this lab was how to use a PID controller to create a constant system output. And that a PID controller works by comparing the measured actual value to the ideal value, where the difference is known as the 'error'. The PID controller works continuously in real-time to reduce the error.
