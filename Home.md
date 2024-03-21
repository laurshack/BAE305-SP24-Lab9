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
  double setpoint = ;
  double measurement = ;
  double output = ;
  double Kp = ;
  double Ki = ;
  double Kd = ;
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
The full code for part 1 is as follows:
``` c++
```

### Part 2 - Keep Your Distance
1. Write a function to move the robot forward and backward depending on distance from an object. 
2. Tune the system as need by adjusting the values of Kp, Ki, and Kd.

The final code should look like the following:
``` c++
```

## Discussion of Results


## Conclusion

This lab introduced us to the PID controller. As previously mentioned PID stands for 'Proportional, Integral, Derivative'. Proportional is related to the amount of error correlated to the current measurement. Error to a PID is the amount that the measured output differs from the ideal output. Integral observes previous errors working to eliminate steady-state error. The derivative term works to predict error by assessing the measured error rate of change. This is done to resist an overshoot in the systems response.

The PID controller is widely used in industry to maintain controlled conditions. For example, a common use of a PID controller is to sustain a steady temperature, this use can be applied in many different ways. A few ways might be to maintain temperature within a food processing plant or an oven, even the temperature in our homes. One application that we had discussed in class is also the use of a PID for cruise control within a car. The takeaway from this was that PID controllers are ingrained within our day-to-day lives. 

Something interesting about this lab was that the controlled car would continue to move back and forth even when placed in front of an unmoving obstacle. This was a good visualization of "overshoot" where the PID controller overcorrects and causes the error to oscillate. This also occurs because the car does not stop moving so it must move back and forth to compensate and maintain the steady distance.

The main takeaway of this lab was how to use a PID controller to create a constant system output. And that a PID controller works by comparing the measured actual value to the ideal value, where the difference is known as the 'error'. The PID controller works continuously in real-time to reduce the error.
