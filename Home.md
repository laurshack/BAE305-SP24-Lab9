# Lab 9 Report - App Development

Group members:
* Anastasia Myers
* Lauren Shackleford
* Rebecca Stacy

Date: 3/21/2024

## Summary
The purpose of this lab was to introduce us to the PID controller. PID controller is short for 'Proportional-Integral-Derivative". A PID controller is a type of feedback controller, it regulates the system by comparing actual outputs to desired outputs. By comparing actual vs. ideal outputs a PID controller can be used to help maintain a steady state in a controlled system. This is done by first measuring, then comparing, then if necessary signaling a corrective action. This process is often constant to maintain ideal outputs. 

For this lab we worked with a robot capable of sensing its distance from an object. Our goal was to utilize a PID controller so that the robot would maintain a constant distance from any obstacle that it sensed. This meant that the robot would sense its distance from any obstacle, then it would either move backwards to increase its distance or move forwards to decrease the distance. To use our PID controller we also worked with librarys found in arduino IDE.

By the end of the lab we had created a robot capable of maintaining a fixed distance from an obstacle. The robot also operated in real time. Something notable about this lab is that the working robot appeared to constantly be making adjustments to its position even if the obstacle in front of it was fixed.

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

### Part 2 - Keep Your Distance
1. 


## Results & Discussion


## Conclusion

