# Lab 9 Report - App Development

Group members:
* Anastasia Myers
* Lauren Shackleford
* Rebecca Stacy

Date: 3/21/2024

## Summary
The purpose of this lab is

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


## Results



## Discussion


## Conclusion

