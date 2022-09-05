# DJI-c620-c610-lib-for-mbed
A PID demo program using the STM32F446RE to control RobotMaster C620/C610 BLDC controller with mbed OS 6.8 +
past the test with full bandwidth 8 motors and 8 drivers with 1kHz PID frequency

## PID 
Paper: https://www.ni.com/en-us/innovations/white-papers/09/can-physical-layer-and-termination-guide.html </br>


m3508/m2006 is a type of BLDC motor and c620/c610 is the motor driver for them. the way u can control is provide a current command via can bus to the driver to make the motor move. The motor driver with drive the motor wiith your current command, and feedback the position, velocity and teh current draw of the motor(rotor). PID is a control algrithm that drive the system to your destinated output from a/ the particular input by using the feedback of the driver. In this repo is the example program use the STMF446RET6 to calculate the PID and send the command to c620/c610 via CAN BUS to driver the motor to a destinated output. 

## Architecture
The PID Algro use 3 Layer control loop design, with position loop, velocity loop and current (torque) loop like the follow pictures. This control loop is base of the physics u learn in secondary school:  
1. current through coil will generate magnetic field and drive the motor to move, therefore to control the motion of the motor(rotor) we need to control the current through the motor  
2. the way to decribe a moving object can be quatify as (angular) velocity, to control the motor behaviour (motion). e.g. speed control,  we can control the velocity of the motor by controlling the current of the motor
3. motion will change the position of the object. therefore, to control the position of the (rotor)motor we needed to control the velocity by controlling the current.

With the driver feedback the real-time position, velocity, current. we can drive the motor with the feedback in a 3 layer control loop.     

![](https://github.com/SamsonChau/DJI-c620-c610-lib-for-mbed/blob/main/Current-Velocity-Position-Loops.png)

the Accelaration filter is used o prevent the error of the pid controller too large and cause the variable overflow. 

**!! if the command variable overflow will cause the driver enter error mode, and motor will rotate like crazy!**

## How to test it safe (CAN BE APPLY WITH ANY MOTOR OR HIGH TORQUE DEVICE)
Connect the Circuit like the follow:
STM32F446re -> Can transceiver (TJA1050) -> c620/c610 speed controller -> m3508/m2006 motor

if you using the c620 driver for the position control pls notice the following few things to prevent damage 

* c620 driver will only execute the last recived command from the can bus, if the can bus cable is disconnected while the running in position mode, the driver will use the last received current setting apply to the motor. 
 
__THIS WILL RESULT THE MOTOR LOSS CONTROL AND OVER-REVING, PLEASE TEST THE PID LOOP BEFORE INSTALL TO THE ACTUAL HARDWARE!!!__ 

* The position loop is may have roundcounterror over 1000rpm (at the output shaft) this may also may cause OVERREVING please test the velocity carefully 

* The connection of the CAN-BUS plug (GH 1.25) is not reliable at all pls check the hardware wire connection first if the motor have no response!

***PLEASE SAFELY SECURE THE MOTOR BEFORE YOU POWER ON ANY DRIVERS, THE POWER OF THIS PACKAGE IS MORE THAN YOU EXPECTED***
 
 **good luck if you hand hold the motor while testing**:smirk:

## Demo program
This program will perform velocity control for the c620. The motor will hold its startup 
position and rotate  360 deg CW then 360 deg CCW  in 60 deg increament (on the output shaft) after pressing the 
User Button(PC13).

The given code provide the base setup of this driver, but you are recommanded to change it
depend on application the velocity control command is also given in this demo. 
 
The details operation and usage u can refer to the c620.cpp to check the operation

```cpp
Located at c620.cpp

```

Welcome to debug this program, pls open your own branch / post an issue for any bug you found. </br>
updated at 2022 - 8 - 31
