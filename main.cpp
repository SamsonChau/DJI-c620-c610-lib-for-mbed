/*
Demo program of c620 Drivers
Mbed OS 6.13

This program will perform position control for the c620. The motor will hold its
startup position and rotate  360 deg CW then 360 deg CCW  in 60 deg increament
(on the output shaft) after pressing the User Button(PC13).

The given code provide the base setup of this driver, but you are recommanded to
change it depend on application the velocity control command is also given in
this demo.

Samson Chau
2022-4-21
*/
#include "c620.h"
#include "mbed.h"
// define the serial pin
#define TARGET_TX_PIN USBTX
#define TARGET_RX_PIN USBRX
#define serial_debug true
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);

CAN can1(PA_11, PA_12, 1000000);
c620 _c620;

DigitalIn upbutton(PC_13); // User Button
DigitalOut led(LED1);
bool state = false;

// steps and delay counter
int step = 0;            // rotation steps
int counter = 0;         // counter of the steps
int8_t id = 1;           // c620 id
int profile_vel = 16000; // profile velocity of the PID loops
int profile_tq = 15000;  // profile torque of the PID loop

Thread main_thread(osPriorityLow);  // main operation thread
Thread c620_thread(osPriorityHigh); // c620 pid thread

volatile int m1_pos = 0;

// cal the pid in 1000Hz
void m3508_pid_cal() {
  // init the driver
  _c620.c620_init(&can1);
  _c620.set_device_num(1); // config the number of motor u are using range 1~8
  // set the PID term of the 3 Ring PID loop
  // use c610 +2006 as example
  // suggest velocity mode setting
  //_c620.set_i_pid_param(0, 1.0, 0.0, 0.0);      // Torque PID W1
  //_c620.set_v_pid_param(0, 1.5, 0.0, 0.0);      // Velocity PID W1
  //_c620.set_p_pid_param(0, 0.0, 0.0, 0.0);      // position PID W1

  // suggest position mode setting
  _c620.set_i_pid_param(id, 1.3, 0.0, 0.0);   // Torque PID W1
  _c620.set_v_pid_param(id, 2.5, 0.0, 0.0);     // Velocity PID W1
  _c620.set_p_pid_param(id, 3.5, 0.5, 0.0); // position PID W1

  // set the LP filter of the desire control
  _c620.profile_velocity_CCW[id] =
      profile_vel; // Maximum is 12000 for c620 and 10000 for c610
  _c620.profile_velocity_CW[id] = - profile_vel;
  _c620.profile_torque_CCW[id] =
      profile_tq; // Maximum is 16000 for c620 and 10000 for c610
  _c620.profile_torque_CW[id] = -profile_tq;
  // set the current limit, overcurrent may destory the driver
  _c620.motor_max_current = 15000; // 10000 max for c610+2006 16000 max for c620

  // output position = motor rotation(deg).  2006 = 1:36 , 3508
  // = 1:19 rotate 180 deg
  _c620.gearatio = 19.2;     // c620
  //_c620.set_position(id, 0); // set starting position as 0
  _c620.set_velocity(id, 0);

  while (true) {
    _c620.set_velocity(id, m1_pos);
    _c620.c620_run(); // PID function of the library
    ThisThread::sleep_for(1ms);
  }
}
void change_state() {

  while (true) {

    // printf("%d\n", _c620.dt );
    // when the button was pressed, toggle the state from lock mode to rotation
    // mode
    if (!upbutton) {
      state = !state;
      wait_us(1000);
    }
    // Change state from 1~12 on each sec
    if (state) {
      led = 1;              // turn on the user led
      counter++;            // increse the counter by 1 in each loop
      if (counter >= 100) { // delay 1 sec (thread run in 100Hz, therefore
                            // counter = 100 after 1 sec)
        counter = 0;        // reset the counter
        step++;             // change the state
      }
    } else {    // if the state = false / 0
      led = 0;  // turn off the user led
      step = 0; // reset the state
    }
    switch (
        step) { // define the state action (rotate 360 CW -> CCW 60 deg per sec)
    case 1:
      m1_pos = 60;
      //_c620.set_velocity(id, 100);
      break;
    case 2:
      m1_pos = 120;
      //_c620.set_velocity(id, 200);
      break;
    case 3:
      m1_pos = 180;
      //_c620.set_velocity(id, 300);
      break;
    case 4:
      m1_pos = 240;
      //_c620.set_velocity(id, 400);
      break;
    case 5:
      m1_pos = 300;
      //_c620.set_velocity(id, 500);
      break;
    case 6:
      m1_pos = 360;
      //_c620.set_velocity(id, 200);
      break;
    case 7:
      m1_pos = 360 * 2;
      //_c620.set_velocity(id, -100);
      break;
    case 8:
      m1_pos = 240 * 2;
      //_c620.set_velocity(id, -200);
      break;
    case 9:
      m1_pos = 180 * 2;
      //_c620.set_velocity(id, -300);
      break;
    case 10:
      m1_pos = 120 * 2;
      //_c620.set_velocity(id, -400);
      break;
    case 11:
      m1_pos = 60 * 2;
      //_c620.set_velocity(id, -500);
      break;
    case 12:
      m1_pos = 0;
      //_c620.set_velocity(id, 0);
      step = 0; // reset the step = 0 in the last step
      break;
    default:
      m1_pos = 0;
      break;
    }
    // printf("pos: %d vel: %d I: %d\n",
    // _c620.get_position(id),_c620.get_velocity(id),_c620.get_current(id));
    ThisThread::sleep_for(10ms);
  }
}

int main() {

  // start the threads
  c620_thread.start(m3508_pid_cal);
  main_thread.start(change_state);
}
